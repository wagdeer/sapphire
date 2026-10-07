#include <faiss/IndexBinary.h>
#include <faiss/IndexBinaryFlat.h>
#include <faiss/IndexBinaryIVF.h>
#include <faiss/IndexIDMap.h>
#include <faiss/impl/IDSelector.h>
#include <faiss/impl/io.h>
#include <faiss/index_io.h>
#include <faiss/utils/random.h>
#include <fcntl.h>
#include <unistd.h>
#include <zlib.h>

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include "backend/visual/faiss/descriptor_archive.hpp"
#include <stdexcept>

#include "backend/visual/faiss/faiss_thread.hpp"

namespace sapphire::mapping {
namespace {
using Id = faiss::idx_t;

struct Best {
  int d = 257, second = 257;
  uint32_t point = UINT32_MAX;
  void add(int distance, uint32_t p) {
    if (point == p) return;
    if (distance < d) {
      second = d;
      d = distance;
      point = p;
    } else
      second = std::min(second, distance);
  }
};
constexpr size_t lists = 128, training_min = 8192, sample_size = lists * 512, neighbors = 64;
}  // namespace
struct DescriptorArchive::Impl {
  struct Slot {
    std::array<uint8_t, 32> descriptor;
    int node = 0;
    uint32_t point = 0, appearance = 0;
    size_t work = 0;
  };
  struct Node {
    uint64_t fingerprint;
    std::vector<Id> ids;
    size_t work = 0;
  };
  struct Work {
    int node = 0;
    Best best;
    Id best_slot = -1;
    uint64_t seen = 0;
    float score = 0;
    std::vector<std::pair<uint32_t, Id>> proposals;
  };
  std::vector<Work> work;
  std::vector<size_t> recycled_work, touched;
  uint64_t epoch = 0;
  std::vector<Slot> slots;
  std::vector<Id> recycled;
  std::map<int, Node> nodes;
  std::unique_ptr<faiss::IndexBinary> index;
  std::future<std::vector<uint8_t>> training;
  bool ivf = false;
  bool restored = false;
  uint64_t revision = 0, searches = 0;
  std::vector<int32_t> distances;
  std::vector<Id> labels;
  std::vector<Best> reverse;
  Impl() {
    auto flat = std::make_unique<faiss::IndexBinaryIDMap2>(new faiss::IndexBinaryFlat(256));
    flat->own_fields = true;
    index = std::move(flat);
  }
  void start_training() {
    if (ivf || training.valid() || index->ntotal < Id(training_min)) return;
    // Match Faiss's standard seeded, without-replacement training subsample.
    // The immutable worker payload is bounded to 2 MiB, regardless of map size.
    std::vector<Id> live;
    live.reserve(index->ntotal);
    for (const auto& [id, node] : nodes) live.insert(live.end(), node.ids.begin(), node.ids.end());
    const size_t count = std::min(sample_size, live.size());
    std::vector<int> permutation;
    if (live.size() > count) {
      if (live.size() > size_t(std::numeric_limits<int>::max())) throw std::length_error("Faiss training population exceeds permutation limit");
      permutation.resize(live.size());
      faiss::rand_perm(permutation.data(), permutation.size(), 1234);
    }
    std::vector<uint8_t> sample;
    sample.reserve(count * 32);
    for (size_t i = 0; i < count; ++i) {
      const auto& descriptor = slots[live[permutation.empty() ? i : permutation[i]]].descriptor;
      sample.insert(sample.end(), descriptor.begin(), descriptor.end());
    }
    training = std::async(std::launch::async, [sample = std::move(sample)] {
      SingleThread thread;
      faiss::IndexBinaryFlat quantizer(256);
      faiss::IndexBinaryIVF trainer(&quantizer, 256, lists);
      trainer.cp.niter = 20;
      trainer.cp.seed = 1234;
      trainer.cp.max_points_per_centroid = 512;
      trainer.train(sample.size() / 32, sample.data());
      return std::vector<uint8_t>(quantizer.xb.data(), quantizer.xb.data() + quantizer.xb.size());
    });
  }
};
DescriptorArchive::DescriptorArchive() : impl_(std::make_unique<Impl>()) {}
DescriptorArchive::~DescriptorArchive() = default;
void DescriptorArchive::clear() { impl_ = std::make_unique<Impl>(); }
uint64_t DescriptorArchive::fingerprint(const scene::FeatureMap& map) {
  uint64_t hash = 1469598103934665603ULL;
  auto add = [&](const void* data, size_t size) {
    auto p = static_cast<const uint8_t*>(data);
    while (size--) {
      hash ^= *p++;
      hash *= 1099511628211ULL;
    }
  };
  auto vocabulary = map.vocabulary_hash();
  add(&vocabulary, sizeof(vocabulary));
  for (const auto& p : map.points()) {
    add(&p.id, sizeof(p.id));
    const uint8_t has_position = p.has_position;
    add(&has_position, sizeof(has_position));
    add(&p.position.x, sizeof(float));
    add(&p.position.y, sizeof(float));
    add(&p.position.z, sizeof(float));
    add(&p.appearance_count, sizeof(p.appearance_count));
    for (uint32_t a = 0; a < p.appearance_count; ++a) add(p.appearances[a].descriptor.data(), 32);
  }
  return hash;
}
void DescriptorArchive::erase(int id) {
  auto& s = *impl_;
  auto it = s.nodes.find(id);
  if (it == s.nodes.end()) return;
  faiss::IDSelectorArray select(it->second.ids.size(), it->second.ids.data());
  s.index->remove_ids(select);
  for (Id slot : it->second.ids) {
    s.slots[slot].node = 0;
    s.recycled.push_back(slot);
  }
  s.work[it->second.work].node = 0;
  s.recycled_work.push_back(it->second.work);
  s.nodes.erase(it);
  ++s.revision;
}
void DescriptorArchive::upsert(int id, const scene::FeatureMap& map) {
  if (id <= 0) throw std::invalid_argument("Descriptor archive requires a positive node ID");
  auto& s = *impl_;
  const auto hash = fingerprint(map);
  auto existing = s.nodes.find(id);
  if (existing != s.nodes.end() && existing->second.fingerprint == hash) return;
  erase(id);
  Impl::Node node{hash, {}};
  if (s.recycled_work.empty()) {
    node.work = s.work.size();
    s.work.emplace_back();
  } else {
    node.work = s.recycled_work.back();
    s.recycled_work.pop_back();
  }
  s.work[node.work].node = id;
  std::vector<uint8_t> data;
  data.reserve(map.descriptors().size() * 32);
  node.ids.reserve(map.descriptors().size());
  for (const auto& ref : map.descriptors()) {
    Id slot;
    if (s.recycled.empty()) {
      slot = s.slots.size();
      s.slots.emplace_back();
    } else {
      slot = s.recycled.back();
      s.recycled.pop_back();
    }
    const auto& descriptor = map.points()[ref.landmark].appearances[ref.appearance].descriptor;
    s.slots[slot] = {descriptor, id, ref.landmark, ref.appearance, node.work};
    node.ids.push_back(slot);
    data.insert(data.end(), descriptor.begin(), descriptor.end());
  }
  SingleThread thread;
  if (!node.ids.empty()) s.index->add_with_ids(node.ids.size(), data.data(), node.ids.data());
  s.nodes.emplace(id, std::move(node));
  ++s.revision;
}
bool DescriptorArchive::poll_training(bool wait) {
  auto& s = *impl_;
  s.start_training();
  if (!s.training.valid() || (!wait && s.training.wait_for(std::chrono::seconds(0)) != std::future_status::ready)) return false;
  auto centroids = s.training.get();
  SingleThread thread;
  auto quantizer = std::make_unique<faiss::IndexBinaryFlat>(256);
  quantizer->add(lists, centroids.data());
  auto index = std::make_unique<faiss::IndexBinaryIVF>(quantizer.get(), 256, lists);
  index->own_fields = true;
  quantizer.release();
  index->is_trained = true;
  index->nprobe = 4;
  index->use_heap = false;
  index->set_direct_map_type(faiss::DirectMap::Hashtable);
  // Add in bounded batches; edits made while training are already reflected in slots.
  std::vector<uint8_t> data;
  std::vector<Id> ids;
  data.reserve(1024 * 32);
  ids.reserve(1024);
  auto flush = [&] {
    if (!ids.empty()) index->add_with_ids(ids.size(), data.data(), ids.data());
    ids.clear();
    data.clear();
  };
  for (size_t i = 0; i < s.slots.size(); ++i)
    if (s.slots[i].node) {
      ids.push_back(i);
      data.insert(data.end(), s.slots[i].descriptor.begin(), s.slots[i].descriptor.end());
      if (ids.size() == 1024) flush();
    }
  flush();
  s.index = std::move(index);
  s.ivf = true;
  ++s.revision;
  return true;
}
DescriptorArchive::Results DescriptorArchive::search(const features::FeatureBlock& query, int max_distance, float ratio) {
  query.validate();
  if (max_distance < 0 || max_distance > 256 || !(ratio > 0 && ratio <= 1)) throw std::invalid_argument("Invalid archive match gates");
  auto& s = *impl_;
  Results result;
  ++s.searches;
  if (s.nodes.empty() || query.descriptors.empty() || !s.index->ntotal) return result;

  SingleThread thread;
  if (s.ivf) static_cast<faiss::IndexBinaryIVF*>(s.index.get())->max_codes = s.index->ntotal;
  const size_t nq = query.keypoints.size(), k = std::min(neighbors, size_t(s.index->ntotal));
  s.distances.resize(nq * k);
  s.labels.resize(nq * k);
  for (size_t first = 0; first < nq; first += 32)
    s.index->search(std::min(size_t(32), nq - first), query.descriptors.ptr(first), k, s.distances.data() + first * k, s.labels.data() + first * k);
  // One reverse entry per descriptor slot (one appearance per landmark today).
  static_assert(scene::appearance_capacity == 1, "Reverse matching needs a shared landmark slot for multiple appearances");
  s.reverse.assign(s.slots.size(), Best{});
  for (auto& w : s.work) {
    w.score = 0;
    w.proposals.clear();
  }
  for (uint32_t q = 0; q < nq; ++q) {
    if (++s.epoch == 0) {
      for (auto& w : s.work) w.seen = 0;
      ++s.epoch;
    }
    s.touched.clear();
    for (size_t j = 0; j < k; ++j) {
      Id slot = s.labels[q * k + j];
      if (slot < 0) break;
      const auto& ref = s.slots.at(slot);
      const int d = s.distances[q * k + j];
      auto& w = s.work[ref.work];
      if (w.seen != s.epoch) {
        w.seen = s.epoch;
        w.best = Best{};
        s.touched.push_back(ref.work);
      }
      w.best.add(d, ref.point);
      if (w.best.point == ref.point) w.best_slot = slot;
      s.reverse[slot].add(d, q);
    }
    const int bound = s.labels[q * k + k - 1] < 0 ? 257 : s.distances[q * k + k - 1];
    for (size_t n : s.touched) {
      auto& w = s.work[n];
      const auto& b = w.best;
      if (b.d > max_distance) continue;
      w.score += (max_distance + 1. - b.d) / (max_distance + 1.);
      const int second = b.second == 257 ? bound : b.second;
      if (b.d < second && b.d <= ratio * second) w.proposals.push_back({q, w.best_slot});
    }
  }
  for (const auto& w : s.work)
    if (w.node && w.score > 0) {
      const auto& meta = s.nodes.at(w.node);
      auto& candidate = result[w.node];
      candidate.fingerprint = meta.fingerprint;
      candidate.score = w.score / std::sqrt(double(nq) * meta.ids.size());
      for (auto [q, slot] : w.proposals) {
        const auto& reverse = s.reverse[slot];
        const auto& ref = s.slots[slot];
        if (reverse.point == q && reverse.d < reverse.second) candidate.matches.push_back({ref.point, q, ref.appearance, reverse.d});
      }
    }
  return result;
}
uint64_t DescriptorArchive::revision() const { return impl_->revision; }
DescriptorArchive::Stats DescriptorArchive::stats() const {
  const auto& s = *impl_;
  size_t bytes = s.slots.capacity() * sizeof(Impl::Slot) + s.recycled.capacity() * sizeof(Id) + s.reverse.capacity() * sizeof(Best) +
                 s.distances.capacity() * sizeof(int32_t) + s.labels.capacity() * sizeof(Id) + size_t(s.index->ntotal) * (32 + sizeof(Id) * 3);
  for (const auto& [id, node] : s.nodes) bytes += sizeof(node) + node.ids.capacity() * sizeof(Id);
  bytes += s.work.capacity() * sizeof(Impl::Work) + (s.recycled_work.capacity() + s.touched.capacity()) * sizeof(size_t);
  for (const auto& w : s.work) bytes += w.proposals.capacity() * sizeof(std::pair<uint32_t, Id>);
  return {s.nodes.size(), size_t(s.index->ntotal), s.slots.size(), bytes, s.searches, s.revision, s.ivf, s.restored};
}

DescriptorArchive::IndexNodeMetadata DescriptorArchive::node_metadata() const {
  IndexNodeMetadata result;
  for (const auto& [id, node] : impl_->nodes) result.emplace(id, std::make_pair(node.fingerprint, node.ids.size()));
  return result;
}
namespace {
// One atomic file contains both the native Faiss index and its application mapping.
// Bump the format when indexing, fingerprint or matching identity semantics change.
constexpr uint64_t cache_magic = 0x3145535349414645ULL;
constexpr uint64_t cache_version = 1;
constexpr uint64_t faiss_version = FAISS_VERSION_MAJOR * 10000 + FAISS_VERSION_MINOR * 100 + FAISS_VERSION_PATCH;
constexpr size_t cache_limit = size_t(1) << 30;  // Optional cache; larger archives rebuild normally.
void put64(std::vector<uint8_t>& bytes, uint64_t value) {
  for (int i = 0; i < 8; ++i) bytes.push_back(uint8_t(value >> (i * 8)));
}
struct IndexCacheReader {
  const std::vector<uint8_t>& bytes;
  size_t pos = 0;
  uint64_t take() {
    if (pos > bytes.size() || bytes.size() - pos < 8) throw std::runtime_error("truncated index cache");
    uint64_t value = 0;
    for (int i = 0; i < 8; ++i) value |= uint64_t(bytes[pos++]) << (i * 8);
    return value;
  }
};
}  // namespace
void DescriptorArchive::save_cache(const std::string& path) const {
  const auto& s = *impl_;
  faiss::VectorIOWriter writer;
  faiss::write_index_binary(s.index.get(), &writer);
  std::vector<uint8_t> bytes;
  for (uint64_t value : {cache_magic, cache_version, faiss_version, uint64_t(sizeof(size_t)), uint64_t(lists), uint64_t(neighbors), uint64_t(s.ivf),
                         uint64_t(s.slots.size()), uint64_t(s.index->ntotal), uint64_t(s.nodes.size())})
    put64(bytes, value);
  for (const auto& [id, node] : s.nodes) {
    put64(bytes, id);
    put64(bytes, node.fingerprint);
    put64(bytes, node.ids.size());
  }
  for (size_t slot = 0; slot < s.slots.size(); ++slot)
    if (s.slots[slot].node) {
      const auto& ref = s.slots[slot];
      put64(bytes, slot);
      put64(bytes, ref.node);
      put64(bytes, ref.point);
      put64(bytes, ref.appearance);
      bytes.insert(bytes.end(), ref.descriptor.begin(), ref.descriptor.end());
    }
  put64(bytes, writer.data.size());
  bytes.insert(bytes.end(), writer.data.begin(), writer.data.end());
  if (bytes.size() > cache_limit - 8) throw std::runtime_error("index cache exceeds cache size limit");
  put64(bytes, crc32(0, bytes.data(), bytes.size()));
  std::string temporary = path + ".tmp-XXXXXX";
  int fd = mkstemp(temporary.data());
  if (fd < 0) throw std::runtime_error("cannot create index cache temporary file");
  try {
    size_t pos = 0;
    while (pos < bytes.size()) {
      const auto n = ::write(fd, bytes.data() + pos, bytes.size() - pos);
      if (n < 0 && errno == EINTR) continue;
      if (n <= 0) throw std::runtime_error("index cache write failed");
      pos += size_t(n);
    }
    if (::fsync(fd) != 0) throw std::runtime_error("index cache sync failed");
    const int status = ::close(fd);
    fd = -1;
    if (status != 0) throw std::runtime_error("index cache close failed");
    std::filesystem::rename(temporary, path);
    auto parent = std::filesystem::path(path).parent_path();
    if (parent.empty()) parent = ".";
    int directory = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY);
    if (directory < 0) throw std::runtime_error("index cache directory open failed");
    const int synced = ::fsync(directory);
    ::close(directory);
    if (synced != 0) throw std::runtime_error("index cache directory sync failed");
  } catch (...) {
    if (fd >= 0) ::close(fd);
    ::unlink(temporary.c_str());
    throw;
  }
}
bool DescriptorArchive::restore_cache(const std::string& path, const IndexNodeMetadata& expected, std::string& reason) {
  try {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
      reason = "index cache unavailable";
      return false;
    }
    const auto length = file.tellg();
    if (length < 96 || uint64_t(length) > cache_limit) throw std::runtime_error("invalid index cache size");
    std::vector<uint8_t> bytes(static_cast<size_t>(length));
    file.seekg(0);
    if (!file.read(reinterpret_cast<char*>(bytes.data()), bytes.size())) throw std::runtime_error("index cache read failed");
    IndexCacheReader checksum{bytes, bytes.size() - 8};
    if (checksum.take() != crc32(0, bytes.data(), bytes.size() - 8)) throw std::runtime_error("index cache checksum mismatch");
    bytes.resize(bytes.size() - 8);
    IndexCacheReader reader{bytes};
    if (reader.take() != cache_magic || reader.take() != cache_version || reader.take() != faiss_version || reader.take() != sizeof(size_t) ||
        reader.take() != lists || reader.take() != neighbors)
      throw std::runtime_error("index cache format or Faiss version mismatch");
    const auto ivf = reader.take(), slots = reader.take(), live = reader.take(), nodes = reader.take();
    size_t expected_live = 0;
    for (const auto& [id, node] : expected) expected_live += node.second;
    if (ivf > 1 || live != expected_live || nodes != expected.size() || live > slots || slots > cache_limit / sizeof(Impl::Slot) ||
        nodes > bytes.size() / 24)
      throw std::runtime_error("index cache population mismatch");
    auto restored = std::make_unique<Impl>();
    restored->slots.resize(slots);
    for (size_t i = 0; i < nodes; ++i) {
      const auto id = reader.take(), fingerprint = reader.take(), count = reader.take();
      const auto node = id <= INT_MAX ? expected.find(int(id)) : expected.end();
      if (id == 0 || node == expected.end() || node->second != std::make_pair(fingerprint, size_t(count)) || restored->nodes.count(int(id)))
        throw std::runtime_error("index cache feature identity mismatch");
      const size_t work = restored->work.size();
      restored->work.emplace_back();
      restored->work.back().node = int(id);
      restored->nodes.emplace(int(id), Impl::Node{fingerprint, {}, work});
    }
    for (size_t i = 0; i < live; ++i) {
      const auto slot = reader.take(), id = reader.take(), point = reader.take(), appearance = reader.take();
      if (slot >= slots || id > INT_MAX || !restored->nodes.count(int(id)) || restored->slots[slot].node || point >= scene::point_capacity ||
          appearance >= scene::appearance_capacity || bytes.size() - reader.pos < 32)
        throw std::runtime_error("invalid index cache slot mapping");
      auto& ref = restored->slots[slot];
      auto& node = restored->nodes.at(int(id));
      ref.node = int(id);
      ref.point = uint32_t(point);
      ref.appearance = uint32_t(appearance);
      ref.work = node.work;
      std::copy_n(bytes.data() + reader.pos, 32, ref.descriptor.data());
      reader.pos += 32;
      node.ids.push_back(slot);
    }
    for (const auto& [id, node] : restored->nodes)
      if (node.ids.size() != expected.at(id).second) throw std::runtime_error("index cache node count mismatch");
    for (size_t i = 0; i < slots; ++i)
      if (!restored->slots[i].node) restored->recycled.push_back(i);
    const auto index_size = reader.take();
    if (index_size != bytes.size() - reader.pos) throw std::runtime_error("index cache index size mismatch");
    faiss::VectorIOReader input;
    input.data.assign(bytes.begin() + reader.pos, bytes.end());
    SingleThread thread;
    restored->index.reset(faiss::read_index_binary(&input));
    auto& index = *restored->index;
    if (input.rp != input.data.size() || index.d != 256 || index.ntotal != Id(live) || !index.is_trained)
      throw std::runtime_error("invalid restored index");
    if (ivf) {
      auto* index_ivf = dynamic_cast<faiss::IndexBinaryIVF*>(&index);
      if (!index_ivf || index_ivf->nlist != lists || index_ivf->direct_map.type != faiss::DirectMap::Hashtable)
        throw std::runtime_error("invalid restored IVF configuration");
      index_ivf->nprobe = 4;
      index_ivf->use_heap = false;
    } else if (!dynamic_cast<faiss::IndexBinaryIDMap2*>(&index))
      throw std::runtime_error("invalid restored flat configuration");
    // Validate every index ID/code against its owner mapping before publishing.
    std::array<uint8_t, 32> code;
    for (size_t i = 0; i < slots; ++i)
      if (restored->slots[i].node) {
        index.reconstruct(i, code.data());
        if (code != restored->slots[i].descriptor) throw std::runtime_error("index cache code mapping mismatch");
      }
    restored->ivf = bool(ivf);
    restored->restored = true;
    restored->revision = impl_->revision + 1;
    impl_ = std::move(restored);
    reason.clear();
    return true;
  } catch (const std::exception& e) {
    reason = e.what();
    return false;
  }
}
}  // namespace sapphire::mapping
