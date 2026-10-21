#include <backend/registration/retrieval/faiss_cosine_index.hpp>
#include <faiss/IndexFlat.h>
#include <faiss/IndexIDMap.h>
#include <faiss/impl/IDSelector.h>
#include <algorithm>
#include <cmath>
#include <climits>
#include <cstring>
#include <map>
#include <string>
namespace sapphire::backend::feature {
namespace {
std::string identity(const GlobalDescriptor& d) {
  return {reinterpret_cast<const char*>(d.info().ptr()), d.info().total()};
}
struct Eligible final : faiss::IDSelector {
  const std::function<bool(int)>& predicate;
  explicit Eligible(const std::function<bool(int)>& p) : predicate(p) {}
  bool is_member(faiss::idx_t id) const override { return id > 0 && id <= INT_MAX && predicate(int(id)); }
};
}
struct FaissCosineIndex::Impl {
  struct Group {
    faiss::IndexFlatIP flat{768};
    faiss::IndexIDMap2 index{&flat};
  };
  std::map<std::string,std::unique_ptr<Group>> groups;
  std::map<int,GlobalDescriptor> rows;
};
FaissCosineIndex::FaissCosineIndex() : impl_(std::make_unique<Impl>()) {}
FaissCosineIndex::~FaissCosineIndex() = default;
bool FaissCosineIndex::valid(const GlobalDescriptor& d) {
  return d.type()==2 && d.data().type()==CV_32F && d.data().rows==1 && d.data().cols==768 &&
    d.info().type()==CV_8U && d.info().rows==1 && !d.info().empty() && cv::checkRange(d.data()) && std::abs(cv::norm(d.data())-1.)<1e-3;
}
void FaissCosineIndex::erase(int id) {
  auto it=impl_->rows.find(id);if(it==impl_->rows.end())return;
  auto group=impl_->groups.find(identity(it->second));
  faiss::IDSelectorRange selector(id,faiss::idx_t(id)+1);
  group->second->index.remove_ids(selector);
  if(!group->second->index.ntotal)impl_->groups.erase(group);
  impl_->rows.erase(it);
}
void FaissCosineIndex::set(int id,const std::vector<GlobalDescriptor>& descriptors) {
  if(id<=0)return;
  for(const auto& d:descriptors)if(valid(d)) {
    const auto key=identity(d);
    auto old=impl_->rows.find(id);
    if(old!=impl_->rows.end() && identity(old->second)==key &&
       !std::memcmp(old->second.data().ptr(),d.data().ptr(),768*sizeof(float)))return;
    erase(id);
    auto& group=impl_->groups[key];if(!group)group=std::make_unique<Impl::Group>();
    const faiss::idx_t label=id;
    group->index.add_with_ids(1,d.data().ptr<float>(),&label);
    impl_->rows.emplace(id,GlobalDescriptor(2,d.data().clone(),d.info().clone()));
    return;
  }
  erase(id);
}
std::vector<float> FaissCosineIndex::scores(const std::vector<GlobalDescriptor>& descriptors, const std::vector<int>& ids) const {
  std::vector<float> result(ids.size(), 0.f);
  for (const auto& query : descriptors) if (valid(query)) {
    const auto model = identity(query);
    for (size_t i = 0; i < ids.size(); ++i) {
      const auto found = impl_->rows.find(ids[i]);
      if (found != impl_->rows.end() && identity(found->second) == model)
        result[i] = float(query.data().dot(found->second.data()));
    }
    break;
  }
  return result;
}
void FaissCosineIndex::clear(){impl_->groups.clear();impl_->rows.clear();}
size_t FaissCosineIndex::memory_size() const {
  size_t size=0;
  for(const auto& [id,d]:impl_->rows)size+=2*768*sizeof(float)+d.info().total()+sizeof(d)+64;
  for(const auto& [key,g]:impl_->groups)size+=key.size()+sizeof(Impl::Group)+g->index.ntotal*32;
  return size;
}
std::vector<int> FaissCosineIndex::search(const std::vector<GlobalDescriptor>& descriptors,size_t limit,
    const std::function<bool(int)>& eligible, std::vector<float>* returned_scores) const {
  if (returned_scores) returned_scores->clear();
  if(!limit)return {};
  for(const auto& d:descriptors)if(valid(d)) {
    const auto group=impl_->groups.find(identity(d));if(group==impl_->groups.end())return {};
    const size_t k=std::min(limit,size_t(group->second->index.ntotal));if(!k)return {};
    std::vector<float> scores(k);std::vector<faiss::idx_t> labels(k);
    Eligible selector(eligible);faiss::SearchParameters params;params.sel=&selector;
    group->second->index.search(1,d.data().ptr<float>(),k,scores.data(),labels.data(),&params);
    std::vector<int> result;
    for (size_t i = 0; i < labels.size(); ++i) if (labels[i] >= 0) {
      result.push_back(int(labels[i]));
      if (returned_scores) returned_scores->push_back(scores[i]);
    }
    return result;
  }
  return {};
}
} // namespace sapphire::backend::feature
