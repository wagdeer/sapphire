#include <iostream>

#include "backend/storage/map_database.hpp"
#include "backend/visual/faiss/descriptor_archive.hpp"
using namespace sapphire;
int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: visual_retrieval_eval map.db\n";
    return 2;
  }
  sqlite3 *raw = nullptr;
  if (sqlite3_open_v2(argv[1], &raw, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
    std::cerr << "Cannot open map database read-only\n";
    if (raw) sqlite3_close(raw);
    return 1;
  }
  std::unique_ptr<sqlite3, decltype(&sqlite3_close)> db(raw, sqlite3_close);
  mapping::DescriptorArchive archive;
  database_detail::Statement nodes(db.get(), "SELECT id FROM Node ORDER BY id;");
  while (sqlite3_step(nodes.get()) == SQLITE_ROW) {
    const int id = sqlite3_column_int(nodes.get(), 0);
    if (id > 4) {
      database_detail::Statement record(db.get(), "SELECT payload FROM VisualScene WHERE node_id=?;");
      sqlite3_bind_int(record.get(), 1, id - 4);
      if (sqlite3_step(record.get()) == SQLITE_ROW) {
        auto scene = mapping::scene::FeatureMap::decode(sqlite3_column_blob(record.get(), 0), sqlite3_column_bytes(record.get(), 0));
        archive.upsert(id - 4, *scene);
      }
    }
    std::map<int, std::pair<std::size_t, float>> best;
    database_detail::Statement frames(db.get(),
                                      "SELECT point_count,points,descriptors_rows,descriptors_cols,descriptors_type,descriptors,stamp,camera_id FROM ImageRecord "
                                      "WHERE node_id=? ORDER BY frame_index;");
    sqlite3_bind_int(frames.get(), 1, id);
    while (sqlite3_step(frames.get()) == SQLITE_ROW) {
      const auto frame = database_detail::readVisualObservation(database_detail::readReal(frames.get(), 6),
          database_detail::readInteger(frames.get(), 7, 0, 1), database_detail::readInteger(frames.get(), 0),
          sqlite3_column_blob(frames.get(), 1), database_detail::blobBytes(frames.get(), 1),
          database_detail::readMatrix(frames.get(), 2, 3, 4, 5));
      const auto &pixels = frame.points();
      const auto &descriptors = frame.descriptors();
      if (descriptors.empty()) continue;
      std::vector<cv::KeyPoint> points;
      for (const auto &p : pixels) points.emplace_back(p.pixel().x(), p.pixel().y(), 31);
      auto block = features::FeatureBlock::create_raw(points, {}, descriptors);
      for (const auto &[target, candidate] : archive.search(*block)) {
        auto &value = best[target - 1];
        value.first = std::max(value.first, candidate.matches.size());
        value.second = std::max(value.second, candidate.score);
      }
    }
    for (const auto &[target, stats] : best) std::cout << id - 1 << "," << target << "," << stats.first << "," << stats.second << "\n";
  }
}
