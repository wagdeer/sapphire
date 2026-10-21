#pragma once

#include <set>
#include <utility>
#include <vector>

namespace sapphire::backend::tools {

// 从 map 中查找 key，如果存在则返回对应的值，否则返回 default_value。
// 返回类型是 Map::mapped_type（值拷贝）。
// 类似于 std::map::at()，但提供了默认值，避免抛异常。
template <typename Map, typename Key, typename Default>
auto value_or(const Map& map, const Key& key, Default&& default_value) -> typename Map::mapped_type {
  const auto found = map.find(key);
  return found == map.end() ? typename Map::mapped_type(std::forward<Default>(default_value)) : found->second;
}

// 对于有序 multimap（如 std::multimap），返回所有不重复的 key，按 key 升序排列。
// 利用 multimap 的有序性，通过 upper_bound 跳过相同 key 的所有元素。
// 注意：要求容器支持 upper_bound，且 key 可比较。不适用于 unordered_multimap。
template <typename Multimap>
auto unique_keys(const Multimap& multimap) -> std::vector<typename Multimap::key_type> {
  std::vector<typename Multimap::key_type> result;
  result.reserve(multimap.size());  // 预分配最大可能大小，避免频繁扩容
  for (auto iterator = multimap.begin(); iterator != multimap.end();) {
    result.push_back(iterator->first);                     // 记录当前 key
    iterator = multimap.upper_bound(iterator->first);      // 跳到下一个不同的 key
  }
  return result;
}

// 提取 map（或任何键值对容器）中的所有 key，放入 std::set 中，自动去重并排序。
// 适用于 std::map、std::multimap、std::unordered_map 等具有 first 成员的容器。
template <typename Map>
auto key_set(const Map& map) -> std::set<typename Map::key_type> {
  std::set<typename Map::key_type> result;
  for (const auto& item : map) {
    result.insert(result.end(), item.first);  // 插入 key，set 自动去重排序
  }
  return result;
}

}  // namespace sapphire::backend::tools