#ifndef D2__CONTROLLER__CSV_SAVER_HPP_
#define D2__CONTROLLER__CSV_SAVER_HPP_

#include <optional>
#include "vector3.hpp"

namespace d2::controller
{

template <class Path>
constexpr std::optional<Vector3> create_pure_pursuit_point(const Path & path, double lookahead_distance)
{
  const auto path_size = std::size(path);
  if (path_size == 0) {
    return std::nullopt;
  }

  const auto begin = std::begin(path);
  const auto end = std::end(path);
  auto itr_last = begin;
  for (auto itr = std::next(begin); itr != end; 
    itr_last = itr, itr = std::next(itr)) {
    const auto vec = itr->data - itr_last->data;
    const auto section_distance = length(vec);
    if (section_distance >= lookahead_distance) {
      const auto ratio = lookahead_distance / section_distance;
      return itr_last->data + vec * ratio;
    }
    lookahead_distance -= section_distance;
  }

  return itr_last->data;
}

}

#endif // D2__CONTROLLER__CSV_SAVER_HPP_
