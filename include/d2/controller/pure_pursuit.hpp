// Copyright 2025 miyajimad0620
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef D2__CONTROLLER__PURE_PURSUIT_HPP_
#define D2__CONTROLLER__PURE_PURSUIT_HPP_

#include <optional>

#include "vector3.hpp"

namespace d2::controller
{

template<class Path>
constexpr std::optional<Vector3> create_pure_pursuit_point(
  const Path & path, double lookahead_distance)
{
  const auto path_size = std::size(path);
  if (path_size == 0) {
    return std::nullopt;
  }

  const auto begin = std::begin(path);
  const auto end = std::end(path);
  auto itr_last = begin;
  for (auto itr = std::next(begin); itr != end; itr_last = itr, itr = std::next(itr)) {
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

}  // namespace d2::controller

#endif  // D2__CONTROLLER__PURE_PURSUIT_HPP_
