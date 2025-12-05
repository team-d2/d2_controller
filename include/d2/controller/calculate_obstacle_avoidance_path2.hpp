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

#ifndef D2__CONTROLLER__CALCULATE_OBSTACLE_AVOIDANCE_PATH2_HPP_
#define D2__CONTROLLER__CALCULATE_OBSTACLE_AVOIDANCE_PATH2_HPP_

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include <limits>
#include <chrono>
#include <optional>

#include "Eigen/Dense"

namespace d2::controller
{

namespace
{

using Vec2 = Eigen::Vector2d;
const double EPS = 1e-9;

struct Intersection {
    Vec2 point;
    int path_seg_idx;
    double t_path;
    int poly_seg_idx;
    double t_poly;
};

double cross(const Vec2 &a, const Vec2 &b) { return a.x()*b.y() - a.y()*b.x(); }

auto segment_intersection(const Vec2 &p, const Vec2 &p2, const Vec2 &q, const Vec2 &q2)
    -> std::optional<std::pair<double,double>> {
    Vec2 r = p2 - p;
    Vec2 s = q2 - q;
    double rxs = cross(r, s);
    Vec2 qp = q - p;
    if (std::abs(rxs) < EPS) return std::nullopt;
    double t = cross(qp, s) / rxs;
    double u = cross(qp, r) / rxs;
    if (t >= -EPS && t <= 1+EPS && u >= -EPS && u <= 1+EPS) return std::make_optional(std::make_pair(t, u));
    return std::nullopt;
}

bool point_on_segment(const Vec2 &a, const Vec2 &b, const Vec2 &p) {
    Vec2 ab = b-a;
    Vec2 ap = p-a;
    if (std::abs(cross(ab, ap)) > 1e-8) return false;
    double dot = ab.dot(ap);
    return dot >= -EPS && dot <= ab.squaredNorm()+EPS;
}

bool point_in_polygon(const std::vector<Vec2> &poly, const Vec2 &pt) {
    bool inside = false;
    int n = (int)poly.size();
    for (int i = 0, j = n-1; i < n; j = i++) {
        const Vec2 &pi = poly[i];
        const Vec2 &pj = poly[j];
        if (point_on_segment(pj, pi, pt)) return true;
        bool intersect = ((pi.y() > pt.y()) != (pj.y() > pt.y())) &&
                         (pt.x() < (pj.x()-pi.x()) * (pt.y()-pi.y()) / (pj.y()-pi.y()+EPS) + pi.x());
        if (intersect) inside = !inside;
    }
    return inside;
}

double polyline_length(const std::vector<Vec2> &pts) {
    double L = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) L += (pts[i]-pts[i-1]).norm();
    return L;
}

Vec2 interp_on_edge(const std::vector<Vec2> &poly, int j, double t) {
    int n = (int)poly.size();
    Vec2 a = poly[j % n];
    Vec2 b = poly[(j+1) % n];
    return a + (b-a) * t;
}

std::vector<Vec2> polygon_arc(const std::vector<Vec2> &poly, int j1, double t1, int j2, double t2, bool forward=true) {
    int n = (int)poly.size();
    j1 = (j1 % n + n) % n;
    j2 = (j2 % n + n) % n;
    std::vector<Vec2> arc;

    arc.push_back(interp_on_edge(poly, j1, t1));
    if (j1 == j2) {
        if (forward) {
            if (t2 > t1 + EPS) arc.push_back(interp_on_edge(poly, j2, t2));
        } else {
            if (t1 > t2 + EPS) arc.push_back(interp_on_edge(poly, j2, t2));
        }
        return arc;
    }

    int cur = j1;
    if (forward) {
        arc.push_back(poly[(j1+1)%n]);
        cur = (j1+1)%n;
        while (cur != j2) {
            arc.push_back(poly[(cur+1)%n]);
            cur = (cur+1)%n;
        }
        arc.back() = interp_on_edge(poly, j2, t2);
    } else {
        arc.push_back(poly[j1]);
        cur = (j1-1+n)%n;
        while ((cur+1)%n != j2) {
            arc.push_back(poly[cur]);
            cur = (cur-1+n)%n;
        }
        arc.push_back(interp_on_edge(poly, j2, t2));
    }

    std::vector<Vec2> out;
    for (auto &p : arc) {
        if (out.empty() || (p - out.back()).norm() > 1e-12) out.push_back(p);
    }
    return out;
}

std::vector<Vec2> choose_shorter_arc(const std::vector<Vec2> &poly, int j1, double t1, int j2, double t2) {
    auto arc_fwd = polygon_arc(poly, j1, t1, j2, t2, true);
    auto arc_bwd = polygon_arc(poly, j1, t1, j2, t2, false);
    return (polyline_length(arc_fwd) <= polyline_length(arc_bwd)) ? arc_fwd : arc_bwd;
}

std::vector<Vec2> detour_along_polygon(const std::vector<Vec2> &path, const std::vector<Vec2> &poly) {
    auto t_all0 = std::chrono::steady_clock::now();
    std::cout << "Detouring along polygon with " << poly.size() << " vertices." << std::endl;

    std::vector<Intersection> inters;
    int npoly = (int)poly.size();
    if (npoly == 0) return path;

    auto t_aabb0 = std::chrono::steady_clock::now();
    std::vector<Eigen::AlignedBox2d> edge_aabbs;
    edge_aabbs.reserve(npoly);
    for (int e = 0; e < npoly; ++e) {
        Eigen::AlignedBox2d box;
        box.extend(poly[e]);
        box.extend(poly[(e+1)%npoly]);
        edge_aabbs.push_back(box);
    }
    auto t_aabb1 = std::chrono::steady_clock::now();
    std::cout << "AABB build      : " << std::chrono::duration_cast<std::chrono::milliseconds>(t_aabb1 - t_aabb0).count() << " ms" << std::endl;

    auto t_int0 = std::chrono::steady_clock::now();
    for (int i = 0; i + 1 < (int)path.size(); ++i) {
        Vec2 p = path[i];
        Vec2 p2 = path[i+1];
        Eigen::AlignedBox2d seg_box;
        seg_box.extend(p);
        seg_box.extend(p2);
        for (int j = 0; j < npoly; ++j) {
            if (!seg_box.intersects(edge_aabbs[j])) continue;
            Vec2 q = poly[j];
            Vec2 q2 = poly[(j+1)%npoly];
            auto res = segment_intersection(p, p2, q, q2);
            if (res) {
                inters.push_back({p + (p2-p)*res->first, i, res->first, j, res->second});
            }
        }
    }
    auto t_int1 = std::chrono::steady_clock::now();
    std::cout << "Intersection    : " << std::chrono::duration_cast<std::chrono::milliseconds>(t_int1 - t_int0).count() << " ms" << std::endl;

    if (inters.empty()) return path;

    auto t_sort0 = std::chrono::steady_clock::now();
    std::sort(inters.begin(), inters.end(), [](auto &a, auto &b) {
        if (a.path_seg_idx != b.path_seg_idx) return a.path_seg_idx < b.path_seg_idx;
        return a.t_path < b.t_path;
    });
    auto t_sort1 = std::chrono::steady_clock::now();
    std::cout << "Sort            : " << std::chrono::duration_cast<std::chrono::milliseconds>(t_sort1 - t_sort0).count() << " ms" << std::endl;

    std::vector<Vec2> out;
    int cur_path_idx = 0;
    size_t k = 0;
    while (cur_path_idx + 1 < (int)path.size()) {
        if (out.empty() || (path[cur_path_idx]-out.back()).norm() > 1e-12) out.push_back(path[cur_path_idx]);
        if (k < inters.size() && inters[k].path_seg_idx == cur_path_idx) {
            auto in = inters[k++];
            if (k >= inters.size()) { out.push_back(in.point); break; }
            auto outi = inters[k++];
            out.push_back(in.point);
            auto arc = choose_shorter_arc(poly, in.poly_seg_idx, in.t_poly, outi.poly_seg_idx, outi.t_poly);
            for (size_t z = 1; z < arc.size(); ++z) out.push_back(arc[z]);
            cur_path_idx = outi.path_seg_idx;
            if ((out.back()-outi.point).norm() > 1e-12) out.push_back(outi.point);
        }
        cur_path_idx++;
    }

    if (!path.empty() && (out.empty() || (path.back()-out.back()).norm() > 1e-12)) out.push_back(path.back());

    auto t_all1 = std::chrono::steady_clock::now();
    std::cout << "Total           : " << std::chrono::duration_cast<std::chrono::milliseconds>(t_all1 - t_all0).count() << " ms" << std::endl;

    // std::cout << "AABB build      : " << std::chrono::duration<double>(t_aabb1 - t_aabb0).count() << " sec\n";
    // std::cout << "Intersection    : " << std::chrono::duration<double>(t_int1 - t_int0).count() << " sec\n";
    // std::cout << "Sort            : " << std::chrono::duration<double>(t_sort1 - t_sort0).count() << " sec\n";
    // std::cout << "Total           : " << std::chrono::duration<double>(t_all1 - t_all0).count() << std::endl;

    return out;
}

std::vector<Vec2> detour_along_polygons(
    std::vector<Vec2> path,
    const std::vector<std::vector<Vec2>> & polygons)
{
    auto t_start = std::chrono::steady_clock::now();
    for (const auto & polygon : polygons) {
        path = detour_along_polygon(path, polygon);
    }
    auto t_end = std::chrono::steady_clock::now();
    std::cout << "Total obstacle avoidance time: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count()
              << " ms" << std::endl;
    return path;
}
}  // namespace
}  // namespace d2::controller

#endif  // D2__CONTROLLER__CALCULATE_OBSTACLE_AVOIDANCE_PATH2_HPP_
