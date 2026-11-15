#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <set>
#include <utility>
#include <vector>

#include "clipper2/clipper.h"
#include "clipper2/clipper.variableoffset.h"

using namespace Clipper2Lib;

namespace {

using PointSet = std::set<std::pair<int64_t, int64_t>>;

PointSet Flatten(const Paths64& paths)
{
  PointSet out;
  for (const Path64& path : paths)
    for (const Point64& pt : path)
      out.emplace(pt.x, pt.y);
  return out;
}

// Hand-rolled reference: keep the original contour and offset each run as an
// open round-capped segment. Returns the raw fragments; the caller performs
// the final union exactly once.
Paths64 ReferenceFragments(const Paths64& contours,
  const std::vector<OffsetRun>& runs, const VariableOffsetOptions& opts)
{
  Paths64 fragments;
  for (const Path64& contour : contours)
    fragments.push_back(contour);

  for (const Path64& contour : contours)
  {
    for (const OffsetRun& run : runs)
    {
      Path64 piece;
      for (size_t i = run.start; i <= run.end; ++i)
        piece.push_back(contour[i % contour.size()]);

      if (run.delta == 0.0)
      {
        fragments.push_back(piece);
        continue;
      }
      ClipperOffset co(opts.miter_limit, opts.arc_tolerance,
        opts.preserve_collinear, opts.reverse_solution);
      co.AddPath(piece, opts.join_type, EndType::Round);
      Paths64 offset;
      co.Execute(run.delta, offset);
      fragments.insert(fragments.end(),
        std::make_move_iterator(offset.begin()),
        std::make_move_iterator(offset.end()));
    }
  }
  return fragments;
}

Path64 MakeContour(size_t n, int64_t radius, uint32_t seed)
{
  Path64 path;
  path.reserve(n);
  for (size_t i = 0; i < n; ++i)
  {
    seed = seed * 1664525u + 1013904223u;
    const double wobble = 0.75 + 0.35 * (double)(seed % 1000) / 1000.0;
    const double angle = 2.0 * PI * static_cast<double>(i) /
      static_cast<double>(n);
    path.emplace_back(
      static_cast<int64_t>(4000 + radius * wobble * std::cos(angle)),
      static_cast<int64_t>(4000 + radius * wobble * std::sin(angle)));
  }
  if (Area(path) < 0)
    std::reverse(path.begin(), path.end());
  return path;
}

} // namespace

int main()
{
  constexpr int kIterations = 1000;
  int mismatches = 0;
  size_t total_points = 0;
  size_t max_point_diff = 0;
  int64_t max_allocated_paths = 0;

  const JoinType join_types[] = {
    JoinType::Square, JoinType::Bevel, JoinType::Round, JoinType::Miter };

  for (int iter = 0; iter < kIterations; ++iter)
  {
    const size_t vertex_count = 6 + static_cast<size_t>(iter % 18);
    const Path64 outer = MakeContour(vertex_count, 3000,
      static_cast<uint32_t>(iter + 1));
    const Path64 hole = MakeContour(
      (vertex_count / 2) + 3, 900, static_cast<uint32_t>(iter + 777));
    Path64 reversed_hole = hole;
    std::reverse(reversed_hole.begin(), reversed_hole.end());
    const Paths64 shape{ outer, reversed_hole };

    const double d0 = 10.0 + static_cast<double>(iter % 60);
    const double d1 = 5.0 + static_cast<double>((iter * 7) % 90);
    auto make_runs = [d0, d1](size_t count) {
      const size_t split = count / 2;
      return std::vector<OffsetRun>{
        OffsetRun(0, split, d0), OffsetRun(split, count, d1) };
    };
    const auto outer_runs = make_runs(outer.size());
    const auto hole_runs = make_runs(reversed_hole.size());

    VariableOffsetOptions opts;
    opts.join_type = join_types[iter % 4];
    opts.miter_limit = 1.5 + static_cast<double>(iter % 5);

    // VariableOffsetShape assumes a shared schedule, so drive each contour
    // with its own schedule and merge the fragments explicitly.
    Paths64 merged_fragments;
    Paths64 ref_fragments;
    for (size_t c = 0; c < shape.size(); ++c)
    {
      const auto& schedule = (c == 0) ? outer_runs : hole_runs;
      Paths64 part = VariableOffsetContour(shape[c], schedule, opts);
      merged_fragments.insert(merged_fragments.end(),
        std::make_move_iterator(part.begin()),
        std::make_move_iterator(part.end()));
      Paths64 ref = ReferenceFragments(Paths64{ shape[c] }, schedule, opts);
      ref_fragments.insert(ref_fragments.end(),
        std::make_move_iterator(ref.begin()),
        std::make_move_iterator(ref.end()));
    }
    const Paths64 merged =
      Union(merged_fragments, FillRule::NonZero);
    const Paths64 reference =
      Union(ref_fragments, FillRule::NonZero);

    const PointSet a = Flatten(merged);
    const PointSet b = Flatten(reference);
    total_points += a.size();
    max_allocated_paths =
      std::max(max_allocated_paths, static_cast<int64_t>(merged.size()));

    if (a != b)
    {
      ++mismatches;
      size_t diff = 0;
      for (const auto& p : a)
        if (!b.count(p)) ++diff;
      for (const auto& p : b)
        if (!a.count(p)) ++diff;
      max_point_diff = std::max(max_point_diff, diff);
    }
  }

  std::cout << "iterations           : " << kIterations << '\n';
  std::cout << "mismatching runs     : " << mismatches << '\n';
  std::cout << "max point diff count : " << max_point_diff << '\n';
  std::cout << "total output points  : " << total_points << '\n';
  std::cout << "max paths per result : " << max_allocated_paths << '\n';
  std::cout << (mismatches == 0 ?
    "RESULT: pointwise-identical on every iteration\n" :
    "RESULT: CONSISTENCY FAILURE\n");
  return mismatches == 0 ? 0 : 1;
}
