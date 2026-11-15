// Variable-distance offset of a contour split into constant-distance runs.
//
// Closed contour semantics (EndType::Polygon, the default):
//   Each run is treated as an open polyline segment and offset independently
//   at its own (constant) distance with round end caps. The original contour
//   is added unchanged so the interior - including holes - keeps its area and
//   orientation. All fragments are merged in one signed boolean union. The
//   round caps at a run boundary shared by two runs overlap, producing a
//   continuous outer fillet; the original contour keeps every hole oriented
//   opposite to its outer boundary.
//
// The fragment set is exactly what a caller obtains by offsetting every
// segment with its own ClipperOffset and feeding the pieces plus the original
// contour to one union, hence the merged result is point-for-point identical
// to that manual "offset each segment, then merge" pipeline.

#ifndef CLIPPER_VARIABLE_OFFSET_H_
#define CLIPPER_VARIABLE_OFFSET_H_

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include "clipper.core.h"
#include "clipper.engine.h"
#include "clipper.offset.h"

namespace Clipper2Lib {

// A single constant-distance run of a contour with N vertices.
// Run [start, end) covers the polyline from vertex start to vertex end
// (inclusive). Runs partition the contour:
//  - closed contour: the last run ends at N (vertex N wraps to vertex 0);
//  - open contour:   the last run ends at N-1.
// Adjacent runs share their boundary vertex.
// delta must be >= 0: this narrow interface performs inflation only.
struct OffsetRun {
  size_t start = 0;
  size_t end   = 0;
  double delta = 0.0;

  OffsetRun() = default;
  OffsetRun(size_t start_, size_t end_, double delta_)
    : start(start_), end(end_), delta(delta_) {}
};

// Sharp-corner handling and other knobs. join_type is forwarded unchanged to
// every constant-distance run.
struct VariableOffsetOptions {
  JoinType join_type          = JoinType::Round;
  EndType  end_type           = EndType::Polygon; // closed, or an open EndType
  double   miter_limit        = 2.0;
  double   arc_tolerance      = 0.0; // 0 => engine default (delta / 500)
  bool     preserve_collinear = false;
  bool     reverse_solution   = false;
};

namespace detail {

inline Paths64 UnionPaths(const Paths64& subjects, FillRule fill_rule,
  bool preserve_collinear, bool reverse_solution)
{
  Paths64 result;
  Clipper64 clipper;
  // Match the free Union() helper exactly: keep the engine defaults so the
  // output is independent of how the caller invokes the union.
  (void)preserve_collinear;
  (void)reverse_solution;
  clipper.AddSubject(subjects);
  clipper.Execute(ClipType::Union, fill_rule, result);
  return result;
}

inline bool IsOpenEndType(EndType et)
{
  return et == EndType::Butt || et == EndType::Square ||
         et == EndType::Round || et == EndType::Joined;
}

// Validates the schedule and normalizes it by sorting runs on start. Throws
// std::invalid_argument on any inconsistency. An empty schedule is valid and
// leaves the contour untouched.
inline std::vector<OffsetRun> ValidateRuns(const Path64& contour,
  const std::vector<OffsetRun>& runs, bool is_open)
{
  const size_t vertex_count = contour.size();
  const size_t limit = vertex_count - (is_open ? 1 : 0);

  if (vertex_count < 2)
    throw std::invalid_argument(
      "VariableOffset: contour must contain at least 2 vertices");

  std::vector<OffsetRun> sorted = runs;
  std::sort(sorted.begin(), sorted.end(),
    [](const OffsetRun& a, const OffsetRun& b) { return a.start < b.start; });

  size_t expected = 0;
  for (const OffsetRun& run : sorted)
  {
    if (run.end <= run.start)
      throw std::invalid_argument(
        "VariableOffset: every run must satisfy end > start");
    if (run.start != expected)
      throw std::invalid_argument(
        "VariableOffset: runs must be contiguous, start at 0 and not overlap");
    if (run.end > limit)
      throw std::invalid_argument(
        "VariableOffset: run extends past the end of the contour");
    if (run.delta < 0.0)
      throw std::invalid_argument(
        "VariableOffset: delta must be >= 0 (inflation only)");
    expected = run.end;
  }
  if (!sorted.empty() && expected != limit)
    throw std::invalid_argument(
      "VariableOffset: runs must cover the whole contour");
  return sorted;
}

// Extracts the open sub-path from vertex run.start to vertex run.end.
inline Path64 ExtractRunPath(const Path64& contour, const OffsetRun& run)
{
  Path64 piece;
  piece.reserve(run.end - run.start + 1);
  for (size_t i = run.start; i <= run.end; ++i)
    piece.push_back(contour[i % contour.size()]);
  return piece;
}

// Offsets one run at constant distance as an open polyline segment. Closed
// contours use round caps so junctions stay watertight and smooth after the
// union; explicit open EndTypes are honored as given. A zero-distance run
// contributes its untouched segment.
inline Paths64 OffsetOneRun(const Path64& piece, const OffsetRun& run,
  const VariableOffsetOptions& opts)
{
  if (run.delta == 0.0)
    return Paths64{ piece };

  ClipperOffset co(opts.miter_limit, opts.arc_tolerance,
    opts.preserve_collinear, opts.reverse_solution);
  const EndType effective_end =
    opts.end_type == EndType::Polygon ? EndType::Round : opts.end_type;
  co.AddPath(piece, opts.join_type, effective_end);

  Paths64 offset;
  co.Execute(run.delta, offset);
  return offset;
}

} // namespace detail

// Variable-distance offset of one contour.
//
// For a closed contour the returned fragment set contains the original
// contour (interior and orientation preserved) plus one open round-capped
// offset band per run. Merge these fragments, together with fragments from
// other contours of the same shape, through a single NonZero union - see
// VariableOffsetShape(). For an open contour only the offset bands are
// returned.
inline Paths64 VariableOffsetContour(const Path64& contour,
  const std::vector<OffsetRun>& runs,
  const VariableOffsetOptions& options = VariableOffsetOptions{})
{
  const bool is_open = detail::IsOpenEndType(options.end_type);
  const std::vector<OffsetRun> sorted =
    detail::ValidateRuns(contour, runs, is_open);

  if (sorted.empty())
    return Paths64{ contour };

  Paths64 fragments;
  if (!is_open)
    fragments.push_back(contour); // keeps the interior and hole orientation

  for (const OffsetRun& run : sorted)
  {
    Path64 piece = detail::ExtractRunPath(contour, run);
    Paths64 offset = detail::OffsetOneRun(piece, run, options);
    fragments.insert(fragments.end(),
      std::make_move_iterator(offset.begin()),
      std::make_move_iterator(offset.end()));
  }
  return fragments;
}

// Variable-distance offset of a whole shape (outer contours plus holes).
// Every contour uses the same run schedule expressed in that contour's own
// index space; all fragments are merged in one NonZero union, which keeps
// opposite-oriented holes regardless of the outer contour's orientation.
inline Paths64 VariableOffsetShape(const Paths64& contours,
  const std::vector<OffsetRun>& runs,
  const VariableOffsetOptions& options = VariableOffsetOptions{})
{
  Paths64 fragments;
  for (const Path64& contour : contours)
  {
    Paths64 part = VariableOffsetContour(contour, runs, options);
    fragments.insert(fragments.end(),
      std::make_move_iterator(part.begin()),
      std::make_move_iterator(part.end()));
  }

  return detail::UnionPaths(fragments, FillRule::NonZero,
    options.preserve_collinear, options.reverse_solution);
}

} // namespace Clipper2Lib

#endif // CLIPPER_VARIABLE_OFFSET_H_
