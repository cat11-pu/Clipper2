#include <gtest/gtest.h>

#include <set>
#include <stdexcept>
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

// Independent reference implementation of the manual pipeline: keep the
// original contours (closed case), offset every run as an open round-capped
// segment with its own ClipperOffset, then merge everything in one signed
// union. It intentionally does not call VariableOffsetContour, so an
// equality here proves the wrapper matches "offset each segment, then merge".
Paths64 ReferenceShape(const Paths64& contours,
  const std::vector<OffsetRun>& runs, const VariableOffsetOptions& opts)
{
  const bool is_open =
    opts.end_type == EndType::Butt || opts.end_type == EndType::Square ||
    opts.end_type == EndType::Round || opts.end_type == EndType::Joined;

  Paths64 fragments;
  if (!is_open)
    for (const Path64& contour : contours)
      fragments.push_back(contour);

  for (const Path64& contour : contours)
  {
    std::vector<OffsetRun> sorted = runs;
    std::sort(sorted.begin(), sorted.end(),
      [](const OffsetRun& a, const OffsetRun& b) { return a.start < b.start; });

    for (const OffsetRun& run : sorted)
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
      const EndType effective_end =
        is_open ? opts.end_type : EndType::Round;
      co.AddPath(piece, opts.join_type, effective_end);
      Paths64 offset;
      co.Execute(run.delta, offset);
      fragments.insert(fragments.end(),
        std::make_move_iterator(offset.begin()),
        std::make_move_iterator(offset.end()));
    }
  }
  return Union(fragments, FillRule::NonZero);
}

Path64 OuterContour()
{
  Path64 path = MakePath({100, 100, 500, 100, 500, 400, 100, 400});
  if (Area(path) < 0)
    std::reverse(path.begin(), path.end());
  return path;
}

Path64 HoleContour()
{
  Path64 path = MakePath({250, 200, 250, 300, 350, 300, 350, 200});
  if (Area(path) > 0)
    std::reverse(path.begin(), path.end());
  return path;
}

// Geometric equivalence: the two solutions cover exactly the same region.
// Symmetric difference must be empty. Tolerates different arc tessellation
// between independent code paths.
bool CoverSameRegion(const Paths64& a, const Paths64& b)
{
  const Paths64 diff = Xor(a, b, FillRule::Positive);
  return diff.empty() || Area(diff) == 0.0;
}

std::vector<OffsetRun> TwoRuns(double d0, double d1)
{
  return { OffsetRun(0, 2, d0), OffsetRun(2, 4, d1) };
}

} // namespace

TEST(VariableOffset, PointwiseEqualityWithSeparateOffsetThenUnion)
{
  const Paths64 subject{ OuterContour(), HoleContour() };
  const auto runs = TwoRuns(40.0, 15.0);

  for (JoinType jt : { JoinType::Square, JoinType::Bevel,
                       JoinType::Round, JoinType::Miter })
  {
    VariableOffsetOptions opts;
    opts.join_type = jt;

    const Paths64 merged = VariableOffsetShape(subject, runs, opts);
    const Paths64 reference = ReferenceShape(subject, runs, opts);

    EXPECT_EQ(Flatten(merged), Flatten(reference));
    EXPECT_EQ(merged.size(), reference.size());
  }
}

TEST(VariableOffset, ConstantScheduleMatchesClipperInflate)
{
  const Paths64 subject{ OuterContour(), HoleContour() };
  const std::vector<OffsetRun> runs{ OffsetRun(0, 4, 25.0) };

  VariableOffsetOptions opts;
  const Paths64 merged = VariableOffsetShape(subject, runs, opts);
  const Paths64 inflated =
    InflatePaths(subject, 25.0, JoinType::Round, EndType::Polygon);

  // The run pipeline and native inflate use different tessellation paths, so
  // assert region equivalence rather than exact integer point equality.
  EXPECT_TRUE(CoverSameRegion(merged, inflated));
}

TEST(VariableOffset, HoleOrientationPreserved)
{
  const Paths64 subject{ OuterContour(), HoleContour() };
  ASSERT_TRUE(IsPositive(subject[0]));
  ASSERT_FALSE(IsPositive(subject[1]));

  const Paths64 merged =
    VariableOffsetShape(subject, TwoRuns(30.0, 30.0));
  ASSERT_GE(merged.size(), 2u);

  bool found_positive_outer = false;
  bool found_negative_hole = false;
  for (const Path64& path : merged)
  {
    if (Area(path) > 0) found_positive_outer = true;
    if (Area(path) < 0) found_negative_hole = true;
  }
  EXPECT_TRUE(found_positive_outer);
  EXPECT_TRUE(found_negative_hole);
}

TEST(VariableOffset, RoundJoinKeepsOuterFilletContinuous)
{
  // The two runs share boundary vertices 2. With round joins every run emits
  // arc points around those vertices, so the union boundary must contain arc
  // vertices rather than a single bevel/miter point.
  const Paths64 subject{ OuterContour() };
  const Paths64 merged =
    VariableOffsetShape(subject, TwoRuns(40.0, 15.0));

  const Point64 boundary = subject[0][2];
  size_t arc_points = 0;
  for (const Path64& path : merged)
    for (const Point64& pt : path)
    {
      const double dx = static_cast<double>(pt.x - boundary.x);
      const double dy = static_cast<double>(pt.y - boundary.y);
      const double radius = std::sqrt(dx * dx + dy * dy);
      if (radius > 16.0 && radius < 42.0)
        ++arc_points;
    }
  EXPECT_GT(arc_points, 2u);
}

TEST(VariableOffset, ZeroDeltaRunIsNoOpForThatSegment)
{
  const Paths64 subject{ OuterContour() };
  const std::vector<OffsetRun> runs{
    OffsetRun(0, 2, 0.0), OffsetRun(2, 4, 30.0) };

  const Paths64 merged = VariableOffsetShape(subject, runs);
  const Paths64 reference = ReferenceShape(subject, runs,
    VariableOffsetOptions{});
  EXPECT_EQ(Flatten(merged), Flatten(reference));

  // A point strictly inside the original contour remains inside the result,
  // and nothing extends beyond the inflated run's reach.
  const Point64 interior(300, 250);
  bool interior_covered = false;
  const Point64 beyond(600, 250); // > 30 units past the right edge
  for (const Path64& path : merged)
  {
    if (PointInPolygon(interior, path) != PointInPolygonResult::IsOutside)
      interior_covered = true;
    EXPECT_EQ(PointInPolygon(beyond, path), PointInPolygonResult::IsOutside);
  }
  EXPECT_TRUE(interior_covered);
}

TEST(VariableOffset, EmptyScheduleReturnsInputContour)
{
  const Path64 subject = OuterContour();
  const Paths64 fragments =
    VariableOffsetContour(subject, std::vector<OffsetRun>{});
  ASSERT_EQ(fragments.size(), 1u);
  ASSERT_EQ(fragments[0].size(), subject.size());
  for (size_t i = 0; i < subject.size(); ++i)
    EXPECT_EQ(fragments[0][i], subject[i]);
}

TEST(VariableOffset, RejectsInvalidSchedules)
{
  const Path64 contour = OuterContour();
  VariableOffsetOptions opts;

  EXPECT_THROW(VariableOffsetContour(contour,
    { OffsetRun(0, 2, 10.0), OffsetRun(3, 4, 10.0) }), // gap
    std::invalid_argument);
  EXPECT_THROW(VariableOffsetContour(contour,
    { OffsetRun(0, 3, 10.0), OffsetRun(2, 4, 10.0) }), // overlap
    std::invalid_argument);
  EXPECT_THROW(VariableOffsetContour(contour,
    { OffsetRun(1, 4, 10.0) }),                        // not starting at 0
    std::invalid_argument);
  EXPECT_THROW(VariableOffsetContour(contour,
    { OffsetRun(0, 5, 10.0) }),                        // past the end
    std::invalid_argument);
  EXPECT_THROW(VariableOffsetContour(contour,
    { OffsetRun(2, 2, 10.0) }),                        // empty run
    std::invalid_argument);
  const Path64 degenerate = MakePath({0, 0});
  EXPECT_THROW(VariableOffsetContour(degenerate,
    { OffsetRun(0, 1, 10.0) }),                        // degenerate contour
    std::invalid_argument);
}

TEST(VariableOffset, RejectsNegativeDelta)
{
  const Path64 contour = OuterContour();
  EXPECT_THROW(VariableOffsetContour(contour,
    { OffsetRun(0, 2, 10.0), OffsetRun(2, 4, -5.0) }),
    std::invalid_argument);
}

TEST(VariableOffset, OpenContourRunsShareBoundaryVertices)
{
  const Path64 open = MakePath({0, 0, 20, 0, 40, 0, 60, 0});
  const std::vector<OffsetRun> runs{
    OffsetRun(0, 2, 10.0), OffsetRun(2, 3, 25.0) }; // N=4 => limit N-1=3

  VariableOffsetOptions opts;
  opts.end_type = EndType::Round;

  const Paths64 fragments = VariableOffsetContour(open, runs, opts);
  const Paths64 merged = Union(fragments, FillRule::Positive);
  const Paths64 reference = ReferenceShape(Paths64{ open }, runs, opts);

  EXPECT_EQ(Flatten(merged), Flatten(reference));
}

TEST(VariableOffset, MiterLimitIsConfigurable)
{
  const Path64 spike = MakePath({0, 0, 100, 50, 0, 100});
  const std::vector<OffsetRun> runs{ OffsetRun(0, 3, 20.0) };

  VariableOffsetOptions tight;
  tight.join_type = JoinType::Miter;
  tight.miter_limit = 1.1;
  VariableOffsetOptions loose = tight;
  loose.miter_limit = 10.0;

  const Paths64 tight_solution =
    VariableOffsetShape(Paths64{ spike }, runs, tight);
  const Paths64 loose_solution =
    VariableOffsetShape(Paths64{ spike }, runs, loose);

  // A generous miter limit reaches further away from the spike tip.
  const PointSet tight_points = Flatten(tight_solution);
  const PointSet loose_points = Flatten(loose_solution);
  EXPECT_NE(tight_points, loose_points);
}

TEST(VariableOffset, DeterministicAcrossRepeatedCalls)
{
  const Paths64 subject{ OuterContour(), HoleContour() };
  const auto runs = TwoRuns(40.0, 15.0);

  const PointSet first =
    Flatten(VariableOffsetShape(subject, runs));
  for (int i = 0; i < 10; ++i)
    EXPECT_EQ(first, Flatten(VariableOffsetShape(subject, runs)));
}
