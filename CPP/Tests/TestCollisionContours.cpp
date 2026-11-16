#include "clipper2/clipper.h"
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <set>
#include <tuple>

using namespace Clipper2Lib;

namespace
{

Path64 DenseSquare(int64_t side, size_t points_per_edge)
{
  Path64 result;
  result.reserve(points_per_edge * 4);
  for (size_t i = 0; i < points_per_edge; ++i)
    result.emplace_back(
      static_cast<int64_t>(i) * side /
        static_cast<int64_t>(points_per_edge), int64_t(0));
  for (size_t i = 0; i < points_per_edge; ++i)
    result.emplace_back(side,
      static_cast<int64_t>(i) * side /
        static_cast<int64_t>(points_per_edge));
  for (size_t i = 0; i < points_per_edge; ++i)
    result.emplace_back(side -
      static_cast<int64_t>(i) * side /
        static_cast<int64_t>(points_per_edge), side);
  for (size_t i = 0; i < points_per_edge; ++i)
    result.emplace_back(int64_t(0), side -
      static_cast<int64_t>(i) * side /
        static_cast<int64_t>(points_per_edge));
  return result;
}

Path64 DenseCircle(size_t count, int64_t radius)
{
  Path64 result;
  result.reserve(count);
  for (size_t i = 0; i < count; ++i)
  {
    const long double angle =
      2.0L * static_cast<long double>(PI) *
      static_cast<long double>(i) / static_cast<long double>(count);
    result.emplace_back(
      static_cast<int64_t>(std::llround(
        static_cast<long double>(radius) * std::cos(angle))),
      static_cast<int64_t>(std::llround(
        static_cast<long double>(radius) * std::sin(angle))));
  }
  return result;
}

size_t CountDuplicateEdges(const CollisionContour& contour)
{
  std::set<std::tuple<int64_t, int64_t, int64_t, int64_t>> edges;
  size_t duplicates = 0;
  for (size_t i = 0; i < contour.path.size(); ++i)
  {
    const Point64& a = contour.path[i];
    const Point64& b = contour.path[(i + 1) % contour.path.size()];
    const auto key = std::make_tuple(
      (std::min)(a.x, b.x), (std::max)(a.x, b.x),
      (std::min)(a.y, b.y), (std::max)(a.y, b.y));
    if (!edges.insert(key).second) ++duplicates;
  }
  return duplicates;
}

} // namespace

TEST(Clipper2Tests, CollisionContoursEmptyAndInvalidOptions)
{
  CollisionContourResult empty =
    SimplifyCollisionContours(Paths64(), {}, {});
  EXPECT_EQ(empty.status, CollisionContourStatus::Empty);
  EXPECT_TRUE(empty.contours.empty());

  Paths64 square = { MakePath({ 0, 0, 10, 0, 10, 10, 0, 10 }) };
  CollisionSimplifyOptions bad;
  bad.tolerance = -1.0;
  EXPECT_EQ(SimplifyCollisionContours(square, {}, {}, bad).status,
    CollisionContourStatus::InvalidOptions);

  bad.tolerance = 1.0;
  bad.max_relative_area_error =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(SimplifyCollisionContours(square, {}, {}, bad).status,
    CollisionContourStatus::InvalidOptions);
}

TEST(Clipper2Tests, CollisionContoursPreserveHoleAndProvenance)
{
  const std::vector<CollisionSourceInput> subjects = {
    CollisionSourceInput{
      7, { MakePath({ 0, 0, 100, 0, 100, 100, 0, 100 }) } }
  };
  const std::vector<CollisionSourceInput> clips = {
    CollisionSourceInput{
      9, { MakePath({ 25, 25, 75, 25, 75, 75, 25, 75 }) } }
  };
  CollisionSimplifyOptions options;
  options.tolerance = 3.0;

  const CollisionContourResult result =
    BuildCollisionContours(ClipType::Difference, FillRule::NonZero,
      subjects, clips, options);
  ASSERT_EQ(result.status, CollisionContourStatus::Ok);
  ASSERT_EQ(result.contours.size(), 2u);
  EXPECT_DOUBLE_EQ(result.area_before, 7500.0);
  EXPECT_DOUBLE_EQ(result.area_after, 7500.0);
  EXPECT_LE(result.relative_area_error, 1e-4);
  EXPECT_EQ(result.unmapped_edge_count, 0u);

  size_t holes = 0;
  for (const CollisionContour& contour : result.contours)
  {
    if (contour.is_hole)
    {
      ++holes;
      EXPECT_LT(Area(contour.path), 0.0);
    }
    else
      EXPECT_GT(Area(contour.path), 0.0);

    ASSERT_EQ(contour.edge_sources.size(), contour.path.size());
    for (size_t i = 0; i < contour.edge_sources.size(); ++i)
    {
      ASSERT_FALSE(contour.edge_sources[i].empty());
      const CollisionSourceSpan& span = contour.edge_sources[i][0];
      EXPECT_EQ(span.role, contour.is_hole ?
        CollisionSourceRole::Clip : CollisionSourceRole::Subject);
      EXPECT_EQ(span.source_id, contour.is_hole ? 9 : 7);
    }
  }
  EXPECT_EQ(holes, 1u);
}

TEST(Clipper2Tests, CollisionContoursRemoveCoincidentEdges)
{
  const Path64 square = MakePath({ 0, 0, 100, 0, 100, 100, 0, 100 });
  const Paths64 duplicated = { square, square };
  const CollisionContourResult result =
    SimplifyCollisionContours(duplicated, {}, {},
      CollisionSimplifyOptions{ 1.0, 1e-4, 0.0 });

  ASSERT_EQ(result.status, CollisionContourStatus::Ok);
  ASSERT_EQ(result.contours.size(), 1u);
  EXPECT_EQ(result.contours[0].path.size(), 4u);
  EXPECT_EQ(CountDuplicateEdges(result.contours[0]), 0u);
  EXPECT_TRUE(result.contours[0].edge_sources.empty());
  EXPECT_EQ(result.unmapped_edge_count, 0u);
  EXPECT_DOUBLE_EQ(result.area_after, 10000.0);
}

TEST(Clipper2Tests, CollisionContoursMergeTouchingEdges)
{
  const Paths64 touching = {
    MakePath({ 0, 0, 100, 0, 100, 100, 0, 100 }),
    MakePath({ 100, 0, 200, 0, 200, 100, 100, 100 })
  };
  const CollisionContourResult result =
    SimplifyCollisionContours(touching, {}, {},
      CollisionSimplifyOptions{ 1.0, 1e-4, 0.0 });

  ASSERT_EQ(result.status, CollisionContourStatus::Ok);
  ASSERT_EQ(result.contours.size(), 1u);
  EXPECT_EQ(result.contours[0].path.size(), 4u);
  EXPECT_EQ(CountDuplicateEdges(result.contours[0]), 0u);
  EXPECT_DOUBLE_EQ(result.area_after, 20000.0);
}

TEST(Clipper2Tests, CollisionContoursAreaFallback)
{
  // A 100x100 block with a narrow 10x10 rectangular notch. The requested
  // tolerance removes the notch and exceeds the 1e-4 area budget, so the
  // implementation must reduce the effective tolerance.
  const Path64 notched = MakePath({
    0, 0, 100, 0, 100, 100, 55, 100,
    55, 90, 45, 90, 45, 100, 0, 100
  });
  CollisionSimplifyOptions options;
  options.tolerance = 20.0;

  const CollisionContourResult result =
    SimplifyCollisionContours({ notched }, {}, {}, options);
  ASSERT_EQ(result.status, CollisionContourStatus::Ok);
  EXPECT_LE(result.relative_area_error, 1e-4);
  EXPECT_LE(result.effective_tolerance, options.tolerance);
  EXPECT_TRUE(result.tolerance_reduced);
  EXPECT_LE(result.absolute_area_error, 1e-4 * result.area_before);
}

TEST(Clipper2Tests, CollisionContoursMapRoundedIntersections)
{
  // Clipper rounds the triangle/rectangle intersection to integer
  // coordinates, so the output edge is no longer exactly collinear with the
  // original triangle edge. Provenance must use the bounded-distance fallback.
  const std::vector<CollisionSourceInput> subjects = {
    CollisionSourceInput{
      1, { MakePath({ 0, 0, 100, 0, 0, 151 }) } }
  };
  const std::vector<CollisionSourceInput> clips = {
    CollisionSourceInput{
      2, { MakePath({ 20, -10, 200, -10, 200, 200, 20, 200 }) } }
  };

  const CollisionContourResult result =
    BuildCollisionContours(ClipType::Intersection, FillRule::NonZero,
      subjects, clips, CollisionSimplifyOptions{ 1.0, 1e-4, 0.0 });
  ASSERT_EQ(result.status, CollisionContourStatus::Ok);
  ASSERT_EQ(result.contours.size(), 1u);
  EXPECT_EQ(result.unmapped_edge_count, 0u);

  bool found_triangle_edge = false;
  for (const auto& edge_sources : result.contours[0].edge_sources)
    for (const CollisionSourceSpan& span : edge_sources)
      if (span.source_id == 1 && span.first_edge == 1)
        found_triangle_edge = true;
  EXPECT_TRUE(found_triangle_edge);
}

TEST(Clipper2Tests, CollisionContoursHundredThousandPoints)
{
  const size_t point_count = 100000;
  const Path64 circle = DenseCircle(point_count, 1000000);
  std::vector<CollisionSourceInput> sources = {
    CollisionSourceInput{ 42, { circle } }
  };

  CollisionSimplifyOptions options;
  options.tolerance = 20.0;
  const CollisionContourResult result =
    SimplifyCollisionContours({ circle }, sources, {}, options);

  ASSERT_EQ(result.status, CollisionContourStatus::Ok);
  ASSERT_EQ(result.contours.size(), 1u);
  EXPECT_EQ(result.raw_input_vertex_count, point_count);
  EXPECT_GT(result.input_vertex_count, 50000u);
  EXPECT_LT(result.output_vertex_count, 2000u);
  EXPECT_LE(result.relative_area_error, 1e-4);
  EXPECT_EQ(result.unmapped_edge_count, 0u);
  EXPECT_GT(result.contours[0].path.size(), 2u);
  EXPECT_FALSE(result.contours[0].edge_sources.empty());
}

TEST(Clipper2Tests, CollisionContoursDenseCollinearSourceSpans)
{
  const Path64 square = DenseSquare(100000, 25000);
  std::vector<CollisionSourceInput> sources = {
    CollisionSourceInput{ 99, { square } }
  };

  const CollisionContourResult result =
    SimplifyCollisionContours({ square }, sources, {},
      CollisionSimplifyOptions{ 2.0, 1e-4, 0.0 });

  ASSERT_EQ(result.status, CollisionContourStatus::Ok);
  ASSERT_EQ(result.input_vertex_count, 4u);
  ASSERT_EQ(result.contours.size(), 1u);
  EXPECT_EQ(result.contours[0].path.size(), 4u);
  EXPECT_EQ(result.unmapped_edge_count, 0u);
  EXPECT_DOUBLE_EQ(result.area_before, 1e10);
  EXPECT_DOUBLE_EQ(result.area_after, 1e10);

  for (const auto& edge_sources : result.contours[0].edge_sources)
  {
    ASSERT_EQ(edge_sources.size(), 1u);
    EXPECT_EQ(edge_sources[0].source_id, 99);
  }
}
