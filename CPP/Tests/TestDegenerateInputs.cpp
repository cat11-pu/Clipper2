#include <gtest/gtest.h>
#include "clipper2/clipper.h"

using namespace Clipper2Lib;

static Point64 P(int64_t x, int64_t y) { return Point64(x, y); }

TEST(Clipper2Tests, RejectOutOfRangeCoordinate)
{
  const Point64 bad = P(MAX_COORD + 1, 5);
  Clipper64 c;
  Paths64 solution;
  c.AddSubject(Paths64{ Path64{ P(0, 0), P(10, 0), bad } });
  EXPECT_FALSE(c.Execute(ClipType::Union, FillRule::NonZero, solution));
  EXPECT_NE(c.ErrorCode() & range_error_i, 0);
  EXPECT_EQ(c.ErrorLocation(), bad);
  EXPECT_EQ(c.ErrorPathIndex(), 0);
  EXPECT_EQ(c.ErrorVertexIndex(), 2);
}

TEST(Clipper2Tests, RejectCollapsedDuplicatePoints)
{
  const Point64 pt = P(7, 11);
  Clipper64 c;
  Paths64 solution;
  c.AddSubject(Paths64{ Path64{ pt, pt, pt, pt } });
  EXPECT_FALSE(c.Execute(ClipType::Union, FillRule::NonZero, solution));
  EXPECT_NE(c.ErrorCode() & degenerate_error_i, 0);
  EXPECT_EQ(c.ErrorLocation(), pt);
}

TEST(Clipper2Tests, RejectTwoPointClosedPath)
{
  Clipper64 c;
  Paths64 solution;
  c.AddSubject(Paths64{ Path64{ P(0, 0), P(10, 10) } });
  EXPECT_FALSE(c.Execute(ClipType::Union, FillRule::NonZero, solution));
  EXPECT_NE(c.ErrorCode() & degenerate_error_i, 0);
  EXPECT_EQ(c.ErrorLocation(), P(10, 10));
}

TEST(Clipper2Tests, RejectCollinearClosedPath)
{
  Clipper64 c;
  Paths64 solution;
  c.AddSubject(Paths64{ Path64{
    P(0, 0), P(10, 0), P(20, 0) } });
  EXPECT_FALSE(c.Execute(ClipType::Union, FillRule::NonZero, solution));
  EXPECT_NE(c.ErrorCode() & degenerate_error_i, 0);
  EXPECT_EQ(c.ErrorLocation(), P(0, 0));
}

TEST(Clipper2Tests, RejectTinyEdgeInLargePath)
{
  Clipper64 c;
  Paths64 solution;
  c.AddSubject(Paths64{ Path64{
    P(-MAX_COORD, 0), P(MAX_COORD, 0),
    P(MAX_COORD, 1), P(-MAX_COORD, 1) } });
  EXPECT_FALSE(c.Execute(ClipType::Union, FillRule::NonZero, solution));
  EXPECT_NE(c.ErrorCode() & degenerate_error_i, 0);
  EXPECT_EQ(c.ErrorLocation(), P(MAX_COORD, 1));
}

TEST(Clipper2Tests, RejectPathDCollapsedByScaling)
{
  const PointD collapsed(0.001, 0.0);
  ClipperD c(2);
  PathsD solution;
  c.AddSubject(PathsD{ PathD{
    PointD(0, 0), collapsed, PointD(10, 0),
    PointD(10, 10), PointD(0, 10) } });
  EXPECT_FALSE(c.Execute(ClipType::Union, FillRule::NonZero, solution));
  EXPECT_NE(c.ErrorCode() & degenerate_error_i, 0);
  EXPECT_EQ(c.ErrorLocationD(), collapsed);
}

TEST(Clipper2Tests, RejectUnstableNearCollinearIntersection)
{
  const Path64 subject = MakePath(std::vector<int64_t>{
    -1152921505606846975LL, -1152921504106846976LL,
    -1152921506606846975LL, -1152921503606846975LL,
    -1152921506606845975LL, -1152921503606847475LL,
    -1152921506606845976LL, -1152921503606847476LL });
  const Path64 clip = MakePath(std::vector<int64_t>{
    -768614336404564650LL, -768614336404564650LL,
    -768614336404564652LL, -768614336404564650LL,
    -768614336404564650LL, -768614336404564652LL,
    -768614336404564648LL, -768614336404564654LL,
    -768614336404564647LL, -768614336404564654LL });

  Clipper64 c;
  Paths64 solution;
  c.AddSubject(Paths64{ subject });
  c.AddClip(Paths64{ clip });
  const bool ok = c.Execute(ClipType::Union, FillRule::NonZero, solution);
#if defined(CLIPPER2_HI_PRECISION) && CLIPPER2_HI_PRECISION
  EXPECT_TRUE(ok);
  EXPECT_EQ(c.ErrorCode(), 0);
#else
  EXPECT_FALSE(ok);
  EXPECT_NE(c.ErrorCode() & intersection_error_i, 0);
  EXPECT_NE(c.ErrorLocation(), InvalidPoint64);
#endif
}

TEST(Clipper2Tests, LegalInputRemainsUnchanged)
{
  const Paths64 subject = { MakePath(std::vector<int64_t>{
    0, 0, 100, 0, 100, 100, 0, 100 }) };
  const Paths64 clip = { MakePath(std::vector<int64_t>{
    50, 50, 150, 50, 150, 150, 50, 150 }) };

  Clipper64 c;
  Paths64 solution;
  c.AddSubject(subject);
  c.AddClip(clip);
  EXPECT_TRUE(c.Execute(ClipType::Union, FillRule::NonZero, solution));
  EXPECT_EQ(c.ErrorCode(), 0);
  EXPECT_EQ(solution.size(), 1);
  EXPECT_DOUBLE_EQ(Area(solution), 17500.0);
}
