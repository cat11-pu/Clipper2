#include "clipper2/clipper.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
using namespace Clipper2Lib;

constexpr uint64_t kDefaultSeed = 0x5eedc11f2e2a2026ULL;
constexpr int kDefaultCases = 200;
constexpr int kMaxReportedIssues = 32;

enum class CaseStyle
{
  Holes,
  SelfIntersections,
  CoincidentEdges,
  TinyAngles
};

struct TestCase
{
  int id = 0;
  CaseStyle style = CaseStyle::Holes;
  ClipType clip_type = ClipType::Union;
  FillRule fill_rule = FillRule::NonZero;
  Paths64 subject;
  Paths64 clip;
};

struct CaseResult
{
  Paths64 output;
  std::vector<std::string> issues;
};

struct SegmentHit
{
  bool intersects = false;
  bool overlap = false;
  bool endpoint_endpoint = false;
  Point64 point;
};

class Rng
{
public:
  explicit Rng(uint64_t seed) : gen_(seed) {}

  int64_t Int(int64_t lo, int64_t hi)
  {
    std::uniform_int_distribution<int64_t> dist(lo, hi);
    return dist(gen_);
  }

  bool Chance(int percent)
  {
    return percent > 0 && Int(1, 100) <= percent;
  }

private:
  std::mt19937_64 gen_;
};

const char* StyleName(CaseStyle style)
{
  switch (style)
  {
    case CaseStyle::Holes: return "holes";
    case CaseStyle::SelfIntersections: return "self_intersections";
    case CaseStyle::CoincidentEdges: return "coincident_edges";
    case CaseStyle::TinyAngles: return "tiny_angles";
  }
  return "unknown";
}

const char* ClipTypeName(ClipType type)
{
  switch (type)
  {
    case ClipType::Intersection: return "Intersection";
    case ClipType::Union: return "Union";
    case ClipType::Difference: return "Difference";
    case ClipType::Xor: return "Xor";
  }
  return "Unknown";
}

const char* FillRuleName(FillRule rule)
{
  switch (rule)
  {
    case FillRule::EvenOdd: return "EvenOdd";
    case FillRule::NonZero: return "NonZero";
    case FillRule::Positive: return "Positive";
    case FillRule::Negative: return "Negative";
  }
  return "Unknown";
}

ClipType PickClipType(Rng& rng)
{
  static const ClipType types[] = {
    ClipType::Intersection,
    ClipType::Union,
    ClipType::Difference,
    ClipType::Xor
  };
  return types[rng.Int(0, 3)];
}

FillRule PickFillRule(Rng& rng)
{
  static const FillRule rules[] = {
    FillRule::EvenOdd,
    FillRule::NonZero,
    FillRule::Positive,
    FillRule::Negative
  };
  return rules[rng.Int(0, 3)];
}

Path64 RectPath(int64_t x0, int64_t y0, int64_t x1, int64_t y1)
{
  if (x0 > x1) std::swap(x0, x1);
  if (y0 > y1) std::swap(y0, y1);
  return {
    Point64(x0, y0),
    Point64(x1, y0),
    Point64(x1, y1),
    Point64(x0, y1)
  };
}

Path64 MakeEllipticPolygon(Rng& rng, int64_t cx, int64_t cy,
  int64_t radius, int count)
{
  constexpr double two_pi = 6.28318530717958647692;
  const double offset = static_cast<double>(rng.Int(0, 1000000)) /
    1000000.0 * two_pi;
  Path64 result;
  result.reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i)
  {
    const double angle = offset + two_pi * static_cast<double>(i) /
      static_cast<double>(count);
    const int64_t r = radius + rng.Int(-radius / 12, radius / 14);
    const int64_t x = cx + static_cast<int64_t>(
      std::llround(std::cos(angle) * static_cast<double>(r)));
    const int64_t y = cy + static_cast<int64_t>(
      std::llround(std::sin(angle) * static_cast<double>(r)));
    if (result.empty() || result.back() != Point64(x, y))
      result.emplace_back(x, y);
  }
  if (result.size() > 1 && result.front() == result.back())
    result.pop_back();
  return result;
}

Path64 MakeSelfIntersectingStar(Rng& rng)
{
  const int count = static_cast<int>(rng.Int(5, 9));
  int step = static_cast<int>(rng.Int(2, count - 1));
  while (std::gcd(step, count) != 1)
    step = step % (count - 1) + 2;

  constexpr double two_pi = 6.28318530717958647692;
  const double offset = static_cast<double>(rng.Int(0, 1000000)) /
    1000000.0 * two_pi;
  const int64_t cx = rng.Int(-120, 120);
  const int64_t cy = rng.Int(-120, 120);
  const int64_t radius = rng.Int(60, 180);
  Path64 points;
  points.reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i)
  {
    const double angle = offset + two_pi * static_cast<double>(i) /
      static_cast<double>(count);
    const int64_t r = radius + rng.Int(-radius / 10, radius / 10);
    points.emplace_back(
      cx + static_cast<int64_t>(
        std::llround(std::cos(angle) * static_cast<double>(r))),
      cy + static_cast<int64_t>(
        std::llround(std::sin(angle) * static_cast<double>(r))));
  }

  Path64 result;
  result.reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i)
    result.push_back(points[static_cast<size_t>((i * step) % count)]);
  return result;
}

Path64 MakeTinyAngleTriangle(Rng& rng, bool below)
{
  const int64_t length = rng.Int(40, 500);
  const int64_t height = rng.Int(1, 3);
  const int64_t x = rng.Int(-160, 80);
  const int64_t y = rng.Int(-120, 120);
  const int64_t signed_height = below ? -height : height;
  return {
    Point64(x, y),
    Point64(x + length, y),
    Point64(x + length, y + signed_height)
  };
}

void AddRandomHoles(Rng& rng, int64_t cx, int64_t cy, Paths64& paths)
{
  const int hole_count = static_cast<int>(rng.Int(1, 3));
  for (int i = 0; i < hole_count; ++i)
  {
    const int64_t radius = rng.Int(8, 20);
    const int64_t distance = rng.Int(22, 48);
    const int64_t angle_index = rng.Int(0, 7);
    const int64_t hx = cx + (angle_index & 1 ? distance : -distance) +
      rng.Int(-6, 6);
    const int64_t hy = cy + (angle_index & 2 ? distance : -distance) +
      rng.Int(-6, 6);
    const int count = static_cast<int>(rng.Int(5, 9));
    paths.push_back(MakeEllipticPolygon(rng, hx, hy, radius, count));
  }
}

TestCase GenerateHolesCase(Rng& rng, int id)
{
  TestCase tc;
  tc.id = id;
  tc.style = CaseStyle::Holes;
  tc.clip_type = PickClipType(rng);
  tc.fill_rule = PickFillRule(rng);

  const int64_t cx = rng.Int(-60, 60);
  const int64_t cy = rng.Int(-60, 60);
  const int64_t radius = rng.Int(120, 190);
  tc.subject.push_back(MakeEllipticPolygon(rng, cx, cy, radius,
    static_cast<int>(rng.Int(6, 11))));
  AddRandomHoles(rng, cx, cy, tc.subject);

  const int64_t clip_cx = cx + rng.Int(-90, 90);
  const int64_t clip_cy = cy + rng.Int(-90, 90);
  const int64_t clip_radius = rng.Int(70, 170);
  tc.clip.push_back(MakeEllipticPolygon(rng, clip_cx, clip_cy, clip_radius,
    static_cast<int>(rng.Int(5, 10))));
  if (rng.Chance(70))
    AddRandomHoles(rng, clip_cx, clip_cy, tc.clip);
  return tc;
}

TestCase GenerateSelfIntersectionCase(Rng& rng, int id)
{
  TestCase tc;
  tc.id = id;
  tc.style = CaseStyle::SelfIntersections;
  tc.clip_type = PickClipType(rng);
  tc.fill_rule = PickFillRule(rng);

  tc.subject.push_back(MakeSelfIntersectingStar(rng));
  if (rng.Chance(35))
    tc.subject.push_back(MakeSelfIntersectingStar(rng));

  if (rng.Chance(55))
    tc.clip.push_back(MakeSelfIntersectingStar(rng));
  else if (rng.Chance(50))
  {
    const int64_t cx = rng.Int(-100, 100);
    const int64_t cy = rng.Int(-100, 100);
    tc.clip.push_back(MakeEllipticPolygon(rng, cx, cy, rng.Int(40, 150),
      static_cast<int>(rng.Int(5, 10))));
  }
  return tc;
}

TestCase GenerateCoincidentEdgeCase(Rng& rng, int id)
{
  TestCase tc;
  tc.id = id;
  tc.style = CaseStyle::CoincidentEdges;
  tc.clip_type = PickClipType(rng);
  tc.fill_rule = PickFillRule(rng);

  const int64_t x0 = rng.Int(-180, -30);
  const int64_t x1 = rng.Int(-10, 10);
  const int64_t x2 = x1 + rng.Int(35, 150);
  const int64_t y0 = rng.Int(-120, 20);
  const int64_t y1 = y0 + rng.Int(35, 130);
  const int64_t y2 = y1 + rng.Int(25, 100);

  tc.subject.push_back(RectPath(x0, y0, x1, y1));
  tc.clip.push_back(RectPath(x1, y0, x2, y1));
  if (rng.Chance(60))
  {
    const int64_t y3 = y0 - rng.Int(25, 100);
    tc.clip.push_back(RectPath(x1, y3, x2, y0));
  }
  if (rng.Chance(45))
    tc.subject.push_back(RectPath(x0, y1, x1, y2));
  if (rng.Chance(35))
    tc.subject.push_back(RectPath(x1, y1, x2, y2));
  return tc;
}

TestCase GenerateTinyAngleCase(Rng& rng, int id)
{
  TestCase tc;
  tc.id = id;
  tc.style = CaseStyle::TinyAngles;
  tc.clip_type = PickClipType(rng);
  tc.fill_rule = PickFillRule(rng);

  tc.subject.push_back(MakeTinyAngleTriangle(rng, false));
  if (rng.Chance(35))
    tc.subject.push_back(MakeTinyAngleTriangle(rng, true));

  if (!rng.Chance(35))
    tc.clip.push_back(MakeTinyAngleTriangle(rng, true));
  return tc;
}

TestCase GenerateCase(Rng& rng, int id)
{
  switch (id % 4)
  {
    case 0: return GenerateHolesCase(rng, id);
    case 1: return GenerateSelfIntersectionCase(rng, id);
    case 2: return GenerateCoincidentEdgeCase(rng, id);
    default: return GenerateTinyAngleCase(rng, id);
  }
}

__int128 Orient(const Point64& a, const Point64& b, const Point64& c)
{
  return static_cast<__int128>(b.x - a.x) *
    static_cast<__int128>(c.y - a.y) -
    static_cast<__int128>(b.y - a.y) *
    static_cast<__int128>(c.x - a.x);
}

bool InRange(int64_t value, int64_t a, int64_t b)
{
  return value >= std::min(a, b) && value <= std::max(a, b);
}

bool OnSegment(const Point64& a, const Point64& b, const Point64& p)
{
  return Orient(a, b, p) == 0 &&
    InRange(p.x, a.x, b.x) && InRange(p.y, a.y, b.y);
}

bool IsEndpoint(const Point64& a, const Point64& b, const Point64& p)
{
  return p == a || p == b;
}

SegmentHit IntersectSegments(const Point64& a, const Point64& b,
  const Point64& c, const Point64& d)
{
  SegmentHit hit;
  const __int128 o1 = Orient(a, b, c);
  const __int128 o2 = Orient(a, b, d);
  const __int128 o3 = Orient(c, d, a);
  const __int128 o4 = Orient(c, d, b);

  const bool collinear = o1 == 0 && o2 == 0 && o3 == 0 && o4 == 0;
  if (collinear)
  {
    const bool use_x = std::abs(b.x - a.x) >= std::abs(b.y - a.y);
    const int64_t a0 = use_x ? a.x : a.y;
    const int64_t a1 = use_x ? b.x : b.y;
    const int64_t b0 = use_x ? c.x : c.y;
    const int64_t b1 = use_x ? d.x : d.y;
    const int64_t lo = std::max(std::min(a0, a1), std::min(b0, b1));
    const int64_t hi = std::min(std::max(a0, a1), std::max(b0, b1));
    if (lo > hi) return hit;

    hit.intersects = true;
    hit.overlap = lo < hi;
    if (!hit.overlap)
    {
      const Point64 candidates[] = {a, b, c, d};
      for (const Point64& candidate : candidates)
      {
        if (OnSegment(a, b, candidate) && OnSegment(c, d, candidate))
        {
          hit.point = candidate;
          break;
        }
      }
      hit.endpoint_endpoint =
        IsEndpoint(a, b, hit.point) && IsEndpoint(c, d, hit.point);
    }
    return hit;
  }

  const bool proper_crossing =
    ((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) &&
    ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0));

  Point64 point;
  bool found = false;
  if (proper_crossing)
  {
    found = true;
  }
  else if (o1 == 0 && OnSegment(a, b, c))
  {
    point = c;
    found = true;
  }
  else if (o2 == 0 && OnSegment(a, b, d))
  {
    point = d;
    found = true;
  }
  else if (o3 == 0 && OnSegment(c, d, a))
  {
    point = a;
    found = true;
  }
  else if (o4 == 0 && OnSegment(c, d, b))
  {
    point = b;
    found = true;
  }

  if (!found) return hit;
  hit.intersects = true;
  hit.point = point;
  hit.endpoint_endpoint = !proper_crossing &&
    IsEndpoint(a, b, point) && IsEndpoint(c, d, point);
  return hit;
}

void AddIssue(std::vector<std::string>& issues, const std::string& issue)
{
  if (issues.size() < kMaxReportedIssues)
    issues.push_back(issue);
}

bool PathsEqual(const Paths64& lhs, const Paths64& rhs)
{
  if (lhs.size() != rhs.size()) return false;
  for (size_t i = 0; i < lhs.size(); ++i)
  {
    if (lhs[i].size() != rhs[i].size()) return false;
    for (size_t j = 0; j < lhs[i].size(); ++j)
      if (lhs[i][j] != rhs[i][j]) return false;
  }
  return true;
}

std::vector<std::string> CheckOutput(const Paths64& output)
{
  std::vector<std::string> issues;
  for (size_t p = 0; p < output.size(); ++p)
  {
    const Path64& path = output[p];
    if (path.size() < 3)
    {
      AddIssue(issues, "degenerate_path(path=" + std::to_string(p) + ")");
      continue;
    }

    const size_t n = path.size();
    for (size_t i = 0; i < n; ++i)
    {
      if (path[i] == path[(i + 1) % n])
      {
        AddIssue(issues, "zero_length_edge(path=" + std::to_string(p) +
          ",edge=" + std::to_string(i) + ")");
      }
    }

    for (size_t i = 0; i < n; ++i)
    {
      const size_t i2 = (i + 1) % n;
      for (size_t j = i + 1; j < n; ++j)
      {
        const size_t j2 = (j + 1) % n;
        const SegmentHit hit = IntersectSegments(
          path[i], path[i2], path[j], path[j2]);
        if (!hit.intersects) continue;

        const bool adjacent = (j == i + 1) || (i == 0 && j2 == 0);
        if (adjacent)
        {
          const Point64 shared = (j == i + 1) ? path[j] : path[i];
          if (hit.overlap)
          {
            AddIssue(issues, "adjacent_overlap(path=" + std::to_string(p) +
              ",edges=" + std::to_string(i) + "," + std::to_string(j) + ")");
          }
          else if (hit.point != shared)
          {
            AddIssue(issues, "self_intersection(path=" + std::to_string(p) +
              ",edges=" + std::to_string(i) + "," + std::to_string(j) + ")");
          }
        }
        else
        {
          AddIssue(issues, "self_intersection(path=" + std::to_string(p) +
            ",edges=" + std::to_string(i) + "," + std::to_string(j) + ")");
        }
      }
    }
  }

  for (size_t p = 0; p < output.size(); ++p)
  {
    const Path64& first = output[p];
    if (first.size() < 2) continue;
    for (size_t q = p + 1; q < output.size(); ++q)
    {
      const Path64& second = output[q];
      if (second.size() < 2) continue;
      for (size_t i = 0; i < first.size(); ++i)
      {
        const size_t i2 = (i + 1) % first.size();
        for (size_t j = 0; j < second.size(); ++j)
        {
          const size_t j2 = (j + 1) % second.size();
          const SegmentHit hit = IntersectSegments(
            first[i], first[i2], second[j], second[j2]);
          if (!hit.intersects || hit.endpoint_endpoint) continue;
          const char* kind = hit.overlap ? "path_overlap" : "path_intersection";
          AddIssue(issues, std::string(kind) + "(paths=" +
            std::to_string(p) + "," + std::to_string(q) + ",edges=" +
            std::to_string(i) + "," + std::to_string(j) + ")");
        }
      }
    }
  }
  return issues;
}

CaseResult ExecuteCase(const TestCase& tc)
{
  CaseResult result;
  try
  {
    const Paths64 output_a = BooleanOp(
      tc.clip_type, tc.fill_rule, tc.subject, tc.clip);
    const Paths64 output_b = BooleanOp(
      tc.clip_type, tc.fill_rule, tc.subject, tc.clip);
    result.output = output_a;
    if (!PathsEqual(output_a, output_b))
      AddIssue(result.issues, "nondeterministic_output");
    const auto geometry_issues = CheckOutput(output_a);
    result.issues.insert(result.issues.end(),
      geometry_issues.begin(), geometry_issues.end());
  }
  catch (const std::exception& ex)
  {
    AddIssue(result.issues, std::string("exception:") + ex.what());
  }
  catch (...)
  {
    AddIssue(result.issues, "exception:unknown");
  }
  return result;
}

bool IsFailure(const TestCase& tc)
{
  return !ExecuteCase(tc).issues.empty();
}

void TranslateToOrigin(TestCase& tc)
{
  int64_t min_x = INT64_MAX;
  int64_t min_y = INT64_MAX;
  auto visit = [&](const Paths64& paths)
  {
    for (const Path64& path : paths)
      for (const Point64& pt : path)
      {
        min_x = std::min(min_x, pt.x);
        min_y = std::min(min_y, pt.y);
      }
  };
  visit(tc.subject);
  visit(tc.clip);
  if (min_x == INT64_MAX) return;
  if (min_x == 0 && min_y == 0) return;
  auto translate = [&](Paths64& paths)
  {
    for (Path64& path : paths)
      for (Point64& pt : path)
      {
        pt.x -= min_x;
        pt.y -= min_y;
      }
  };
  translate(tc.subject);
  translate(tc.clip);
}

int64_t CoordinateGcd(const TestCase& tc)
{
  int64_t value = 0;
  auto visit = [&](const Paths64& paths)
  {
    for (const Path64& path : paths)
      for (const Point64& pt : path)
      {
        value = std::gcd(value, std::abs(pt.x));
        value = std::gcd(value, std::abs(pt.y));
      }
  };
  visit(tc.subject);
  visit(tc.clip);
  return value;
}

TestCase MinimizeFailure(const TestCase& original)
{
  TestCase current = original;
  TranslateToOrigin(current);
  if (!IsFailure(current))
    current = original;

  bool changed = true;
  while (changed)
  {
    changed = false;

    for (size_t i = 0; i < current.subject.size(); ++i)
    {
      TestCase candidate = current;
      candidate.subject.erase(candidate.subject.begin() +
        static_cast<std::ptrdiff_t>(i));
      if (IsFailure(candidate))
      {
        current = std::move(candidate);
        changed = true;
        break;
      }
    }
    if (changed) continue;

    for (size_t i = 0; i < current.clip.size(); ++i)
    {
      TestCase candidate = current;
      candidate.clip.erase(candidate.clip.begin() +
        static_cast<std::ptrdiff_t>(i));
      if (IsFailure(candidate))
      {
        current = std::move(candidate);
        changed = true;
        break;
      }
    }
    if (changed) continue;

    for (size_t p = 0; p < current.subject.size() && !changed; ++p)
    {
      for (size_t i = 0; i < current.subject[p].size() && !changed; ++i)
      {
        if (current.subject[p].size() <= 3) break;
        TestCase candidate = current;
        candidate.subject[p].erase(candidate.subject[p].begin() +
          static_cast<std::ptrdiff_t>(i));
        if (IsFailure(candidate))
        {
          current = std::move(candidate);
          changed = true;
        }
      }
    }
    if (changed) continue;

    for (size_t p = 0; p < current.clip.size() && !changed; ++p)
    {
      for (size_t i = 0; i < current.clip[p].size() && !changed; ++i)
      {
        if (current.clip[p].size() <= 3) break;
        TestCase candidate = current;
        candidate.clip[p].erase(candidate.clip[p].begin() +
          static_cast<std::ptrdiff_t>(i));
        if (IsFailure(candidate))
        {
          current = std::move(candidate);
          changed = true;
        }
      }
    }
  }

  const int64_t gcd = CoordinateGcd(current);
  if (gcd > 1)
  {
    TestCase candidate = current;
    auto divide = [&](Paths64& paths)
    {
      for (Path64& path : paths)
        for (Point64& pt : path)
        {
          pt.x /= gcd;
          pt.y /= gcd;
        }
    };
    divide(candidate.subject);
    divide(candidate.clip);
    if (IsFailure(candidate))
      current = std::move(candidate);
  }
  return current;
}

std::string JsonPoint(const Point64& pt)
{
  return "[" + std::to_string(pt.x) + "," + std::to_string(pt.y) + "]";
}

std::string JsonPath(const Path64& path)
{
  std::ostringstream out;
  out << "[";
  for (size_t i = 0; i < path.size(); ++i)
  {
    if (i) out << ",";
    out << JsonPoint(path[i]);
  }
  out << "]";
  return out.str();
}

std::string JsonPaths(const Paths64& paths)
{
  std::ostringstream out;
  out << "[";
  for (size_t i = 0; i < paths.size(); ++i)
  {
    if (i) out << ",";
    out << JsonPath(paths[i]);
  }
  out << "]";
  return out.str();
}

std::string JsonString(const std::string& value)
{
  std::ostringstream out;
  out << '"';
  for (char ch : value)
  {
    switch (ch)
    {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default: out << ch; break;
    }
  }
  out << '"';
  return out.str();
}

std::string JsonInput(const TestCase& tc)
{
  std::ostringstream out;
  out << "{\"case\":" << tc.id
      << ",\"style\":" << JsonString(StyleName(tc.style))
      << ",\"clip_type\":" << JsonString(ClipTypeName(tc.clip_type))
      << ",\"fill_rule\":" << JsonString(FillRuleName(tc.fill_rule))
      << ",\"subject\":" << JsonPaths(tc.subject)
      << ",\"clip\":" << JsonPaths(tc.clip)
      << "}";
  return out.str();
}

std::string JsonIssues(const std::vector<std::string>& issues)
{
  std::ostringstream out;
  out << "[";
  for (size_t i = 0; i < issues.size(); ++i)
  {
    if (i) out << ",";
    out << JsonString(issues[i]);
  }
  out << "]";
  return out.str();
}

std::string JsonCaseRecord(const TestCase& tc, const CaseResult& result,
  const TestCase* minimized, const CaseResult* minimized_result)
{
  std::ostringstream out;
  out << "{\"case\":" << tc.id
      << ",\"style\":" << JsonString(StyleName(tc.style))
      << ",\"clip_type\":" << JsonString(ClipTypeName(tc.clip_type))
      << ",\"fill_rule\":" << JsonString(FillRuleName(tc.fill_rule))
      << ",\"subject\":" << JsonPaths(tc.subject)
      << ",\"clip\":" << JsonPaths(tc.clip)
      << ",\"output\":" << JsonPaths(result.output)
      << ",\"issues\":" << JsonIssues(result.issues)
      << ",\"passed\":" << (result.issues.empty() ? "true" : "false");
  if (minimized && minimized_result)
  {
    out << ",\"minimized\":" << JsonInput(*minimized)
        << ",\"minimized_output\":" << JsonPaths(minimized_result->output)
        << ",\"minimized_issues\":" << JsonIssues(minimized_result->issues);
  }
  out << "}";
  return out.str();
}

void WriteTextFile(const std::filesystem::path& path,
  const std::string& contents)
{
  std::ofstream out(path, std::ios::binary);
  if (!out)
    throw std::runtime_error("cannot write " + path.string());
  out << contents;
  if (!out)
    throw std::runtime_error("write failed for " + path.string());
}

struct StyleCounts
{
  int holes = 0;
  int self_intersections = 0;
  int coincident_edges = 0;
  int tiny_angles = 0;
};

void CountStyle(StyleCounts& counts, CaseStyle style)
{
  switch (style)
  {
    case CaseStyle::Holes: ++counts.holes; break;
    case CaseStyle::SelfIntersections: ++counts.self_intersections; break;
    case CaseStyle::CoincidentEdges: ++counts.coincident_edges; break;
    case CaseStyle::TinyAngles: ++counts.tiny_angles; break;
  }
}

uint64_t ParseSeed(const std::string& text)
{
  size_t consumed = 0;
  const uint64_t value = std::stoull(text, &consumed, 0);
  if (consumed != text.size())
    throw std::runtime_error("invalid seed: " + text);
  return value;
}

void PrintUsage()
{
  std::cout
    << "usage: RandomPolygonProperties [--cases N] [--seed S] [--out DIR]\n"
    << "       [--no-shrink] [--quiet]\n";
}

} // namespace

int main(int argc, char** argv)
{
  int case_count = kDefaultCases;
  uint64_t seed = kDefaultSeed;
  std::filesystem::path output_dir = "property-artifacts";
  bool shrink = true;
  bool quiet = false;

  try
  {
    for (int i = 1; i < argc; ++i)
    {
      const std::string arg = argv[i];
      if (arg == "--cases" && i + 1 < argc)
      {
        case_count = std::stoi(argv[++i]);
      }
      else if (arg == "--seed" && i + 1 < argc)
      {
        seed = ParseSeed(argv[++i]);
      }
      else if (arg == "--out" && i + 1 < argc)
      {
        output_dir = argv[++i];
      }
      else if (arg == "--no-shrink")
      {
        shrink = false;
      }
      else if (arg == "--quiet")
      {
        quiet = true;
      }
      else if (arg == "--help" || arg == "-h")
      {
        PrintUsage();
        return 0;
      }
      else
      {
        throw std::runtime_error("unknown argument: " + arg);
      }
    }
    if (case_count < 0)
      throw std::runtime_error("--cases must be non-negative");
  }
  catch (const std::exception& ex)
  {
    std::cerr << "argument error: " << ex.what() << "\n";
    PrintUsage();
    return 2;
  }

  std::filesystem::create_directories(output_dir);
  Rng rng(seed);
  StyleCounts style_counts;
  std::vector<TestCase> cases;
  cases.reserve(static_cast<size_t>(case_count));
  for (int i = 0; i < case_count; ++i)
  {
    TestCase tc = GenerateCase(rng, i);
    CountStyle(style_counts, tc.style);
    cases.push_back(std::move(tc));
  }

  int passed = 0;
  int failed = 0;
  int nondeterministic = 0;
  std::vector<std::string> records;
  std::vector<std::string> counterexamples;
  records.reserve(cases.size());
  counterexamples.reserve(cases.size());

  for (const TestCase& tc : cases)
  {
    CaseResult result = ExecuteCase(tc);
    if (result.issues.empty())
    {
      ++passed;
      records.push_back(JsonCaseRecord(tc, result, nullptr, nullptr));
      continue;
    }

    ++failed;
    const bool has_nondeterminism = std::find(result.issues.begin(),
      result.issues.end(), "nondeterministic_output") != result.issues.end();
    if (has_nondeterminism) ++nondeterministic;

    TestCase minimized = shrink ? MinimizeFailure(tc) : tc;
    CaseResult minimized_result = ExecuteCase(minimized);
    records.push_back(JsonCaseRecord(tc, result, &minimized,
      &minimized_result));
    counterexamples.push_back(JsonCaseRecord(tc, result, &minimized,
      &minimized_result));

    if (!quiet)
    {
      std::cout << "FAIL case=" << tc.id
                << " style=" << StyleName(tc.style)
                << " clip=" << ClipTypeName(tc.clip_type)
                << " fill=" << FillRuleName(tc.fill_rule)
                << " issues=" << JsonIssues(result.issues) << "\n";
      std::cout << "MINIMAL " << JsonInput(minimized) << "\n";
      std::cout << "MINIMAL_OUTPUT " << JsonPaths(minimized_result.output)
                << "\n";
      std::cout << "MINIMAL_ISSUES " << JsonIssues(minimized_result.issues)
                << "\n";
    }
  }

  std::ostringstream outputs_jsonl;
  for (const std::string& record : records)
    outputs_jsonl << record << "\n";

  std::ostringstream counterexamples_json;
  counterexamples_json << "[\n";
  for (size_t i = 0; i < counterexamples.size(); ++i)
  {
    counterexamples_json << "  " << counterexamples[i];
    if (i + 1 < counterexamples.size()) counterexamples_json << ",";
    counterexamples_json << "\n";
  }
  counterexamples_json << "]\n";

  std::ostringstream summary;
  summary << "{\n"
          << "  \"seed\": " << seed << ",\n"
          << "  \"cases\": " << case_count << ",\n"
          << "  \"passed\": " << passed << ",\n"
          << "  \"failed\": " << failed << ",\n"
          << "  \"nondeterministic\": " << nondeterministic << ",\n"
          << "  \"styles\": {\"holes\": " << style_counts.holes
          << ", \"self_intersections\": " << style_counts.self_intersections
          << ", \"coincident_edges\": " << style_counts.coincident_edges
          << ", \"tiny_angles\": " << style_counts.tiny_angles << "},\n"
          << "  \"counterexamples\": " << failed << "\n"
          << "}\n";

  try
  {
    WriteTextFile(output_dir / "outputs.jsonl", outputs_jsonl.str());
    WriteTextFile(output_dir / "counterexamples.json",
      counterexamples_json.str());
    WriteTextFile(output_dir / "summary.json", summary.str());
  }
  catch (const std::exception& ex)
  {
    std::cerr << "artifact error: " << ex.what() << "\n";
    return 2;
  }

  std::cout << "seed=" << seed
            << " cases=" << case_count
            << " passed=" << passed
            << " failed=" << failed
            << " nondeterministic=" << nondeterministic
            << " counterexamples=" << failed << "\n";
  std::cout << "styles: holes=" << style_counts.holes
            << " self_intersections=" << style_counts.self_intersections
            << " coincident_edges=" << style_counts.coincident_edges
            << " tiny_angles=" << style_counts.tiny_angles << "\n";
  std::cout << "artifacts=" << output_dir.string() << "\n";
  return failed == 0 ? 0 : 1;
}
