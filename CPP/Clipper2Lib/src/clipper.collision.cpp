/*******************************************************************************
* Purpose   :  Collision-contour simplification with edge provenance          *
* License   :  https://www.boost.org/LICENSE_1_0.txt                           *
*******************************************************************************/

#include "clipper2/clipper.collision.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>

namespace Clipper2Lib
{
namespace
{

#if defined(__SIZEOF_INT128__)
__extension__ typedef __int128 WideInt;
#else
typedef long double WideInt;
#endif

constexpr long double kSourceMatchRadius = 2.0L;

struct LineKey
{
  WideInt dx = 0;
  WideInt dy = 0;
  WideInt c = 0;

  bool operator==(const LineKey& other) const
  {
    return dx == other.dx && dy == other.dy && c == other.c;
  }
};

inline uint64_t MixHash(uint64_t hash, uint64_t value)
{
  hash ^= value + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
  return hash;
}

#if defined(__SIZEOF_INT128__)
inline void SplitWideInt(WideInt value, uint64_t& high, uint64_t& low)
{
  high = static_cast<uint64_t>(value >> 64);
  low = static_cast<uint64_t>(value);
}
#else
inline void SplitWideInt(WideInt value, uint64_t& high, uint64_t& low)
{
  high = static_cast<uint64_t>(std::hash<long double>{}(value));
  low = static_cast<uint64_t>(std::hash<long double>{}(value * 0.6180339887498948482L));
}
#endif

struct LineKeyHash
{
  size_t operator()(const LineKey& key) const
  {
    uint64_t high = 0, low = 0;
    size_t hash = 0;
    SplitWideInt(key.dx, high, low);
    hash = static_cast<size_t>(MixHash(hash, high));
    hash = static_cast<size_t>(MixHash(hash, low));
    SplitWideInt(key.dy, high, low);
    hash = static_cast<size_t>(MixHash(hash, high));
    hash = static_cast<size_t>(MixHash(hash, low));
    SplitWideInt(key.c, high, low);
    hash = static_cast<size_t>(MixHash(hash, high));
    return static_cast<size_t>(MixHash(hash, low));
  }
};

inline WideInt AbsWide(WideInt value)
{
  return value < 0 ? -value : value;
}

#if defined(__SIZEOF_INT128__)
WideInt GcdWide(WideInt a, WideInt b)
{
  a = AbsWide(a);
  b = AbsWide(b);
  while (b != 0)
  {
    const WideInt remainder = a % b;
    a = b;
    b = remainder;
  }
  return a;
}
#endif

bool MakeLineKey(const Point64& a, const Point64& b, LineKey& key)
{
  if (a == b) return false;

#if defined(__SIZEOF_INT128__)
  WideInt dx = static_cast<WideInt>(b.x) - static_cast<WideInt>(a.x);
  WideInt dy = static_cast<WideInt>(b.y) - static_cast<WideInt>(a.y);
  const WideInt gcd = GcdWide(dx, dy);
  if (gcd == 0) return false;
  dx /= gcd;
  dy /= gcd;
#else
  const long double dx_raw =
    static_cast<long double>(b.x) - static_cast<long double>(a.x);
  const long double dy_raw =
    static_cast<long double>(b.y) - static_cast<long double>(a.y);
  const long double length = std::sqrt(dx_raw * dx_raw + dy_raw * dy_raw);
  if (length == 0) return false;
  WideInt dx = dx_raw / length;
  WideInt dy = dy_raw / length;
#endif

  if (dx < 0 || (dx == 0 && dy < 0))
  {
    dx = -dx;
    dy = -dy;
  }

  key.dx = dx;
  key.dy = dy;
  key.c = dx * static_cast<WideInt>(a.y) -
    dy * static_cast<WideInt>(a.x);
  return true;
}

inline WideInt ProjectOnLine(const LineKey& key, const Point64& pt)
{
  return key.dx * static_cast<WideInt>(pt.x) +
    key.dy * static_cast<WideInt>(pt.y);
}

struct IndexedSourceEdge
{
  int64_t source_id = 0;
  CollisionSourceRole role = CollisionSourceRole::Subject;
  size_t source_path = 0;
  size_t edge_index = 0;
  size_t edge_count = 0;
  Point64 a;
  Point64 b;
  WideInt min_t = 0;
  WideInt max_t = 0;
  bool forward = true;
};

struct GridKey
{
  int64_t x = 0;
  int64_t y = 0;

  bool operator==(const GridKey& other) const
  {
    return x == other.x && y == other.y;
  }
};

struct GridKeyHash
{
  size_t operator()(const GridKey& key) const
  {
    size_t hash = std::hash<int64_t>{}(key.x);
    return static_cast<size_t>(MixHash(
      static_cast<uint64_t>(hash),
      static_cast<uint64_t>(std::hash<int64_t>{}(key.y))));
  }
};

struct SourceGrid
{
  std::unordered_map<GridKey, std::vector<size_t>, GridKeyHash> cells;
  std::vector<size_t> long_edges;
  long double min_x = 0.0L;
  long double min_y = 0.0L;
  long double cell_size = 1.0L;
  bool built = false;
};

struct SourceRef
{
  size_t edge = 0;
  bool reversed = false;
};

struct SourceIndex
{
  std::vector<IndexedSourceEdge> edges;
  std::unordered_map<LineKey, std::vector<size_t>, LineKeyHash> by_line;
  SourceGrid grid;
  bool requested = false;
};

struct ContourMeta
{
  Path64 path;
  bool is_hole = false;
};

void CollectTree(const PolyPath64& parent,
  std::vector<ContourMeta>& contours)
{
  for (const auto& child : parent)
  {
    contours.push_back(ContourMeta{ child->Polygon(), child->IsHole() });
    CollectTree(*child, contours);
  }
}

bool NormalizeInput(const Paths64& input,
  std::vector<ContourMeta>& contours)
{
  contours.clear();
  if (input.empty()) return true;

  PolyTree64 tree;
  Clipper64 clipper;
  clipper.PreserveCollinear(false);
  clipper.AddSubject(input);
  if (!clipper.Execute(ClipType::Union, FillRule::NonZero, tree))
    return false;

  CollectTree(tree, contours);
  return true;
}

size_t EdgeCount(const Paths64& paths)
{
  size_t result = 0;
  for (const Path64& path : paths)
    result += path.size();
  return result;
}

void AddSourceGroup(const std::vector<CollisionSourceInput>& groups,
  CollisionSourceRole role, SourceIndex& index)
{
  for (const CollisionSourceInput& group : groups)
  {
    for (size_t path_index = 0; path_index < group.paths.size(); ++path_index)
    {
      const Path64& path = group.paths[path_index];
      if (path.size() < 2) continue;

      for (size_t i = 0; i < path.size(); ++i)
      {
        const Point64& a = path[i];
        const Point64& b = path[(i + 1) % path.size()];
        LineKey key;
        if (!MakeLineKey(a, b, key)) continue;

        IndexedSourceEdge edge;
        edge.source_id = group.source_id;
        edge.role = role;
        edge.source_path = path_index;
        edge.edge_index = i;
        edge.edge_count = path.size();
        edge.a = a;
        edge.b = b;
        const WideInt ta = ProjectOnLine(key, a);
        const WideInt tb = ProjectOnLine(key, b);
        edge.min_t = (std::min)(ta, tb);
        edge.max_t = (std::max)(ta, tb);
        edge.forward = ta < tb;

        index.by_line[key].push_back(index.edges.size());
        index.edges.push_back(edge);
      }
    }
  }
}

void BuildSourceIndex(const std::vector<CollisionSourceInput>& subjects,
  const std::vector<CollisionSourceInput>& clips, SourceIndex& index)
{
  size_t source_edge_count = 0;
  for (const CollisionSourceInput& group : subjects)
    source_edge_count += EdgeCount(group.paths);
  for (const CollisionSourceInput& group : clips)
    source_edge_count += EdgeCount(group.paths);

  index.edges.reserve(source_edge_count);
  index.by_line.reserve(source_edge_count * 2 + 1);
  index.requested = !subjects.empty() || !clips.empty();

  AddSourceGroup(subjects, CollisionSourceRole::Subject, index);
  AddSourceGroup(clips, CollisionSourceRole::Clip, index);
}

long double PointLineDistSq(const Point64& pt,
  const Point64& a, const Point64& b)
{
  const long double dx =
    static_cast<long double>(b.x) - static_cast<long double>(a.x);
  const long double dy =
    static_cast<long double>(b.y) - static_cast<long double>(a.y);
  const long double denom = dx * dx + dy * dy;
  if (denom == 0) return 0;
  const long double cross =
    dx * (static_cast<long double>(pt.y) - static_cast<long double>(a.y)) -
    dy * (static_cast<long double>(pt.x) - static_cast<long double>(a.x));
  return cross * cross / denom;
}

double PointSegmentDistSq(const Point64& pt,
  const Point64& a, const Point64& b)
{
  const long double dx =
    static_cast<long double>(b.x) - static_cast<long double>(a.x);
  const long double dy =
    static_cast<long double>(b.y) - static_cast<long double>(a.y);
  const long double px =
    static_cast<long double>(pt.x) - static_cast<long double>(a.x);
  const long double py =
    static_cast<long double>(pt.y) - static_cast<long double>(a.y);
  const long double denom = dx * dx + dy * dy;
  if (denom == 0) return static_cast<double>(px * px + py * py);

  long double t = (px * dx + py * dy) / denom;
  if (t < 0) t = 0;
  else if (t > 1) t = 1;
  const long double ex = px - t * dx;
  const long double ey = py - t * dy;
  return static_cast<double>(ex * ex + ey * ey);
}

void BuildSourceGrid(SourceIndex& index)
{
  if (index.grid.built) return;
  index.grid.built = true;
  if (index.edges.empty()) return;

  long double min_x = static_cast<long double>(index.edges[0].a.x);
  long double max_x = min_x;
  long double min_y = static_cast<long double>(index.edges[0].a.y);
  long double max_y = min_y;
  for (const IndexedSourceEdge& edge : index.edges)
  {
    min_x = (std::min)(min_x, static_cast<long double>(edge.a.x));
    min_x = (std::min)(min_x, static_cast<long double>(edge.b.x));
    max_x = (std::max)(max_x, static_cast<long double>(edge.a.x));
    max_x = (std::max)(max_x, static_cast<long double>(edge.b.x));
    min_y = (std::min)(min_y, static_cast<long double>(edge.a.y));
    min_y = (std::min)(min_y, static_cast<long double>(edge.b.y));
    max_y = (std::max)(max_y, static_cast<long double>(edge.a.y));
    max_y = (std::max)(max_y, static_cast<long double>(edge.b.y));
  }

  const long double extent = (std::max)(max_x - min_x, max_y - min_y);
  index.grid.min_x = min_x;
  index.grid.min_y = min_y;
  index.grid.cell_size = (std::max)(1.0L,
    extent / std::sqrt(static_cast<long double>(index.edges.size())));
  index.grid.cells.reserve(index.edges.size() * 2 + 1);

  for (size_t i = 0; i < index.edges.size(); ++i)
  {
    const IndexedSourceEdge& edge = index.edges[i];
    const int64_t x0 = static_cast<int64_t>(std::floor(
      (static_cast<long double>(edge.a.x) - min_x) /
        index.grid.cell_size));
    const int64_t x1 = static_cast<int64_t>(std::floor(
      (static_cast<long double>(edge.b.x) - min_x) /
        index.grid.cell_size));
    const int64_t y0 = static_cast<int64_t>(std::floor(
      (static_cast<long double>(edge.a.y) - min_y) /
        index.grid.cell_size));
    const int64_t y1 = static_cast<int64_t>(std::floor(
      (static_cast<long double>(edge.b.y) - min_y) /
        index.grid.cell_size));
    const long double cell_count =
      (static_cast<long double>(std::abs(x1 - x0)) + 1.0L) *
      (static_cast<long double>(std::abs(y1 - y0)) + 1.0L);
    if (cell_count > 16.0L)
    {
      index.grid.long_edges.push_back(i);
      continue;
    }

    for (int64_t x = (std::min)(x0, x1); x <= (std::max)(x0, x1); ++x)
      for (int64_t y = (std::min)(y0, y1); y <= (std::max)(y0, y1); ++y)
        index.grid.cells[GridKey{ x, y }].push_back(i);
  }
}

void GatherGridCandidates(const Point64& a, const Point64& b,
  SourceIndex& index, std::vector<size_t>& candidates)
{
  constexpr long double match_radius = kSourceMatchRadius;
  BuildSourceGrid(index);
  candidates.clear();
  if (index.edges.empty()) return;
  if (!index.grid.built || index.grid.cell_size <= 0.0L)
  {
    candidates.resize(index.edges.size());
    for (size_t i = 0; i < candidates.size(); ++i) candidates[i] = i;
    return;
  }

  const long double min_x =
    static_cast<long double>((std::min)(a.x, b.x)) - match_radius;
  const long double max_x =
    static_cast<long double>((std::max)(a.x, b.x)) + match_radius;
  const long double min_y =
    static_cast<long double>((std::min)(a.y, b.y)) - match_radius;
  const long double max_y =
    static_cast<long double>((std::max)(a.y, b.y)) + match_radius;

  const long double fx0 = std::floor(
    (min_x - index.grid.min_x) / index.grid.cell_size);
  const long double fx1 = std::floor(
    (max_x - index.grid.min_x) / index.grid.cell_size);
  const long double fy0 = std::floor(
    (min_y - index.grid.min_y) / index.grid.cell_size);
  const long double fy1 = std::floor(
    (max_y - index.grid.min_y) / index.grid.cell_size);
  const long double cell_count = (fx1 - fx0 + 1.0L) * (fy1 - fy0 + 1.0L);

  if (cell_count > 256.0L)
  {
    candidates.resize(index.edges.size());
    for (size_t i = 0; i < candidates.size(); ++i) candidates[i] = i;
    return;
  }

  const int64_t x0 = static_cast<int64_t>(fx0);
  const int64_t x1 = static_cast<int64_t>(fx1);
  const int64_t y0 = static_cast<int64_t>(fy0);
  const int64_t y1 = static_cast<int64_t>(fy1);
  for (int64_t x = x0; x <= x1; ++x)
  {
    for (int64_t y = y0; y <= y1; ++y)
    {
      const auto found = index.grid.cells.find(GridKey{ x, y });
      if (found != index.grid.cells.end())
        candidates.insert(candidates.end(),
          found->second.begin(), found->second.end());
    }
  }
  candidates.insert(candidates.end(),
    index.grid.long_edges.begin(), index.grid.long_edges.end());

  std::sort(candidates.begin(), candidates.end());
  candidates.erase(
    std::unique(candidates.begin(), candidates.end()), candidates.end());
}

std::vector<SourceRef> FindApproxSourceRefs(const Point64& a,
  const Point64& b, SourceIndex& index)
{
  std::vector<size_t> candidates;
  GatherGridCandidates(a, b, index, candidates);
  if (candidates.empty()) return {};

  constexpr long double match_radius = kSourceMatchRadius;
  const long double rx = static_cast<long double>(b.x) -
    static_cast<long double>(a.x);
  const long double ry = static_cast<long double>(b.y) -
    static_cast<long double>(a.y);
  const long double result_len_sq = rx * rx + ry * ry;
  if (result_len_sq == 0) return {};

  struct ApproxMatch
  {
    size_t edge;
    long double distance;
    long double projection;
    bool reversed;
  };

  std::vector<ApproxMatch> matches;
  long double best_distance = std::numeric_limits<long double>::max();
  for (size_t edge_index : candidates)
  {
    const IndexedSourceEdge& edge = index.edges[edge_index];
    const long double sx = static_cast<long double>(edge.b.x) -
      static_cast<long double>(edge.a.x);
    const long double sy = static_cast<long double>(edge.b.y) -
      static_cast<long double>(edge.a.y);
    const long double source_len_sq = sx * sx + sy * sy;
    if (source_len_sq == 0) continue;

    const long double dot = rx * sx + ry * sy;
    const long double direction_cos =
      std::abs(dot) / std::sqrt(result_len_sq * source_len_sq);
    if (direction_cos < 0.99L) continue;

    const long double pa_x = static_cast<long double>(a.x) -
      static_cast<long double>(edge.a.x);
    const long double pa_y = static_cast<long double>(a.y) -
      static_cast<long double>(edge.a.y);
    const long double pb_x = static_cast<long double>(b.x) -
      static_cast<long double>(edge.a.x);
    const long double pb_y = static_cast<long double>(b.y) -
      static_cast<long double>(edge.a.y);
    const long double ta = pa_x * sx + pa_y * sy;
    const long double tb = pb_x * sx + pb_y * sy;
    const long double overlap =
      (std::min)((std::max)(ta, tb), source_len_sq) -
      (std::max)((std::min)(ta, tb), 0.0L);
    if (overlap <= 0.0L) continue;

    const long double distance = (std::max)(
      PointLineDistSq(a, edge.a, edge.b),
      PointLineDistSq(b, edge.a, edge.b));
    if (distance > match_radius * match_radius) continue;

    const long double mid_x =
      (static_cast<long double>(edge.a.x) +
        static_cast<long double>(edge.b.x)) * 0.5L -
      static_cast<long double>(a.x);
    const long double mid_y =
      (static_cast<long double>(edge.a.y) +
        static_cast<long double>(edge.b.y)) * 0.5L -
      static_cast<long double>(a.y);
    const long double projection = mid_x * rx + mid_y * ry;
    matches.push_back(ApproxMatch{
      edge_index, distance, projection, dot < 0 });
    best_distance = (std::min)(best_distance, distance);
  }

  std::sort(matches.begin(), matches.end(),
    [&index](const ApproxMatch& lhs, const ApproxMatch& rhs)
    {
      if (lhs.projection != rhs.projection)
        return lhs.projection < rhs.projection;
      const IndexedSourceEdge& le = index.edges[lhs.edge];
      const IndexedSourceEdge& re = index.edges[rhs.edge];
      if (le.source_id != re.source_id) return le.source_id < re.source_id;
      if (le.role != re.role) return le.role < re.role;
      if (le.source_path != re.source_path)
        return le.source_path < re.source_path;
      return le.edge_index < re.edge_index;
    });

  std::vector<SourceRef> result;
  for (const ApproxMatch& match : matches)
  {
    if (match.distance > best_distance + 1e-12L) continue;
    result.push_back(SourceRef{ match.edge, match.reversed });
  }
  return result;
}

std::vector<SourceRef> FindSourceRefs(const Point64& a, const Point64& b,
  const SourceIndex& index)
{
  std::vector<SourceRef> result;
  LineKey key;
  if (!MakeLineKey(a, b, key)) return result;

  const auto bucket = index.by_line.find(key);
  if (bucket == index.by_line.end()) return result;

  const WideInt ta = ProjectOnLine(key, a);
  const WideInt tb = ProjectOnLine(key, b);
  const WideInt query_min = (std::min)(ta, tb);
  const WideInt query_max = (std::max)(ta, tb);
  const bool query_forward = ta < tb;

  result.reserve(bucket->second.size());
  for (size_t edge_index : bucket->second)
  {
    const IndexedSourceEdge& edge = index.edges[edge_index];
    if ((std::max)(query_min, edge.min_t) >=
      (std::min)(query_max, edge.max_t))
      continue;

    result.push_back(SourceRef{
      edge_index, query_forward != edge.forward });
  }

  std::sort(result.begin(), result.end(),
    [&index](const SourceRef& lhs, const SourceRef& rhs)
    {
      const IndexedSourceEdge& le = index.edges[lhs.edge];
      const IndexedSourceEdge& re = index.edges[rhs.edge];
      if (le.min_t != re.min_t) return le.min_t < re.min_t;
      if (le.source_id != re.source_id) return le.source_id < re.source_id;
      if (le.role != re.role) return le.role < re.role;
      if (le.source_path != re.source_path)
        return le.source_path < re.source_path;
      return le.edge_index < re.edge_index;
    });

  if (!query_forward) std::reverse(result.begin(), result.end());
  return result;
}

std::vector<SourceRef> FindNearSourceRefs(const Point64& a,
  const Point64& b, SourceIndex& index)
{
  const long double dx = static_cast<long double>(b.x) -
    static_cast<long double>(a.x);
  const long double dy = static_cast<long double>(b.y) -
    static_cast<long double>(a.y);
  const long double micro_edge_limit = 4.0L * kSourceMatchRadius;
  if (dx * dx + dy * dy > micro_edge_limit * micro_edge_limit)
    return {};

  std::vector<size_t> candidates;
  GatherGridCandidates(a, b, index, candidates);
  if (candidates.empty()) return {};

  constexpr double match_radius = static_cast<double>(kSourceMatchRadius);
  std::vector<SourceRef> result;
  double best_distance = std::numeric_limits<double>::max();
  for (size_t edge_index : candidates)
  {
    const IndexedSourceEdge& edge = index.edges[edge_index];
    const double distance = (std::min)(
      PointSegmentDistSq(a, edge.a, edge.b),
      PointSegmentDistSq(b, edge.a, edge.b));
    if (distance > match_radius * match_radius) continue;
    if (distance < best_distance)
    {
      best_distance = distance;
      result.clear();
    }
    if (distance <= best_distance) result.push_back(SourceRef{ edge_index, false });
  }
  std::sort(result.begin(), result.end(),
    [&index](const SourceRef& lhs, const SourceRef& rhs)
    {
      const IndexedSourceEdge& le = index.edges[lhs.edge];
      const IndexedSourceEdge& re = index.edges[rhs.edge];
      if (le.source_id != re.source_id) return le.source_id < re.source_id;
      if (le.role != re.role) return le.role < re.role;
      if (le.source_path != re.source_path)
        return le.source_path < re.source_path;
      return le.edge_index < re.edge_index;
    });
  return result;
}

std::vector<std::vector<std::vector<SourceRef>>> MapInputEdges(
  const std::vector<ContourMeta>& contours, SourceIndex& index,
  size_t& unmapped_edges)
{
  std::vector<std::vector<std::vector<SourceRef>>> result;
  unmapped_edges = 0;
  if (!index.requested) return result;

  result.resize(contours.size());
  for (size_t c = 0; c < contours.size(); ++c)
  {
    const Path64& path = contours[c].path;
    result[c].resize(path.size());
    if (path.size() < 2) continue;

    for (size_t i = 0; i < path.size(); ++i)
    {
      result[c][i] = FindSourceRefs(
        path[i], path[(i + 1) % path.size()], index);
      if (result[c][i].empty())
        result[c][i] = FindApproxSourceRefs(
          path[i], path[(i + 1) % path.size()], index);
      if (result[c][i].empty())
        result[c][i] = FindNearSourceRefs(
          path[i], path[(i + 1) % path.size()], index);
      if (result[c][i].empty()) ++unmapped_edges;
    }
  }
  return result;
}

struct RdpFrame
{
  size_t begin;
  size_t end;
};

void RdpChain(const Path64& path, size_t begin, size_t end,
  double eps_sqrd, std::vector<uint8_t>& keep)
{
  const size_t count = path.size();
  std::vector<RdpFrame> stack;
  stack.push_back(RdpFrame{ begin, end });

  while (!stack.empty())
  {
    const RdpFrame frame = stack.back();
    stack.pop_back();

    const size_t span = (frame.end + count - frame.begin) % count;
    if (span <= 1) continue;

    double max_dist = -1.0;
    size_t max_index = frame.begin;
    for (size_t i = (frame.begin + 1) % count; i != frame.end;
      i = (i + 1) % count)
    {
      const double dist = PointSegmentDistSq(path[i],
        path[frame.begin], path[frame.end]);
      if (dist > max_dist)
      {
        max_dist = dist;
        max_index = i;
      }
    }

    if (max_dist > eps_sqrd)
    {
      keep[max_index] = 1;
      stack.push_back(RdpFrame{ frame.begin, max_index });
      stack.push_back(RdpFrame{ max_index, frame.end });
    }
  }
}

std::vector<size_t> SimplifiedIndices(const Path64& path, double tolerance)
{
  std::vector<size_t> result;
  const size_t count = path.size();
  if (count <= 3)
  {
    result.resize(count);
    for (size_t i = 0; i < count; ++i) result[i] = i;
    return result;
  }

  size_t opposite = 1;
  double max_dist = -1.0;
  for (size_t i = 1; i < count; ++i)
  {
    const long double dx =
      static_cast<long double>(path[i].x) - static_cast<long double>(path[0].x);
    const long double dy =
      static_cast<long double>(path[i].y) - static_cast<long double>(path[0].y);
    const double dist = static_cast<double>(dx * dx + dy * dy);
    if (dist > max_dist)
    {
      max_dist = dist;
      opposite = i;
    }
  }

  std::vector<uint8_t> keep(count, 0);
  keep[0] = 1;
  keep[opposite] = 1;

  const double eps_sqrd = tolerance * tolerance;
  RdpChain(path, 0, opposite, eps_sqrd, keep);
  RdpChain(path, opposite, 0, eps_sqrd, keep);

  size_t kept = 0;
  for (uint8_t value : keep) kept += value != 0;
  if (kept < 3)
  {
    double best = -1.0;
    size_t best_index = 0;
    for (size_t i = 0; i < count; ++i)
    {
      if (keep[i]) continue;
      const double dist = PointSegmentDistSq(path[i], path[0], path[opposite]);
      if (dist > best)
      {
        best = dist;
        best_index = i;
      }
    }
    if (best > 0) keep[best_index] = 1;
  }

  for (size_t i = 0; i < count; ++i)
    if (keep[i]) result.push_back(i);
  if (result.size() < 3)
  {
    result.resize(count);
    for (size_t i = 0; i < count; ++i) result[i] = i;
  }
  return result;
}

bool SameSignArea(const Path64& a, const Path64& b)
{
  const double area_a = Area(a);
  const double area_b = Area(b);
  if (area_a == 0.0 || area_b == 0.0) return false;
  return (area_a > 0.0) == (area_b > 0.0);
}

struct AreaSignature
{
  bool is_hole;
  double area;
};

bool AreaSignatureLess(const AreaSignature& lhs, const AreaSignature& rhs)
{
  if (lhs.is_hole != rhs.is_hole) return lhs.is_hole < rhs.is_hole;
  return lhs.area < rhs.area;
}

bool SameArea(const AreaSignature& lhs, const AreaSignature& rhs)
{
  const double tolerance = (std::max)(
    (std::max)(std::abs(lhs.area), std::abs(rhs.area)) * 1e-9, 1e-9);
  return std::abs(lhs.area - rhs.area) <= tolerance;
}

bool HasSimpleRegionTopology(const Paths64& candidate,
  const std::vector<ContourMeta>& contours)
{
  PolyTree64 tree;
  Clipper64 clipper;
  clipper.PreserveCollinear(false);
  clipper.AddSubject(candidate);
  if (!clipper.Execute(ClipType::Union, FillRule::NonZero, tree))
    return false;

  std::vector<ContourMeta> repaired_contours;
  CollectTree(tree, repaired_contours);
  if (repaired_contours.size() != contours.size()) return false;

  std::vector<AreaSignature> expected;
  expected.reserve(contours.size());
  for (size_t i = 0; i < contours.size(); ++i)
    expected.push_back(AreaSignature{
      contours[i].is_hole, std::abs(Area(candidate[i])) });

  std::vector<AreaSignature> actual;
  actual.reserve(repaired_contours.size());
  for (const ContourMeta& contour : repaired_contours)
    actual.push_back(AreaSignature{
      contour.is_hole, std::abs(Area(contour.path)) });

  std::sort(expected.begin(), expected.end(), AreaSignatureLess);
  std::sort(actual.begin(), actual.end(), AreaSignatureLess);
  for (size_t i = 0; i < expected.size(); ++i)
    if (!SameArea(expected[i], actual[i])) return false;
  return true;
}

bool ValidateCandidate(const std::vector<Path64>& candidate,
  const std::vector<ContourMeta>& contours,
  double area_before, const CollisionSimplifyOptions& options,
  bool check_topology, double& area_after)
{
  if (candidate.size() != contours.size()) return false;

  area_after = 0.0;
  for (size_t i = 0; i < candidate.size(); ++i)
  {
    const Path64& path = candidate[i];
    if (path.size() < 3) return false;
    if (!SameSignArea(path, contours[i].path)) return false;
    area_after += Area(path);
  }
  area_after = std::abs(area_after);

  const double area_error = std::abs(area_after - area_before);
  const double area_budget =
    options.max_relative_area_error * area_before +
    options.max_absolute_area_error;
  if (area_error > area_budget * (1.0 + 1e-12) + 1e-12)
    return false;

  if (check_topology &&
    !HasSimpleRegionTopology(candidate, contours))
    return false;
  return true;
}

void AppendSourceSpan(std::vector<CollisionSourceSpan>& spans,
  const IndexedSourceEdge& edge, bool reversed)
{
  if (!spans.empty())
  {
    CollisionSourceSpan& last = spans.back();
    if (last.source_id == edge.source_id &&
      last.role == edge.role &&
      last.source_path == edge.source_path &&
      last.reversed == reversed)
    {
      const size_t expected = reversed ?
        (last.last_edge + edge.edge_count - 1) % edge.edge_count :
        (last.last_edge + 1) % edge.edge_count;
      if (expected == edge.edge_index)
      {
        last.last_edge = edge.edge_index;
        return;
      }
    }
  }

  spans.push_back(CollisionSourceSpan{
    edge.source_id, edge.role, edge.source_path,
    edge.edge_index, edge.edge_index, reversed });
}

CollisionContourResult InvalidResult(CollisionContourStatus status)
{
  CollisionContourResult result;
  result.status = status;
  return result;
}

bool ValidOptions(const CollisionSimplifyOptions& options)
{
  return std::isfinite(options.tolerance) && options.tolerance >= 0.0 &&
    std::isfinite(options.max_relative_area_error) &&
    options.max_relative_area_error >= 0.0 &&
    std::isfinite(options.max_absolute_area_error) &&
    options.max_absolute_area_error >= 0.0;
}

} // namespace

CollisionContourResult BuildCollisionContours(
  ClipType clip_type,
  FillRule fill_rule,
  const std::vector<CollisionSourceInput>& subjects,
  const std::vector<CollisionSourceInput>& clips,
  const CollisionSimplifyOptions& options)
{
  if (!ValidOptions(options))
    return InvalidResult(CollisionContourStatus::InvalidOptions);

  Paths64 subject_paths;
  Paths64 clip_paths;
  for (const CollisionSourceInput& subject : subjects)
    subject_paths.insert(subject_paths.end(),
      subject.paths.begin(), subject.paths.end());
  for (const CollisionSourceInput& clip : clips)
    clip_paths.insert(clip_paths.end(),
      clip.paths.begin(), clip.paths.end());

  if (subject_paths.empty() && clip_paths.empty())
    return InvalidResult(CollisionContourStatus::Empty);

  Paths64 solution;
  Clipper64 clipper;
  clipper.PreserveCollinear(false);
  if (!subject_paths.empty()) clipper.AddSubject(subject_paths);
  if (!clip_paths.empty()) clipper.AddClip(clip_paths);
  if (!clipper.Execute(clip_type, fill_rule, solution))
    return InvalidResult(CollisionContourStatus::InvalidInput);

  return SimplifyCollisionContours(solution, subjects, clips, options);
}

CollisionContourResult SimplifyCollisionContours(
  const Paths64& boolean_result,
  const std::vector<CollisionSourceInput>& subjects,
  const std::vector<CollisionSourceInput>& clips,
  const CollisionSimplifyOptions& options)
{
  if (!ValidOptions(options))
    return InvalidResult(CollisionContourStatus::InvalidOptions);
  if (boolean_result.empty())
    return InvalidResult(CollisionContourStatus::Empty);

  size_t raw_input_vertices = 0;
  for (const Path64& path : boolean_result)
    raw_input_vertices += path.size();

  std::vector<ContourMeta> contours;
  if (!NormalizeInput(boolean_result, contours))
    return InvalidResult(CollisionContourStatus::InvalidInput);
  if (contours.empty())
    return InvalidResult(CollisionContourStatus::Empty);

  double area_before = 0.0;
  for (const ContourMeta& contour : contours)
  {
    if (contour.path.size() < 3 || Area(contour.path) == 0.0)
      return InvalidResult(CollisionContourStatus::InvalidInput);
    area_before += Area(contour.path);
  }
  area_before = std::abs(area_before);
  if (area_before == 0.0)
    return InvalidResult(CollisionContourStatus::InvalidInput);

  SourceIndex source_index;
  BuildSourceIndex(subjects, clips, source_index);
  size_t unmapped_edges = 0;
  const std::vector<std::vector<std::vector<SourceRef>>> input_sources =
    MapInputEdges(contours, source_index, unmapped_edges);

  const size_t input_vertices = [&contours]()
  {
    size_t count = 0;
    for (const ContourMeta& contour : contours) count += contour.path.size();
    return count;
  }();

  double tolerance = options.tolerance;
  bool accepted = false;
  bool topology_ok = false;
  double area_after = area_before;
  std::vector<std::vector<size_t>> best_indices;
  std::vector<Path64> candidate;

  for (int attempt = 0; attempt < 32; ++attempt)
  {
    candidate.clear();
    candidate.reserve(contours.size());
    best_indices.clear();
    best_indices.reserve(contours.size());

    bool modified = false;
    for (const ContourMeta& contour : contours)
    {
      best_indices.push_back(SimplifiedIndices(contour.path, tolerance));
      if (best_indices.back().size() != contour.path.size())
        modified = true;

      Path64 simplified;
      simplified.reserve(best_indices.back().size());
      for (size_t index : best_indices.back())
        simplified.push_back(contour.path[index]);
      candidate.push_back(std::move(simplified));
    }

    if (ValidateCandidate(candidate, contours, area_before,
      options, modified, area_after))
    {
      accepted = true;
      topology_ok = true;
      break;
    }

    if (tolerance == 0.0) break;
    tolerance *= 0.5;
    if (tolerance < 1e-9) tolerance = 0.0;
  }

  if (!accepted)
  {
    candidate.clear();
    best_indices.clear();
    for (const ContourMeta& contour : contours)
    {
      candidate.push_back(contour.path);
      best_indices.emplace_back(contour.path.size());
      for (size_t i = 0; i < contour.path.size(); ++i)
        best_indices.back()[i] = i;
    }

    area_after = area_before;
    const double area_budget =
      options.max_relative_area_error * area_before +
      options.max_absolute_area_error;
    const bool area_ok = std::abs(area_after - area_before) <=
      area_budget * (1.0 + 1e-12) + 1e-12;
    topology_ok = HasSimpleRegionTopology(candidate, contours);
    if (!area_ok)
      return InvalidResult(CollisionContourStatus::AreaLimitExceeded);
    if (!topology_ok)
      return InvalidResult(CollisionContourStatus::TopologyChanged);
    accepted = true;
  }

  CollisionContourResult result;
  result.status = CollisionContourStatus::Ok;
  result.area_before = area_before;
  result.area_after = area_after;
  result.absolute_area_error = std::abs(area_after - area_before);
  result.relative_area_error = area_before == 0.0 ?
    (result.absolute_area_error == 0.0 ? 0.0 :
      std::numeric_limits<double>::infinity()) :
    result.absolute_area_error / area_before;
  result.effective_tolerance = tolerance;
  result.raw_input_vertex_count = raw_input_vertices;
  result.input_vertex_count = input_vertices;
  result.output_vertex_count = 0;
  result.unmapped_edge_count = unmapped_edges;
  result.area_limit_met = true;
  result.topology_preserved = topology_ok;
  result.tolerance_reduced = tolerance < options.tolerance;

  result.contours.reserve(candidate.size());
  for (size_t c = 0; c < candidate.size(); ++c)
  {
    CollisionContour contour;
    contour.path = std::move(candidate[c]);
    contour.is_hole = contours[c].is_hole;
    result.output_vertex_count += contour.path.size();

    if (source_index.requested)
    {
      contour.edge_sources.reserve(contour.path.size());
      for (size_t edge = 0; edge < contour.path.size(); ++edge)
      {
        const size_t begin = best_indices[c][edge];
        const size_t end =
          best_indices[c][(edge + 1) % best_indices[c].size()];
        std::vector<CollisionSourceSpan> spans;
        for (size_t input_edge = begin; input_edge != end;
          input_edge = (input_edge + 1) % contours[c].path.size())
        {
          if (c >= input_sources.size() ||
            input_edge >= input_sources[c].size())
            break;
          for (const SourceRef& ref : input_sources[c][input_edge])
            AppendSourceSpan(
              spans, source_index.edges[ref.edge], ref.reversed);
        }
        contour.edge_sources.push_back(std::move(spans));
      }
    }
    result.contours.push_back(std::move(contour));
  }

  return result;
}

} // namespace Clipper2Lib
