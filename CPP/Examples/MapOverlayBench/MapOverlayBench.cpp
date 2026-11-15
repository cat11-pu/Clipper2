// MapOverlayBench: benchmark for mass-polygon boolean operations
// (map overlay / collision pre-processing style workloads).
//
// Builds three tiers of polygon counts, times Union and Intersection,
// and can dump every output point for point-by-point verification
// between engine versions.
//
// Usage:
//   MapOverlayBench              -> prints timing table + checksums
//   MapOverlayBench --dump <dir> -> also writes one file per op/tier
//                                   containing every output point.

#include "clipper2/clipper.h"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <random>
#include <string>
#include <sys/stat.h>

using namespace Clipper2Lib;
using Clock = std::chrono::steady_clock;

// deterministic test-data generation (map-like: parcels + building outlines)
static Paths64 MakeLayer(size_t count, int64_t extent, uint64_t seed)
{
  std::mt19937_64 rng(seed);
  std::uniform_int_distribution<int64_t> pos(0, extent);
  std::uniform_int_distribution<int> kind(0, 9);
  std::uniform_int_distribution<int64_t> span(8, 60);
  std::uniform_int_distribution<int> verts(3, 9);
  std::uniform_real_distribution<double> ang(0.0, 6.283185307179586);
  std::uniform_real_distribution<double> rad(6.0, 45.0);

  Paths64 layer;
  layer.reserve(count);
  for (size_t i = 0; i < count; ++i)
  {
    const int64_t cx = pos(rng);
    const int64_t cy = pos(rng);
    if (kind(rng) < 3)
    {
      // axis-aligned rectangle (common in map data, exercises horizontals)
      const int64_t w = span(rng);
      const int64_t h = span(rng);
      Path64 r;
      r.reserve(4);
      r.push_back(Point64(cx, cy));
      r.push_back(Point64(cx + w, cy));
      r.push_back(Point64(cx + w, cy + h));
      r.push_back(Point64(cx, cy + h));
      layer.push_back(r);
    }
    else
    {
      // random convex-ish polygon
      const int n = verts(rng);
      const double a0 = ang(rng);
      Path64 p;
      p.reserve(n);
      for (int k = 0; k < n; ++k)
      {
        const double a = a0 + 6.283185307179586 * k / n;
        const double r = rad(rng);
        p.push_back(Point64(
          cx + static_cast<int64_t>(r * cos(a)),
          cy + static_cast<int64_t>(r * sin(a))));
      }
      layer.push_back(p);
    }
  }
  return layer;
}

static uint64_t fnv1a(uint64_t h, int64_t v)
{
  const uint64_t u = static_cast<uint64_t>(v);
  for (int i = 0; i < 8; ++i)
  {
    h ^= (u >> (i * 8)) & 0xff;
    h *= 1099511628211ull;
  }
  return h;
}

static uint64_t Checksum(const Paths64& paths, size_t& point_count)
{
  uint64_t h = 1469598103934665603ull;
  point_count = 0;
  for (const Path64& p : paths)
  {
    h = fnv1a(h, static_cast<int64_t>(p.size()));
    for (const Point64& pt : p)
    {
      h = fnv1a(h, pt.x);
      h = fnv1a(h, pt.y);
    }
    point_count += p.size();
  }
  return h;
}

static void Dump(const std::string& filename, const Paths64& paths)
{
  std::ofstream out(filename);
  out << "paths " << paths.size() << "\n";
  for (size_t i = 0; i < paths.size(); ++i)
  {
    out << "path " << i << " count " << paths[i].size() << "\n";
    for (const Point64& pt : paths[i])
      out << pt.x << " " << pt.y << "\n";
  }
}

struct Tier {
  size_t polygons_per_layer;
  int repeats;
};

int main(int argc, char** argv)
{
  std::string dump_dir;
  for (int i = 1; i < argc; ++i)
  {
    if (std::string(argv[i]) == "--dump" && i + 1 < argc)
      dump_dir = argv[++i];
  }
  if (!dump_dir.empty()) mkdir(dump_dir.c_str(), 0755);

  const Tier tiers[] = {
    { 2000, 25 },
    { 10000, 11 },
    { 50000, 5 },
  };

  printf("%-8s %-13s %10s %12s %12s %s\n",
    "polys", "op", "min_ms", "out_paths", "out_points", "checksum");
  fflush(stdout);

  for (const Tier& tier : tiers)
  {
    const int64_t extent =
      static_cast<int64_t>(100.0 * std::sqrt(static_cast<double>(tier.polygons_per_layer)));
    Paths64 subject = MakeLayer(tier.polygons_per_layer, extent, 12345);
    Paths64 clip    = MakeLayer(tier.polygons_per_layer, extent, 67890);

    struct Op { const char* name; ClipType ct; };
    const Op ops[] = {
      { "union", ClipType::Union },
      { "intersection", ClipType::Intersection },
    };

    for (const Op& op : ops)
    {
      Paths64 solution;
      double best_ms = 1e30;
      for (int r = 0; r < tier.repeats; ++r)
      {
        const auto t0 = Clock::now();
        solution = BooleanOp(op.ct, FillRule::NonZero, subject, clip);
        const auto t1 = Clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (ms < best_ms) best_ms = ms;
      }
      size_t point_count = 0;
      const uint64_t sum = Checksum(solution, point_count);
      printf("%-8zu %-13s %10.2f %12zu %12zu %016zx\n",
        tier.polygons_per_layer, op.name, best_ms,
        solution.size(), point_count, sum);
      fflush(stdout);

      if (!dump_dir.empty())
      {
        Dump(dump_dir + "/" + op.name + "_" +
          std::to_string(tier.polygons_per_layer) + ".txt", solution);
      }
    }
  }
  return 0;
}
