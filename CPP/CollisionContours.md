# Collision Contours

`clipper.collision.h` provides a narrow, game-map oriented API for simplifying
Boolean polygon results while retaining winding, topology and edge provenance.

## API

```cpp
#include "clipper2/clipper.h"

using namespace Clipper2Lib;

std::vector<CollisionSourceInput> subjects = {
  CollisionSourceInput{ 7, subject_paths }
};
std::vector<CollisionSourceInput> clips = {
  CollisionSourceInput{ 9, clip_paths }
};

CollisionSimplifyOptions options;
options.tolerance = 2.0;                  // coordinate units
options.max_relative_area_error = 1e-4;   // default

CollisionContourResult result = BuildCollisionContours(
  ClipType::Difference, FillRule::NonZero,
  subjects, clips, options);
```

If the Boolean result already exists, call:

```cpp
CollisionContourResult result =
  SimplifyCollisionContours(boolean_result, subjects, clips, options);
```

`CollisionSourceInput::source_id` identifies the original map feature.
`source_path` and `first_edge`/`last_edge` identify its path and edge range.

## Result semantics

- `contours[i].path` is a closed ring with no repeated closing point.
- `contours[i].is_hole` comes from the normalized Boolean hierarchy.
- `raw_input_vertex_count` is the caller-supplied Boolean result size.
- `input_vertex_count` is the size after normalization, before simplification.
- `area_before` is measured after normalization and before simplification.
- `relative_area_error = abs(area_after - area_before) / area_before`.
- `effective_tolerance` is the largest tolerance that passed both checks.
- `tolerance_reduced` is true when `effective_tolerance < options.tolerance`.
- `edge_sources[i]` describes `path[i] -> path[(i + 1) % path.size()]`.
- A source span is a contiguous inclusive edge range. `reversed` means the
  source range is traversed backwards.
- `unmapped_edge_count` counts normalized input edges for which provenance was
  requested but no source was found.

The implementation does the following:

1. Runs a `NonZero` union on the input. This removes duplicate and coincident
   edges, repairs overlaps, and rebuilds the outer/hole hierarchy.
2. Simplifies each ring by retaining a subset of its normalized vertices.
   Winding direction is therefore preserved exactly.
3. Checks the relative area budget and verifies that a union of the simplified
   rings has the same contour count, hole classification and contour areas.
4. If the requested tolerance fails, halves it and retries. At worst it
   returns the normalized input unchanged.
5. Maps provenance using an exact normalized-line hash. Clipper rounds
   intersection coordinates to integers, so an edge within two coordinate
   units of a source edge falls back to a spatial-grid lookup. Tiny Boolean
   artifacts up to eight units map to their nearest incident source edge.

The result is best-effort with a hard area cap, not a promise that every
requested tolerance reduction will happen. Always inspect `effective_tolerance`
and `tolerance_reduced`.

## Boundary cases

| Case | Behavior |
| --- | --- |
| Empty input / empty Boolean result | `status == Empty` |
| Negative, NaN or infinite option | `status == InvalidOptions` |
| Duplicate identical rings | Union keeps one ring; no duplicate output edges |
| Two polygons sharing a full edge | Union merges them and emits the edge once |
| Outer ring with holes and islands | Hole flag and winding are preserved |
| Boolean intersection rounded to an integer point | Provenance uses the bounded-distance grid fallback |
| Tolerance removes more than the area budget | Effective tolerance is reduced |
| Simplification would merge, split or invert regions | Candidate is rejected; a smaller tolerance is tried |
| No source inputs | `Ok`; `edge_sources` stays empty and `unmapped_edge_count == 0` |
| Source inputs do not cover a result edge | That edge is counted in `unmapped_edge_count` |
| Zero-area or otherwise invalid Boolean output | `status == InvalidInput` |
| `tolerance == 0` | Only union normalization and exactly collinear cleanup remain |

## Consistency data

Measured with GCC 15.2, Release, Ubuntu 26.04.

| Case | Raw input | Normalized in | Output | Contours | Area before | Area after | Relative error | Effective tol. | Unmapped | Time |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Coincident rings | 8 | 4 | 4 | 1 | 10000 | 10000 | 0 | 1 | 0 | <1 ms |
| Rectangle with hole | 8 | 8 | 8 | 2 | 7500 | 7500 | 0 | 3 | 0 | <1 ms |
| Rounded intersection | 3 | 3 | 3 | 1 | 4800 | 4800 | 0 | 1 | 0 | <1 ms |
| Area fallback | 8 | 8 | 8 | 1 | 9900 | 9900 | 0 | 5 (requested 20) | 0 | <1 ms |
| Dense square | 100000 | 4 | 4 | 1 | 1e10 | 1e10 | 0 | 2 | 0 | 23 ms |
| Dense circle | 100000 | 75100 | 580 | 1 | 3.141592663376e12 | 3.141525549341e12 | 2.1363e-5 | 20 | 0 | 56 ms |
| Random Boolean mix | 1000 cases | - | - | - | - | - | max 9.8344e-5 | 3 | 0 | 33 ms total |

The complete run, including all scenarios above, had a Linux `ru_maxrss`
high-water mark of 45.6 MB. The 1000-case random run had zero failures and zero
unmapped edges.

Raw data: `Tests/CollisionContourConsistency.csv`.
