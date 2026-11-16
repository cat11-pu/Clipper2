/*******************************************************************************
* Purpose   :  Collision-contour simplification with edge provenance          *
* License   :  https://www.boost.org/LICENSE_1_0.txt                           *
*******************************************************************************/

#ifndef CLIPPER_COLLISION_H
#define CLIPPER_COLLISION_H

#include "clipper2/clipper.core.h"
#include "clipper2/clipper.engine.h"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace Clipper2Lib
{

enum class CollisionSourceRole : uint8_t { Subject, Clip };

// A user-defined source group. Multiple paths in one group share source_id;
// source_path distinguishes them in the result metadata.
struct CollisionSourceInput
{
  int64_t source_id = 0;
  Paths64 paths;
};

struct CollisionSimplifyOptions
{
  // Maximum geometric deviation in input coordinate units. The result may use
  // a smaller effective tolerance when needed to satisfy the area/topology
  // guarantees.
  double tolerance = 0.0;

  // Relative area budget: |area_after - area_before| / area_before.
  // The default is 1/10000.
  double max_relative_area_error = 1e-4;

  // Optional absolute floor for the area budget. This is useful when
  // area_before is zero or very small.
  double max_absolute_area_error = 0.0;
};

// A contiguous run of source-path edges used by one output edge.
// Edge indices are inclusive. When reversed is true, the run is traversed
// from first_edge down to last_edge (modulo the source path size).
struct CollisionSourceSpan
{
  int64_t source_id = 0;
  CollisionSourceRole role = CollisionSourceRole::Subject;
  size_t source_path = 0;
  size_t first_edge = 0;
  size_t last_edge = 0;
  bool reversed = false;
};

struct CollisionContour
{
  Path64 path;
  bool is_hole = false;

  // edge_sources[i] describes path[i] -> path[(i + 1) % path.size()].
  // Empty vectors mean provenance was not supplied or could not be mapped.
  std::vector<std::vector<CollisionSourceSpan>> edge_sources;
};

enum class CollisionContourStatus : uint8_t
{
  Ok,
  Empty,
  InvalidOptions,
  InvalidInput,
  AreaLimitExceeded,
  TopologyChanged
};

struct CollisionContourResult
{
  std::vector<CollisionContour> contours;
  CollisionContourStatus status = CollisionContourStatus::Empty;

  double area_before = 0.0;
  double area_after = 0.0;
  double absolute_area_error = 0.0;
  double relative_area_error = 0.0;
  double effective_tolerance = 0.0;

  size_t raw_input_vertex_count = 0;
  size_t input_vertex_count = 0; // after normalization
  size_t output_vertex_count = 0;
  size_t unmapped_edge_count = 0;

  bool area_limit_met = true;
  bool topology_preserved = true;
  bool tolerance_reduced = false;
};

// Simplifies an existing Boolean-operation result.
//
// The input is normalized with a NonZero union first. Normalization removes
// duplicate/coincident edges, repairs overlaps and establishes a stable
// outer/hole hierarchy. Simplification then keeps a subset of the normalized
// vertices, preserving winding direction. If the requested tolerance cannot
// meet the area budget or change-region topology check, the effective
// tolerance is reduced automatically. At worst the normalized input is
// returned without geometric simplification.
//
// subjects and clips are only used for provenance lookup. They may be empty.
CollisionContourResult SimplifyCollisionContours(
  const Paths64& boolean_result,
  const std::vector<CollisionSourceInput>& subjects,
  const std::vector<CollisionSourceInput>& clips,
  const CollisionSimplifyOptions& options = {});

// Convenience wrapper that executes the Boolean operation before simplifying.
CollisionContourResult BuildCollisionContours(
  ClipType clip_type,
  FillRule fill_rule,
  const std::vector<CollisionSourceInput>& subjects,
  const std::vector<CollisionSourceInput>& clips,
  const CollisionSimplifyOptions& options = {});

} // namespace Clipper2Lib

#endif // CLIPPER_COLLISION_H
