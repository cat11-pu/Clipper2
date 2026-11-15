# Variable-Distance Offset (narrow interface)

Header-only interface in
`Clipper2Lib/include/clipper2/clipper.variableoffset.h`.

## Interface

- `struct OffsetRun { size_t start, end; double delta; }`
  One constant-distance run. `[start, end)` covers the open polyline from
  vertex `start` to vertex `end` (inclusive). Runs must partition the whole
  contour: closed contours end at `N`, open contours end at `N-1`.
  `delta >= 0` (inflation only; negative values throw).
- `struct VariableOffsetOptions`
  `join_type` (`Square`/`Bevel`/`Round`/`Miter`, configurable sharp-corner
  handling), `end_type` (`Polygon` for closed, otherwise an open `EndType`),
  `miter_limit`, `arc_tolerance`, `preserve_collinear`, `reverse_solution`.
- `Paths64 VariableOffsetContour(const Path64&, const std::vector<OffsetRun>&, const VariableOffsetOptions& = {})`
  Returns the raw fragments for one contour.
- `Paths64 VariableOffsetShape(const Paths64&, const std::vector<OffsetRun>&, const VariableOffsetOptions& = {})`
  Convenience wrapper that runs every contour (outer + holes) and merges all
  fragments in one `NonZero` union.

## Semantics

For a closed contour every run is offset independently as an open polyline
with round end caps, and the original contour is added unchanged. The final
union:

- keeps the interior and every hole (the original contour preserves
  orientation; holes stay opposite to their outer boundary);
- produces a continuous outer fillet at run junctions because the round caps
  of the two adjacent runs overlap;
- is point-for-point identical to manually offsetting each segment with its
  own `ClipperOffset` and unioning the pieces plus the original contour, since
  both pipelines emit the identical fragment set and use the same union.

## Validation

- `Tests/TestVariableOffset.cpp` (11 GoogleTest cases): pointwise equality
  with an independent hand-written "offset each segment, then merge"
  reference for all four join types, hole orientation, fillet continuity,
  zero-delta runs, empty schedule, invalid schedules (gap/overlap/overrun/
  empty run/non-zero origin), negative-delta rejection, open contours,
  configurable miter limit and repeatability.
- `Examples/VariableOffset/VariableOffsetStress.cpp`: 1000 randomized
  iterations over 6-23 vertex contours with holes, comparing the wrapper
  against the independent reference. Under AddressSanitizer + LeakSanitizer
  + UBSan the run reports 0 mismatching iterations, 0 point differences and
  no leaks. An optimized build finishes 1000 iterations in ~0.12 s with a
  ~4.7 MB resident set.

Build the stress target with
`cmake --build <dir> --target VariableOffsetStress` and the API demo with
`--target VariableOffsetApi`.
