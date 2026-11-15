// Minimal usage of the variable-distance offset narrow interface.
// Builds a shape with one hole, offsets the contour in two constant-distance
// runs and prints the merged result. The same result is obtained by offsetting
// each run with its own ClipperOffset and unioning the pieces.

#include <iostream>

#include "clipper2/clipper.h"
#include "clipper2/clipper.variableoffset.h"

using namespace Clipper2Lib;

int main()
{
  Path64 outer = MakePath({100, 100, 500, 100, 500, 400, 100, 400});
  Path64 hole  = MakePath({250, 200, 250, 300, 350, 300, 350, 200});
  if (Area(outer) < 0) std::reverse(outer.begin(), outer.end());
  if (Area(hole)  > 0) std::reverse(hole.begin(), hole.end());
  const Paths64 subject{ outer, hole };

  // Both contours have 4 vertices; two runs split each ring in half.
  const std::vector<OffsetRun> runs{
    OffsetRun(0, 2, 40.0), // vertices 0..1 offset by 40
    OffsetRun(2, 4, 15.0)  // vertices 2..3 offset by 15
  };

  VariableOffsetOptions options;
  options.join_type   = JoinType::Round; // configurable: Square/Bevel/Round/Miter
  options.miter_limit = 2.0;

  const Paths64 solution = VariableOffsetShape(subject, runs, options);

  std::cout << "solution paths: " << solution.size() << '\n';
  for (const Path64& path : solution)
    std::cout << "  path points=" << path.size()
              << " signed area=" << Area(path) << '\n';
  return 0;
}
