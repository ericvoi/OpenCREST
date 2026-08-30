#pragma once
#include <cstddef>

namespace openCREST {

// Method-of-images geometric channel: highest reflection order (bounces) the
// image expansion supports and the resulting max path count. Order n adds two
// rays (except the direct path), so 1 + 2*kMaxImageOrder arrivals at full
// expansion. Order 2 (5 paths) is the default; higher orders are opt-in.
constexpr int    kMaxImageOrder      = 4;
constexpr size_t MAX_GEOMETRIC_PATHS = 1 + 2 * kMaxImageOrder;  // 9

} // namespace openCREST
