#include <utility>
#include <vector>

namespace Offset {
typedef std::pair<int, int> Offset;
typedef std::vector<Offset> Vec;

const static Vec Structural = {
    {0, 1},
    {0, -1},
    {1, 0},
    {-1, 0},
};
const static Vec Bend = {
    {0, 2},
    {0, -2},
    {2, 0},
    {-2, 0},
};
const static Vec Shear = {
    {1, 1},
    {1, -1},
    {-1, 1},
    {-1, -1},
};

const static Vec Surrounding = {
    {0, 1}, {0, -1}, {1, 0}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
};

const static Vec Interp = {
    {1, 2}, {1, -2}, {2, 1}, {2, -1}, {-1, 2}, {-1, -2}, {-2, 1}, {-2, -1},
};
}  // namespace Offset
