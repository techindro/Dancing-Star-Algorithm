#include "dancingstar.hpp"
using namespace dancing;

int main() {
    std::vector<std::string> grid = {
        "...........",
        ".####.###..",
        ".....#.....",
        "####.#.##..",
        "...........",
        ".#####.###.",
        "..........."
    };
    Pos start{0, 0}, goal{10, 6};

    Guard a, b;
    for (int x = 0; x <= 9; x++) a.route.push_back({x, 4});   // sweeps row 4
    for (int y = 0; y <= 5; y++) b.route.push_back({10, y});  // sweeps column 10

    DancingStar planner(grid, {a, b}, 60);
    auto r = planner.solve(start, goal);
    if (!r.found) { std::cout << "No path found.\n"; return 0; }

    std::cout << "DancingStar found a path in " << r.steps << " steps ("
              << r.expanded << " states expanded)\n\n";
    for (size_t t = 0; t < r.path.size(); t++) planner.draw((int)t, r.path[t], goal);
}
