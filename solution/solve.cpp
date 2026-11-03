// Reference solution for the problem "Robot and Patrolling Guards" (see PROBLEM.md)
#include "../src/Deltastar.hpp"
using namespace delta;

int main() {
    int n, m, k, T;
    std::cin >> n >> m >> k >> T;
    std::vector<std::string> grid(n);
    Pos s{}, g{};
    for (int i = 0; i < n; i++) {
        std::cin >> grid[i];
        for (int j = 0; j < m; j++) {
            if (grid[i][j] == 'S') { s = {j, i}; grid[i][j] = '.'; }
            if (grid[i][j] == 'G') { g = {j, i}; grid[i][j] = '.'; }
        }
    }
    std::vector<Guard> guards(k);
    for (auto& gd : guards) {
        int L; std::cin >> L;
        gd.route.resize(L);
        for (auto& p : gd.route) { int r, c; std::cin >> r >> c; p = {c - 1, r - 1}; }
    }
    DeltaStar planner(grid, guards, T);
    auto res = planner.solve(s, g);
    std::cout << (res.found ? res.steps : -1) << "\n";
}
