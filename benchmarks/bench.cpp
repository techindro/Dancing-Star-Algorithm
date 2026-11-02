// Benchmarks for DeltaStar. Prints CSV sections to stdout.
#include "../src/deltastar.hpp"
using namespace dancing;
using Clock = std::chrono::steady_clock;

static std::mt19937 rng(12345);
static int rnd(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }

struct Instance { std::vector<std::string> grid; std::vector<Guard> guards; };

Instance makeInstance(int N, int K, double wallP) {
    Instance in;
    in.grid.assign(N, std::string(N, '.'));
    for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++)
            if (std::uniform_real_distribution<double>(0, 1)(rng) < wallP) in.grid[y][x] = '#';
    in.grid[0][0] = '.'; in.grid[N - 1][N - 1] = '.';
    while ((int)in.guards.size() < K) {
        bool horiz = rnd(0, 1);
        int L = rnd(3, 8);
        int x = rnd(0, N - 1), y = rnd(0, N - 1);
        Guard g; bool ok = true;
        for (int i = 0; i < L && ok; i++) {
            int cx = x + (horiz ? i : 0), cy = y + (horiz ? 0 : i);
            if (cx >= N || cy >= N || in.grid[cy][cx] == '#') ok = false;
            else g.route.push_back({cx, cy});
        }
        if (ok) in.guards.push_back(g);
    }
    return in;
}

// Plain A*/BFS on the static grid only (ignores guards) -> list of cells
std::vector<Pos> staticPath(const std::vector<std::string>& grid, Pos s, Pos g) {
    int N = (int)grid.size();
    std::vector<int> par(N * N, -2);
    std::queue<int> q; q.push(s.y * N + s.x); par[s.y * N + s.x] = -1;
    int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
    while (!q.empty()) {
        int c = q.front(); q.pop();
        if (c == g.y * N + g.x) break;
        for (int d = 0; d < 4; d++) {
            int nx = c % N + dx[d], ny = c / N + dy[d];
            if (nx < 0 || ny < 0 || nx >= N || ny >= N || grid[ny][nx] == '#') continue;
            if (par[ny * N + nx] != -2) continue;
            par[ny * N + nx] = c; q.push(ny * N + nx);
        }
    }
    std::vector<Pos> p;
    if (par[g.y * N + g.x] == -2) return p;
    for (int c = g.y * N + g.x; c != -1; c = par[c]) p.push_back({c % N, c / N});
    std::reverse(p.begin(), p.end());
    return p;
}

bool collides(const std::vector<Pos>& path, const std::vector<Guard>& guards) {
    for (size_t t = 0; t < path.size(); t++) {
        for (auto& g : guards) {
            Pos a = g.at((int)t);
            if (a.x == path[t].x && a.y == path[t].y) return true;
            if (t + 1 < path.size()) {
                Pos b = g.at((int)t + 1);
                if (a.x == path[t + 1].x && a.y == path[t + 1].y &&
                    b.x == path[t].x && b.y == path[t].y) return true;
            }
        }
    }
    return false;
}

int main() {
    // 1) Worst case: goal unreachable -> whole (x,y,t) space is explored. T = N.
    std::cout << "#scaleN\nN,T,K,ms,expanded\n";
    for (int N : {20, 30, 40, 50, 60, 80, 100, 120}) {
        double ms = 0; long long exp = 0; int reps = 3;
        for (int r = 0; r < reps; r++) {
            auto in = makeInstance(N, N / 5, 0.0);
            in.grid[N - 1][N - 2] = '#'; in.grid[N - 2][N - 1] = '#';   // wall the goal in
            int T = N;
            auto t0 = Clock::now();
            DeltaStar d(in.grid, in.guards, T);
            auto res = d.solve({0, 0}, {N - 1, N - 1});
            ms += std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            exp += res.expanded;
        }
        std::cout << N << "," << N << "," << N / 5 << "," << ms / reps << "," << exp / reps << "\n";
    }
    // 2) Worst case with fixed N=60, growing time horizon T
    std::cout << "#scaleT\nN,T,K,ms,expanded\n";
    for (int T : {20, 40, 80, 120, 160, 240, 320, 400}) {
        int N = 60; double ms = 0; long long exp = 0; int reps = 3;
        for (int r = 0; r < reps; r++) {
            auto in = makeInstance(N, 12, 0.0);
            in.grid[N - 1][N - 2] = '#'; in.grid[N - 2][N - 1] = '#';
            auto t0 = Clock::now();
            DeltaStar d(in.grid, in.guards, T);
            auto res = d.solve({0, 0}, {N - 1, N - 1});
            ms += std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            exp += res.expanded;
        }
        std::cout << N << "," << T << ",12," << ms / reps << "," << exp / reps << "\n";
    }
    // 3) Safety comparison on random instances: static shortest path vs DancingStar
    std::cout << "#compare\nguards,instances,static_collide_pct,dancing_collide_pct,delta_solved_pct,avg_extra_steps\n";
    for (int K : {2, 5, 10, 15, 20}) {
        int N = 30, inst = 300, sc = 0, solved = 0, dc = 0, valid = 0; double extra = 0;
        for (int i = 0; i < inst; i++) {
            auto in = makeInstance(N, K, 0.15);
            auto sp = staticPath(in.grid, {0, 0}, {N - 1, N - 1});
            if (sp.empty()) continue;
            valid++;
            if (collides(sp, in.guards)) sc++;
            DeltaStar d(in.grid, in.guards, 200);
            auto res = d.solve({0, 0}, {N - 1, N - 1});
            if (res.found) {
                solved++;
                if (collides(res.path, in.guards)) dc++;
                extra += res.steps - ((int)sp.size() - 1);
            }
        }
        std::cout << K << "," << valid << "," << 100.0 * sc / valid << "," << 100.0 * dc / std::max(1, solved)
                  << "," << 100.0 * solved / valid << "," << extra / std::max(1, solved) << "\n";
    }
}
