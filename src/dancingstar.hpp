// Dancing Star Algorithm - Space-Time A* for grids with patrolling (moving) obstacles.
// State = (x, y, t). Moves: 4 directions or WAIT. Every move costs 1 time unit.
// Guard occupancy is precomputed, so every move check is O(1).
#pragma once
#include <bits/stdc++.h>

namespace dancing {

struct Pos { int x, y; };

// A guard patrols its route back and forth: p0 p1 ... p(L-1) p(L-2) ... p1 p0 ...
struct Guard {
    std::vector<Pos> route;
    Pos at(int t) const {
        int n = (int)route.size();
        if (n == 1) return route[0];
        int period = 2 * (n - 1);
        int k = t % period;
        if (k >= n) k = period - k;
        return route[k];
    }
};

class DancingStar {
public:
    struct Result {
        bool found = false;
        int steps = -1;               // minimum time to reach the goal
        std::vector<Pos> path;        // path[t] = robot position at time t
        long long expanded = 0;       // number of states popped from the heap
    };

    DancingStar(std::vector<std::string> grid, std::vector<Guard> guards, int maxT)
        : grid_(std::move(grid)), guards_(std::move(guards)), maxT_(maxT) {
        H_ = (int)grid_.size();
        W_ = (int)grid_[0].size();
        K_ = (int)guards_.size();
        if (K_ > 32) throw std::runtime_error("at most 32 guards supported");
        for (auto& g : guards_) {
            if (g.route.empty()) throw std::invalid_argument("guard route must not be empty");
            for (auto& c : g.route)
                if (c.x < 0 || c.y < 0 || c.x >= W_ || c.y >= H_)
                    throw std::invalid_argument("guard route cell is outside the grid");
        }
        // occupancy[t][cell] = bitmask of guards standing on that cell at time t
        occ_.assign((size_t)(maxT_ + 2) * H_ * W_, 0u);
        for (int t = 0; t <= maxT_ + 1; t++)
            for (int g = 0; g < K_; g++) {
                Pos p = guards_[g].at(t);
                occ_[idx(p.x, p.y, t)] |= (1u << g);
            }
    }

    Result solve(Pos start, Pos goal) const {
        Result res;
        if (blocked(start.x, start.y, 0)) return res;

        std::vector<char> seen((size_t)(maxT_ + 1) * H_ * W_, 0);
        std::vector<int> parent((size_t)(maxT_ + 1) * H_ * W_, -1);
        auto h = [&](int x, int y) { return std::abs(x - goal.x) + std::abs(y - goal.y); };

        using Node = std::tuple<int, int, int, int>;   // f, x, y, t
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
        pq.push({h(start.x, start.y), start.x, start.y, 0});
        seen[idx(start.x, start.y, 0)] = 1;

        const int dx[5] = {0, 1, -1, 0, 0};
        const int dy[5] = {0, 0, 0, 1, -1};              // first move = WAIT

        while (!pq.empty()) {
            auto [f, x, y, t] = pq.top();
            pq.pop();
            res.expanded++;
            int cur = (int)idx(x, y, t);

            if (x == goal.x && y == goal.y) {            // g == t, so first hit is optimal
                res.found = true;
                res.steps = t;
                for (int c = cur; c != -1; c = parent[c]) {
                    int cell = c % (H_ * W_);
                    res.path.push_back({cell % W_, cell / W_});
                }
                std::reverse(res.path.begin(), res.path.end());
                return res;
            }
            if (t >= maxT_) continue;

            for (int d = 0; d < 5; d++) {
                int nx = x + dx[d], ny = y + dy[d], nt = t + 1;
                if (blocked(nx, ny, nt)) continue;
                // swap conflict: a guard stands on my next cell now and on my cell next step
                if (occ_[idx(nx, ny, t)] & occ_[idx(x, y, nt)]) continue;
                size_t nid = idx(nx, ny, nt);
                if (seen[nid]) continue;
                seen[nid] = 1;
                parent[nid] = cur;
                pq.push({nt + h(nx, ny), nx, ny, nt});
            }
        }
        return res;
    }

    // Draw the world at time t (R robot, X guard, G goal, # wall)
    void draw(int t, Pos robot, Pos goal, std::ostream& os = std::cout) const {
        std::vector<std::string> v = grid_;
        v[goal.y][goal.x] = 'G';
        for (auto& g : guards_) { Pos q = g.at(t); v[q.y][q.x] = 'X'; }
        v[robot.y][robot.x] = 'R';
        os << "t = " << t << "\n";
        for (auto& r : v) os << r << "\n";
        os << "\n";
    }

private:
    std::vector<std::string> grid_;
    std::vector<Guard> guards_;
    int W_, H_, K_, maxT_;
    std::vector<uint32_t> occ_;

    size_t idx(int x, int y, int t) const { return ((size_t)t * H_ + y) * W_ + x; }

    bool blocked(int x, int y, int t) const {
        if (x < 0 || y < 0 || x >= W_ || y >= H_) return true;
        if (grid_[y][x] == '#') return true;
        return occ_[idx(x, y, t)] != 0;
    }
};

}  // namespace dancing
