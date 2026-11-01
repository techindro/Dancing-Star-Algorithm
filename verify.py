# Independent brute-force checker (plain BFS over (row, col, time)). Used to validate the C++ solver.
import sys, collections

def solve(text):
    tok = text.split()
    n, m, k, T = map(int, tok[:4]); p = 4
    grid = [list(tok[p + i]) for i in range(n)]; p += n
    guards = []
    for _ in range(k):
        L = int(tok[p]); p += 1
        route = [(int(tok[p + 2*i]) - 1, int(tok[p + 2*i + 1]) - 1) for i in range(L)]; p += 2 * L
        guards.append(route)
    def gpos(route, t):
        L = len(route)
        if L == 1: return route[0]
        per = 2 * (L - 1); q = t % per
        return route[q if q < L else per - q]
    s = g = None
    for i in range(n):
        for j in range(m):
            if grid[i][j] == 'S': s = (i, j)
            if grid[i][j] == 'G': g = (i, j)
    def occ(t): return {gpos(r, t) for r in guards}
    if s in occ(0): return -1
    frontier = {s}
    for t in range(T + 1):
        if g in frontier: return t
        nxt = set(); o_now, o_next = occ(t), occ(t + 1)
        for (r, c) in frontier:
            for dr, dc in ((0,0),(1,0),(-1,0),(0,1),(0,-1)):
                nr, nc = r + dr, c + dc
                if not (0 <= nr < n and 0 <= nc < m) or grid[nr][nc] == '#': continue
                if (nr, nc) in o_next: continue
                swap = any(gpos(rt, t) == (nr, nc) and gpos(rt, t + 1) == (r, c) for rt in guards)
                if swap: continue
                nxt.add((nr, nc))
        frontier = nxt
    return -1

if __name__ == "__main__":
    print(solve(open(sys.argv[1]).read()))
