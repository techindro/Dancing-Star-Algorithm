# Robot and Patrolling Guards

**time limit per test:** 1 second  
**memory limit per test:** 256 megabytes  
**input:** standard input  **output:** standard output

A robot stands on cell `S` of an `n × m` grid. Some cells are walls (`#`), the others are free (`.`). The robot wants to reach the cell `G` as fast as possible.

There are `k` guards. Every guard has a **patrol route**: a sequence of `L` free cells `p1, p2, ..., pL`, where consecutive cells share a side. The guard walks along the route one cell per time step and bounces at both ends, so its position at time `t = 0, 1, 2, ...` is

```
p1, p2, ..., pL, p(L-1), ..., p2, p1, p2, ...
```

(if `L = 1` the guard never moves).

Time is discrete. At every step the robot can **move to a side-adjacent free cell** or **wait** in its current cell. Moving and waiting both take exactly one time unit. The robot is not allowed to

* stand in the same cell as a guard at any integer time `t`, and
* swap places with a guard, i.e. move from cell `a` to cell `b` during the step in which a guard moves from `b` to `a`.

Several guards may stand in the same cell. Find the minimum time `t` (`0 ≤ t ≤ T`) at which the robot can stand on `G`, or report that this is impossible.

## Input

The first line contains four integers `n`, `m`, `k`, `T` (`1 ≤ n, m ≤ 30`, `0 ≤ k ≤ 10`, `1 ≤ T ≤ 500`).

Each of the next `n` lines contains a string of `m` characters: `.` (free cell), `#` (wall), `S` (start, exactly one) or `G` (goal, exactly one). `S` and `G` are free cells.

Then `k` guard descriptions follow. The description of a guard starts with a line containing the integer `L` (`1 ≤ L ≤ 30`), followed by `L` lines `r c` (`1 ≤ r ≤ n`, `1 ≤ c ≤ m`) – the cells of the route (rows are numbered from top, columns from left). All route cells are free (not walls) and consecutive cells share a side.

### Constraints summary

| Parameter | Limit |
|---|---|
| grid size `n`, `m` | 1 … 30 |
| guards `k` | 0 … 10 |
| time limit `T` | 1 … 500 |
| route length `L` | 1 … 30 |
| `S`, `G` | exactly one each, both free cells |

It is guaranteed that every route cell is free and that consecutive cells of a route share a side. Guards ignore the robot and each other.

## Output

Print one integer – the minimum time needed to reach `G`, or `-1` if the robot cannot reach `G` within `T` time units (this includes the case when a guard stands on `S` at time `0`).

## Examples

### Input
```
3 4 0 20
S...
.##.
...G
```
### Output
```
5
```

### Input
```
3 6 2 30
S.#..#
..##..
.....G
2
3 1
3 2
3
3 4
3 5
3 6
```
### Output
```
13
```

### Input
```
1 5 1 30
S...G
2
1 3
1 4
```
### Output
```
-1
```

## Note

In the first example there are no guards, so the answer is just the length of the shortest path.

In the second example the shortest path ignoring the guards has length `7`, but both guards patrol along the bottom row, so the robot is forced to wait and the answer is `13`.

In the third example the robot lives in a one-cell-wide corridor and a guard patrols inside it. The robot can never get past the guard without standing on it or swapping places with it, so the answer is `-1`.

## Solution idea

Plain BFS/A* on the grid is not enough because the free cells change with time. Add time to the state: `(row, col, t)`. From `(r, c, t)` the robot can go to five states at time `t + 1` (four moves + wait). Precompute for every time step which cells are occupied by guards, then every transition is checked in `O(1)`. The first time the goal is popped gives the answer.

* States: `n · m · (T + 1)` ≤ 30 · 30 · 501 ≈ 450 000
* Time: `O(n · m · T · log(n · m · T))` (`O(n · m · T)` with plain BFS since every step costs 1)
* Memory: `O(n · m · T)`
* Measured on the worst case allowed by the constraints (`30 × 30`, `k = 10`, `L = 30`, `T = 500`, goal unreachable): about 0.07 s and 11 MB.

Reference implementation: [`solution/solve.cpp`](solution/solve.cpp) (uses [`src/dancingstar.hpp`](src/dancingstar.hpp)). An independent brute-force checker is in [`benchmarks/verify.py`](benchmarks/verify.py).
