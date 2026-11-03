# Delta Star Algorithm – Mathematical Formulation

*Formulas, correctness proof and complexity derivation for the Delta Star Algorithm (Space-Time A\* with patrolling obstacles).*

[← Back to README](README.md)

## Problem definition

**Input:** grid $W \times H$ with walls, start $S$, goal $G$, horizon $T$, and $K$ obstacles with known positions $g_i(t)$ for $t = 0, \dots, T$.

**Output:** the minimum $T^*$ and a path $v_0 = S, v_1, \dots, v_{T^*} = G$ with $v_{s+1} \in N(v_s) \cup \{v_s\}$, $v_s \notin O(s)$ for all $s$, and no swap with an obstacle – or "no path" if none exists with $T^* \le T$.

## Notation

| Symbol | Meaning |
|---|---|
| $W \times H$ | grid size |
| $V$ | set of free cells (not walls) |
| $K$ | number of guards |
| $T$ | time horizon |
| $S, G$ | start and goal cell |
| $N(v)$ | side-adjacent cells of $v$ |

## 1. Guard position

Guard $i$ has a route $p_0, p_1, \dots, p_{L-1}$ and bounces at both ends. Let $P = 2(L-1)$ and $q = t \bmod P$. Then

$$
\mathrm{idx}(t) =
\begin{cases}
q & \text{if } q \le L-1\\
P - q & \text{otherwise}
\end{cases}
\qquad
g_i(t) = p_{\mathrm{idx}(t)}
$$

(for $L = 1$ the guard never moves). Cells occupied at time $t$:

$$
O(t) = \{\, g_i(t) : i = 1,\dots,K \,\}
$$

## 2. State and transitions

A state is a pair $s = (v, t)$: cell and time. A move $(v,t) \to (v',t+1)$ is legal iff

$$
v' \in N(v) \cup \{v\}, \qquad v' \in V, \qquad v' \notin O(t+1)
$$

and the robot does not swap places with a guard:

$$
\nexists\, i : \; g_i(t) = v' \;\wedge\; g_i(t+1) = v
$$

The case $v' = v$ is the **wait** move. In code, with the bitmask table $\mathrm{occ}[t][c]$, the swap test is
`occ[t][v'] & occ[t+1][v] == 0`.

## 3. Cost equation (the Dijkstra-style relaxation)

Like Dijkstra's $d[v] = \min(d[v],\, d[u] + w(u,v))$, the cost of reaching a state is

$$
g(v',t+1) = \min\bigl(g(v',t+1),\; g(v,t) + w\bigr)
$$

Every move (including wait) costs $w = 1$ time unit, and every state $(v,t)$ is only reachable at time exactly $t$, so

$$
g(v,t) = t
$$

and no `min` is needed – the first visit to a state is already optimal (this is why the code only keeps a `seen` array).

If you add a wait cost $c_w$ and a danger penalty $\lambda$:

$$
g(v',t+1) = \min\bigl(g(v',t+1),\; g(v,t) + c(v \to v') + \lambda \cdot \mathrm{danger}(v', t+1)\bigr)
$$

## 4. Layer (dynamic-programming) form

Let $R_t$ be the set of cells the robot can stand on at time $t$:

$$
R_0 = \{S\}, \qquad
R_{t+1} = \{\, v' : \exists\, v \in R_t \text{ with a legal move } (v,t)\to(v',t+1) \,\}
$$

The answer is

$$
T^* = \min\{\, t \le T : G \in R_t \,\}
$$

(or $-1$ if no such $t$ exists).

## 5. Heuristic and optimality

$$
f(v,t) = g(v,t) + h(v) = t + h(v), \qquad h(v) = |x - x_G| + |y - y_G|
$$

* **Admissible.** At least $h(v)$ moves are needed to reach $G$ and every move takes $\ge 1$ time unit, so $h(v) \le$ true remaining time.
* **Consistent.** For every move, $|h(v) - h(v')| \le 1$ and the cost is $1$, so $h(v) \le 1 + h(v')$. For the wait move $h(v') = h(v)$, which also satisfies it.

With a consistent heuristic, A\* pops a state with its optimal $g$ the first time. So the first time a state $(G, t)$ leaves the heap, $t = T^*$ is the minimum arrival time.

## 6. Complexity

Number of states $\le W \cdot H \cdot (T+1)$. Each state has 5 successors, each checked in $O(1)$, and heap operations cost $O(\log(WHT))$:

$$
\text{Time} = O\bigl(W H T \log(WHT)\bigr), \qquad \text{Memory} = O(WHT)
$$

Preprocessing the guard table: $O(T \cdot K)$.

### Tighter bound for a corner start

At time $t$ the robot can only be on cells with $x + y \le t$, so

$$
|R_t| \le \min\!\Bigl(WH,\; \tfrac{(t+1)(t+2)}{2}\Bigr)
$$

For $T \le \min(W,H)$ the total number of states is at most

$$
\sum_{t=0}^{T} \frac{(t+1)(t+2)}{2} = \binom{T+3}{3} \approx \frac{T^3}{6}
$$

### Check against the benchmark (worst case: goal unreachable, $N = T$)

| $N = T$ | $\binom{T+3}{3}$ | measured states expanded |
|---|---|---|
| 20 | 1 771 | 1 746 |
| 60 | 39 711 | 39 558 |
| 120 | 302 621 | 301 927 |

Measured values are slightly below the bound because guards block some cells. This is why the benchmark graph shows time growing like $N^3$ when $T = N$.

## 7. Guard table

$$
\mathrm{occ}[t][c] = \bigvee_{i \,:\, g_i(t) = c} 2^{\,i}
$$

Built once in $O(T \cdot K)$; afterwards every collision and swap test is $O(1)$ (a single bit operation).

## 8. When the results are valid

The optimality proof (section 5) and the bounds (section 6) need:

1. unit cost for every move and wait (or non-negative costs, with the heuristic still consistent);
2. 4-connected moves, so that the Manhattan distance never overestimates;
3. guard positions $g_i(t)$ known for all $t \le T$;
4. a finite horizon $T$. "Not found" means $G \notin R_t$ for all $t \le T$.

With $K \le 32$ guards the bitmask fits in a `uint32_t`; memory is about $9\,WH(T+1)$ bytes (1 byte `seen`, 4 bytes `parent`, 4 bytes guard table per state).

## Prior art

Searching in $(x, y, t)$ is the known *Space-Time A\** idea. The formulas above are its standard analysis written for patrolling obstacles; the tight bound $\binom{T+3}{3}$ and the bitmask swap test are checked against the benchmarks in this repository.
