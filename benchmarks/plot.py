import csv, re, math, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------- parse results.csv ----------
sections, cur = {}, None
for line in open("benchmarks/results.csv"):
    line = line.strip()
    if line.startswith("#"): cur = line[1:]; sections[cur] = []; continue
    if line: sections[cur].append(line.split(","))
def table(name):
    rows = sections[name]; head, body = rows[0], rows[1:]
    return {h: [float(r[i]) for r in body] for i, h in enumerate(head)}

A, B, C = table("scaleN"), table("scaleT"), table("compare")

plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": 0.3})
fig, ax = plt.subplots(1, 3, figsize=(16, 4.6))

# (a) time vs N  (T = N, worst case)
N, ms = A["N"], A["ms"]
ax[0].loglog(N, ms, "o-", color="#2563eb", label="Delta Star (measured)")
ref = [ms[0] * (n / N[0]) ** 3 for n in N]
ax[0].loglog(N, ref, "--", color="gray", label="reference  ~ N³")
ax[0].set_title("(a) Worst-case time vs grid size N  (T = N)")
ax[0].set_xlabel("grid side N  (N × N cells)"); ax[0].set_ylabel("time (ms)")
ax[0].legend()

# (b) time vs T for N = 60
T, ms2 = B["T"], B["ms"]
ax[1].plot(T, ms2, "o-", color="#16a34a", label="Delta Star (measured)")
ax[1].set_title("(b) Worst-case time vs time horizon T  (N = 60)")
ax[1].set_xlabel("time horizon T"); ax[1].set_ylabel("time (ms)")
ax[1].legend()

# (c) safety comparison
K = [int(k) for k in C["guards"]]
x = range(len(K)); w = 0.38
ax[2].bar([i - w/2 for i in x], C["static_collide_pct"], w, color="#dc2626", label="Plain shortest path (ignores time)")
ax[2].bar([i + w/2 for i in x], C["delta_collide_pct"], w, color="#2563eb", label="Delta Star")
for i in x: ax[2].text(i + w/2, 1.2, "0 %", ha="center", color="#2563eb", fontsize=9, weight="bold")
ax[2].set_xticks(list(x)); ax[2].set_xticklabels(K)
ax[2].set_xlabel("number of patrolling guards (30 × 30 grid, 300 random maps each)")
ax[2].set_ylabel("paths that collide with a guard (%)")
ax[2].set_title("(c) Collision rate")
ax[2].legend(loc="upper left")
plt.tight_layout(); plt.savefig("docs/benchmark.png", dpi=150); plt.close()

# ---------- path picture from demo ----------
frames = re.split(r"\n(?=t = )", open("/tmp/demo_out.txt").read().split("\n\n", 1)[1].strip())
robot, guards_by_t, grid = [], [], None
for fr in frames:
    rows = fr.splitlines()[1:]
    if grid is None: grid = [[c == "#" for c in r] for r in rows]
    guards_by_t.append([(x, y) for y, r in enumerate(rows) for x, c in enumerate(r) if c == "X"])
    for y, r in enumerate(rows):
        if "R" in r: robot.append((r.index("R"), y))
H, W = len(grid), len(grid[0])
fig, ax = plt.subplots(figsize=(9.5, 5.6))
for y in range(H):
    for x in range(W):
        ax.add_patch(plt.Rectangle((x - .5, y - .5), 1, 1, color="#1f2937" if grid[y][x] else "#f3f4f6", ec="white"))
gx = [p[0] for t in guards_by_t for p in t]; gy = [p[1] for t in guards_by_t for p in t]
ax.scatter(gx, gy, s=70, c="#fca5a5", marker="s", label="cells visited by guards over time", zorder=2)
xs, ys = [p[0] for p in robot], [p[1] for p in robot]
sc = ax.scatter(xs, ys, c=range(len(robot)), cmap="viridis", s=150, zorder=4, label="robot (colour = time)")
ax.plot(xs, ys, color="#2563eb", lw=2, zorder=3)
for t in range(0, len(robot), 2): ax.text(xs[t], ys[t], str(t), ha="center", va="center", color="white", fontsize=7, zorder=5)
ax.text(0, 0, "S", ha="center", va="center", fontsize=11, weight="bold", color="white", zorder=6)
ax.text(W - 1, H - 1, "G", ha="center", va="center", fontsize=11, weight="bold", color="white", zorder=6)
ax.set_xlim(-.5, W - .5); ax.set_ylim(H - .5, -.5); ax.set_aspect("equal"); ax.grid(False)
ax.set_xticks([]); ax.set_yticks([])
ax.set_title(f"Delta Star demo: robot reaches the goal in {len(robot)-1} steps while 2 guards patrol")
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.02), ncol=2, frameon=False)
plt.colorbar(sc, ax=ax, fraction=0.03, label="time step")
plt.tight_layout(); plt.savefig("docs/demo_path.png", dpi=150); plt.close()
print("ok")
