"""Compare nonreg result files per (instance, seed).

usage: compare.py <results_dir> <name_a> <name_b> [more names...]
Prints per-(instance, seed) rows for every name, then a bit-identity verdict
between the first two names (cost, served, km, iters, per-zone cost/iters).
"""
import json
import os
import sys

res_dir = sys.argv[1]
names = sys.argv[2:]


def load(name):
    rows = {}
    p = os.path.join(res_dir, name + ".jsonl")
    if not os.path.isfile(p):
        return rows
    for line in open(p, encoding="utf-8"):
        line = line.strip()
        if not line:
            continue
        r = json.loads(line)
        rows[(r["instance"], r["seed_offset"])] = r
    return rows


data = {n: load(n) for n in names}
keys = sorted(set().union(*[set(d) for d in data.values()]))

hdr = f"{'inst':9} {'sd':>2} " + " ".join(f"| {n[:22]:>40}" for n in names)
print(hdr)
print("          " + " ".join("| " + f"{'served':>6} {'cost':>11} {'km':>7} {'2day':>4} {'>day':>4} {'wall':>6}" for _ in names))
for k in keys:
    cells = []
    for n in names:
        r = data[n].get(k)
        if r is None:
            cells.append(f"| {'-':>40}")
        else:
            cells.append("| " + f"{r['served']:>6} {r['cost']:>11.0f} {r['km']:>7.1f} "
                         f"{r.get('routes_2day', '?')!s:>4} {r.get('routes_over_day', '?')!s:>4} "
                         f"{r['total_wall_s']:>6.1f}")
    print(f"{k[0]:9} {k[1]:>2} " + " ".join(cells))

if len(names) >= 2:
    a, b = names[0], names[1]
    print(f"\nbit-identity {a} vs {b}:")
    common = [k for k in keys if k in data[a] and k in data[b]]
    same = 0
    for k in common:
        ra, rb = data[a][k], data[b][k]
        za = [(z["zone"], z["cost"], z["served"], z["dist_m"], z["iters"], z["duration_s"]) for z in ra["zones"]]
        zb = [(z["zone"], z["cost"], z["served"], z["dist_m"], z["iters"], z["duration_s"]) for z in rb["zones"]]
        ident = (ra["cost"] == rb["cost"] and ra["served"] == rb["served"] and ra["km"] == rb["km"]
                 and ra["iters"] == rb["iters"] and sorted(za) == sorted(zb))
        same += ident
        flag = "IDENTICAL" if ident else "DIFF"
        extra = ""
        if not ident:
            extra = f"  cost {ra['cost']} vs {rb['cost']}, served {ra['served']} vs {rb['served']}, iters {ra['iters']} vs {rb['iters']}"
        print(f"  {k[0]} seed{k[1]}: {flag}{extra}")
    print(f"  {same}/{len(common)} identical")
