"""Extract a duration (and distance) matrix for a resolved OptimizeRequest from
the local matrix_pairs PostgreSQL cache (read-only), keyed by coord_hash — same
mechanism as bench/extract_prod_run_matrix.py.

The request is a plain OptimizeRequest JSON (top-level "jobs"/"depot").
Output is a (n_jobs+1) x (n_jobs+1) square matrix: index 0 = depot, 1..n = jobs
(rows/cols follow job order, NOT deduplicated locations).

Usage:
    python extract_matrix_db.py <request.json> <out_matrix.json> [--distance <out_dist.json>]
"""
import hashlib
import json
import subprocess
import sys


def coord_hash(loc):
    lon, lat = round(float(loc[0]), 6), round(float(loc[1]), 6)
    return hashlib.sha256(f"{lon:.6f},{lat:.6f}".encode()).hexdigest()


def fetch_pairs(hashes):
    ids_sql = ",".join("'%s'" % h for h in hashes)
    sql = (
        "SELECT la.coord_hash, lb.coord_hash, p.dur_s, p.dist_m "
        "FROM matrix_pairs p "
        "JOIN matrix_locations la ON la.id = p.loc_a_id "
        "JOIN matrix_locations lb ON lb.id = p.loc_b_id "
        f"WHERE la.coord_hash IN ({ids_sql}) AND lb.coord_hash IN ({ids_sql});"
    )
    # Use stdin (docker exec -i) to avoid the Windows command-line length limit
    # (large IN-clauses blow past CreateProcess's 8 KiB cap).
    out = subprocess.run(
        ["docker", "exec", "-i", "hows-router-postgres-1", "psql", "-U",
         "router", "-d", "hows_router", "-t", "-A", "-F", "|"],
        input=sql, capture_output=True, text=True, encoding="utf-8",
    )
    if out.returncode != 0:
        raise RuntimeError(f"psql failed: {out.stderr}")
    pairmap = {}
    for line in out.stdout.strip().splitlines():
        if not line.strip():
            continue
        ha, hb, dur, dist = line.split("|")
        pairmap[(ha, hb)] = (float(dur), int(float(dist)))
    return pairmap


def main():
    req_path = sys.argv[1]
    out_dur = sys.argv[2]
    out_dist = sys.argv[3] if len(sys.argv) > 3 else None

    with open(req_path, encoding="utf-8") as fh:
        req = json.load(fh)
    depot = req["depot"]["location"]
    jobs = req["jobs"]
    locs = [depot] + [j["location"] for j in jobs]
    hashes = [coord_hash(l) for l in locs]
    n = len(hashes)
    uniq = len(set(hashes))
    print(f"n_locs={n} unique_hashes={uniq}")

    pairmap = fetch_pairs(hashes)
    print(f"pair rows fetched: {len(pairmap)}")

    dur_m = [[0.0] * n for _ in range(n)]
    dist_m = [[0.0] * n for _ in range(n)]
    missing = []
    for i in range(n):
        for j in range(n):
            key = (hashes[i], hashes[j])
            if key in pairmap:
                dur_m[i][j], dist_m[i][j] = pairmap[key]
            else:
                missing.append((i, j))
    print(f"missing cells: {len(missing)} / {n*n}")

    with open(out_dur, "w", encoding="utf-8") as fh:
        json.dump(dur_m, fh)
    print(f"saved duration matrix -> {out_dur}")
    if out_dist:
        with open(out_dist, "w", encoding="utf-8") as fh:
            json.dump(dist_m, fh)
        print(f"saved distance matrix -> {out_dist}")


if __name__ == "__main__":
    main()
