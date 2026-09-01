"""Re-POST stored request_payload to /v1/optimize with dev JWT (sync battery item 3)."""
import json
import sys
import time
import uuid
from datetime import datetime, timedelta, timezone

import jwt
import urllib.request
import urllib.error

SECRET = "dev-secret-please-change-0123456789abcdef"
BASE = sys.argv[1] if len(sys.argv) > 1 else "http://localhost:8001"
PAYLOAD_PATH = sys.argv[2] if len(sys.argv) > 2 else "sync_885_request_payload.json"
OUT_PATH = sys.argv[3] if len(sys.argv) > 3 else "sync_885_response.json"


def make_token() -> str:
    now = datetime.now(tz=timezone.utc)
    payload = {
        "sub": "admin",
        "email": "admin@acme.com",
        "site": "standalone",
        "tax_id": "12345678000190",
        "role": "admin",
        "enable_diagnostics": True,
        "type": "access",
        "exp": now + timedelta(hours=1),
        "iat": now,
        "jti": str(uuid.uuid4()),
    }
    return jwt.encode(payload, SECRET, algorithm="HS256")


def main() -> None:
    with open(PAYLOAD_PATH, encoding="utf-8") as f:
        body = json.load(f)
    token = make_token()
    req = urllib.request.Request(
        f"{BASE}/v1/optimize",
        data=json.dumps(body).encode("utf-8"),
        headers={
            "Authorization": f"Bearer {token}",
            "Content-Type": "application/json",
            "X-Tenant-Site": "standalone",
            "X-Tenant-Tax-Id": "12345678000190",
        },
        method="POST",
    )
    t0 = time.perf_counter()
    try:
        with urllib.request.urlopen(req, timeout=1200) as resp:
            raw = resp.read().decode("utf-8")
            status = resp.status
    except urllib.error.HTTPError as e:
        raw = e.read().decode("utf-8", errors="replace")
        status = e.code
    elapsed = time.perf_counter() - t0
    with open(OUT_PATH, "w", encoding="utf-8") as f:
        f.write(raw)
    print(f"HTTP status: {status}, elapsed_s={elapsed:.1f}")
    print(f"raw bytes: {len(raw)}")
    try:
        j = json.loads(raw)
        s = j.get("summary", {})
        em = j.get("engine_metadata", {})
        print(json.dumps({
            "status": status,
            "elapsed_s": round(elapsed, 1),
            "plan_status": j.get("plan_status"),
            "total_distance_m": s.get("total_distance_m"),
            "total_duration_s": s.get("total_duration_s"),
            "total_service_s": s.get("total_service_s"),
            "routes": len(j.get("routes") or []),
            "assigned": s.get("assigned"),
            "unassigned": s.get("unassigned"),
            "violations": len(j.get("violations") or []),
            "engine_metadata": {k: em.get(k) for k in ("solver", "feasible", "strategy", "decomposition_zones", "max_runtime_s", "real_window_violations")},
        }, ensure_ascii=False, indent=1))
    except Exception as e:
        print("response parse failed:", e)
        print(raw[:2000])


if __name__ == "__main__":
    main()
