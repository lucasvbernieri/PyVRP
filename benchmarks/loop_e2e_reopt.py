"""Bateria 4 — end-to-end reopt sanity (in-process, loop venv build).

Runs the three repro flows (pin-free, pin, pin12) through the real FastAPI
route /v1/orders/reoptimize-route using TestClient, and reports the emitted
route metrics (distance, duration, warp, plan_status, warnings). This validates
the Route.h change end-to-end through the full hows-router reopt pipeline.

Usage:
  python loop_e2e_reopt.py
"""
from __future__ import annotations

import json
import os
import sys
import time

HOWS_ROUTER = r"C:\Users\lupi_\projetos\zanella\CascadeProjects\hows-router"
sys.path.insert(0, HOWS_ROUTER)

# Env for in-process boot against the live local stack (postgres / valhalla /
# ors / redis all published on localhost). DB is read-only here.
os.environ.setdefault(
    "ROUTER_DATABASE_URL",
    "postgresql+asyncpg://router:router@localhost:5432/hows_router",
)
os.environ.setdefault("ROUTER_JWT_SECRET", "dev-secret-please-change-0123456789abcdef")
os.environ.setdefault("ROUTER_ORS_BASE_URL", "http://localhost:8082/ors")
os.environ.setdefault("ROUTER_VALHALLA_BASE_URL", "http://localhost:8002")
os.environ.setdefault("ROUTER_VROOM_BASE_URL", "http://localhost:3000")
os.environ.setdefault("REDIS_URL", "redis://localhost:6379")

PAYLOADS = {
    "pin-free": os.path.join(HOWS_ROUTER, "reopt_repro_payload.json"),
    "pin": os.path.join(HOWS_ROUTER, "reopt_repro_payload_pin.json"),
    "pin12": os.path.join(HOWS_ROUTER, "reopt_repro_payload_pin12.json"),
}


def main() -> None:
    # NOTE: we do NOT monkeypatch pyvrp.Model.solve here. A naive patch would
    # replace the callable's signature and trip the app's own
    # ``_HAS_WAIT_COST_RATE`` fail-fast (which inspects Model.solve's
    # signature). The reopt response carries everything Bateria 4 needs.
    from src.main import app
    from fastapi.testclient import TestClient

    with TestClient(app) as client:
        lr = client.post(
            "/v1/auth/login",
            json={"email": "admin@acme.com", "password": "pass1234"},
        )
        print(f"login: {lr.status_code}")
        tok = lr.json()["access_token"]
        headers = {"Authorization": f"Bearer {tok}"}

        for name, path in PAYLOADS.items():
            payload = json.load(open(path, encoding="utf-8"))
            t0 = time.perf_counter()
            resp = client.post(
                "/v1/orders/reoptimize-route",
                headers=headers,
                json=payload,
            )
            wall = time.perf_counter() - t0
            print("=" * 70)
            print(f"[{name}] HTTP {resp.status_code} wall={wall:.1f}s")
            if resp.status_code != 200:
                print(resp.text[:500])
                continue
            j = resp.json()
            s = j.get("summary", {})
            m = j.get("engine_metadata", {})
            _route = (j.get("routes") or [{}])[0]
            _cb = _route.get("cost_breakdown") or {}
            print(
                f"  plan_status={j.get('plan_status')} "
                f"dist_m={s.get('total_distance_m')} "
                f"dur_s={s.get('total_duration_s')} "
                f"late_stops={s.get('late_stops')} "
                f"unassigned={s.get('unassigned')}"
            )
            print(
                f"  warp_s={_cb.get('time_warp_s')} "
                f"route_warp_s={_cb.get('route_time_warp_s')} "
                f"warnings={[w.get('code') for w in j.get('warnings', [])]}"
            )
            print(
                f"  emitted_source={m.get('emitted_source')} "
                f"feasible={m.get('feasible')} num_solves={m.get('num_solves')}"
            )

    print("=" * 70)
    print("done")


if __name__ == "__main__":
    main()
