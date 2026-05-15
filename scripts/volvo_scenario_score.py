#!/usr/bin/env python3
"""Scenario-based offline scorer for Volvo longitudinal behavior.

Scenarios:
1) no-lead red stop
2) no-lead green takeoff
3) low-speed engage no lead
4) lead cut-in near standstill

Reads local route segments copied from comma realdata.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import argparse

from openpilot.tools.lib.logreader import LogReader


FSM0_ADDR = 81
FSM1_ADDR = 577
FSM3_ADDR = 624


@dataclass
class Frame:
  t: float
  seg: int
  v_ego: Optional[float] = None
  cruise_enabled: Optional[bool] = None
  cruise_available: Optional[bool] = None
  cruise_standstill: Optional[bool] = None
  acc_distance: Optional[int] = None
  lead_status: Optional[bool] = None
  lead_drel: Optional[float] = None
  lead_vrel: Optional[float] = None
  plan_a: Optional[float] = None
  should_stop: Optional[bool] = None
  gas_pressed: Optional[bool] = None
  brake_pressed: Optional[bool] = None


def iter_route(route_dir: Path) -> Tuple[List[Frame], Dict[int, Dict[int, int]], List[Tuple[float, int]]]:
  seg_dirs = sorted([p for p in route_dir.iterdir() if p.is_dir() and "--" in p.name], key=lambda p: int(p.name.rsplit("--", 1)[-1]))
  t0 = None
  rows: List[Frame] = []
  last = Frame(t=0.0, seg=-1)
  src_counts: Dict[int, Dict[int, int]] = {FSM0_ADDR: {}, FSM3_ADDR: {}}
  dist_events: List[Tuple[float, int]] = []
  prev_dist: Optional[int] = None

  for seg_dir in seg_dirs:
    seg = int(seg_dir.name.rsplit("--", 1)[-1])
    rlog = seg_dir / "rlog.zst"
    if not rlog.exists():
      continue

    for m in LogReader(str(rlog)):
      if t0 is None:
        t0 = m.logMonoTime / 1e9
      t = m.logMonoTime / 1e9 - t0
      w = m.which()

      if w == "carState":
        last.v_ego = m.carState.vEgo
        last.cruise_enabled = bool(m.carState.cruiseState.enabled)
        last.cruise_available = bool(m.carState.cruiseState.available)
        last.cruise_standstill = bool(m.carState.cruiseState.standstill)
        last.gas_pressed = bool(m.carState.gasPressed)
        last.brake_pressed = bool(m.carState.brakePressed)
      elif w == "radarState":
        lo = m.radarState.leadOne
        last.lead_status = bool(lo.status)
        if lo.status:
          last.lead_drel = lo.dRel
          last.lead_vrel = lo.vRel
        else:
          last.lead_drel = None
          last.lead_vrel = None
      elif w == "longitudinalPlan":
        accels = list(m.longitudinalPlan.accels) if hasattr(m.longitudinalPlan, "accels") else []
        last.plan_a = accels[0] if accels else None
        last.should_stop = bool(getattr(m.longitudinalPlan, "shouldStop", False))
      elif w == "can":
        for c in m.can:
          if c.address in (FSM0_ADDR, FSM3_ADDR):
            src_counts[c.address][c.src] = src_counts[c.address].get(c.src, 0) + 1
          if c.address == FSM1_ADDR and len(c.dat) == 8:
            dist = int(c.dat[0])
            last.acc_distance = dist
            if prev_dist is None or dist != prev_dist:
              dist_events.append((t, dist))
              prev_dist = dist

      rows.append(Frame(**{**last.__dict__, "t": t, "seg": seg}))

  return rows, src_counts, dist_events


def find_stops(rows: List[Frame], min_stationary_s: float = 2.0) -> List[Tuple[float, float, int]]:
  stops: List[Tuple[float, float, int]] = []
  in_stop = False
  start_t = 0.0
  start_seg = -1

  for r in rows:
    v = r.v_ego
    if v is None:
      continue
    if (not in_stop) and v < 0.25:
      in_stop = True
      start_t = r.t
      start_seg = r.seg
    elif in_stop and v > 1.0:
      end_t = r.t
      in_stop = False
      if end_t - start_t >= min_stationary_s:
        stops.append((start_t, end_t, start_seg))

  return stops


def window(rows: List[Frame], t0: float, t1: float) -> List[Frame]:
  return [r for r in rows if t0 <= r.t <= t1]


def pct(cond_count: int, total: int) -> float:
  return 0.0 if total == 0 else (100.0 * cond_count / total)


def scenario_no_lead_red_stop(rows: List[Frame], stops: List[Tuple[float, float, int]]) -> None:
  candidates = []
  for s, e, seg in stops:
    w = window(rows, s - 6.0, s + 0.5)
    if not w:
      continue
    no_lead = sum(1 for r in w if r.lead_status is False)
    stop_intent = sum(1 for r in w if r.should_stop is True or (r.plan_a is not None and r.plan_a < -0.4))
    if no_lead > len(w) * 0.8 and stop_intent > len(w) * 0.4:
      candidates.append((s, e, seg, len(w), no_lead, stop_intent))

  print("\n[Scenario 1] no-lead red stop")
  if not candidates:
    print("  FAIL: no qualifying events found")
    return

  for i, (s, e, seg, n, no_lead, stop_intent) in enumerate(candidates, 1):
    print(f"  Event {i}: seg={seg} t={s:.1f}->{e:.1f}s dur={e-s:.1f}s no_lead={pct(no_lead,n):.1f}% stop_intent={pct(stop_intent,n):.1f}%")
  print("  PASS: qualifying no-lead stop events present for offline validation")


def scenario_no_lead_green_takeoff(rows: List[Frame], stops: List[Tuple[float, float, int]]) -> None:
  print("\n[Scenario 2] no-lead green takeoff")
  found = 0
  for s, e, seg in stops:
    post = window(rows, e - 0.5, e + 4.0)
    if not post:
      continue
    no_lead = sum(1 for r in post if r.lead_status is False)
    go_intent = sum(1 for r in post if r.plan_a is not None and r.plan_a > 0.2)
    moving = next((r for r in post if (r.v_ego or 0) > 1.5), None)
    if no_lead > len(post) * 0.8 and go_intent > 5:
      found += 1
      move_t = moving.t if moving is not None else None
      print(f"  Event {found}: seg={seg} stop_end={e:.1f}s go_intent_samples={go_intent} move_t={move_t}")
  if found == 0:
    print("  FAIL: no no-lead takeoff opportunities with positive planner intent")
  else:
    print("  PASS: found no-lead takeoff opportunities for resume/coherence checks")


def scenario_low_speed_engage_no_lead(rows: List[Frame]) -> None:
  print("\n[Scenario 3] low-speed engage no lead")
  candidates = [r for r in rows if r.v_ego is not None and 1.0 < r.v_ego < 11.0 and r.cruise_available is True and r.lead_status is False]
  enabled = [r for r in candidates if r.cruise_enabled is True]
  print(f"  candidate_frames={len(candidates)} enabled_frames={len(enabled)} enabled_ratio={pct(len(enabled), len(candidates)):.1f}%")
  if len(candidates) < 50:
    print("  FAIL: insufficient low-speed no-lead candidate data")
  elif len(enabled) == 0:
    print("  FAIL: never observed ACC enabled in low-speed no-lead window")
  else:
    print("  PASS: low-speed no-lead engagement opportunities exist")


def scenario_lead_cutin_near_standstill(rows: List[Frame]) -> None:
  print("\n[Scenario 4] lead cut-in near standstill")
  near_stop = [r for r in rows if (r.v_ego is not None and r.v_ego < 2.0)]
  transitions = 0
  for i in range(1, len(near_stop)):
    a = near_stop[i - 1]
    b = near_stop[i]
    if a.lead_status is False and b.lead_status is True:
      transitions += 1
  print(f"  near_stop_frames={len(near_stop)} lead_appearance_transitions={transitions}")
  if transitions == 0:
    print("  FAIL: no lead cut-in/appearance events near standstill")
  else:
    print("  PASS: lead cut-in transitions exist for coherence checks")


def summarize_bus_source_counts(src_counts: Dict[int, Dict[int, int]]) -> None:
  print("\n[Coherence] FSM source distribution")
  for addr in (FSM0_ADDR, FSM3_ADDR):
    by_src = src_counts.get(addr, {})
    total = sum(by_src.values())
    parts = ", ".join([f"src{src}:{cnt}" for src, cnt in sorted(by_src.items(), key=lambda x: (-x[1], x[0]))])
    print(f"  addr={addr} total={total} {parts}")


def summarize_distance_events(dist_events: List[Tuple[float, int]], center_t: Optional[float]) -> None:
  print("\n[Coherence] ACC distance-wheel events")
  print(f"  total_changes={len(dist_events)}")
  if center_t is None:
    return
  lo = center_t - 20.0
  hi = center_t + 20.0
  nearby = [(t, d) for t, d in dist_events if lo <= t <= hi]
  print(f"  around_t={center_t:.1f}s window=[{lo:.1f},{hi:.1f}] events={len(nearby)}")
  for t, d in nearby[:20]:
    print(f"    t={t:.2f} dist={d}")


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument("--route", default="/Users/astroianu/personal/sunnypilot/routes/00000071--daba433334")
  ap.add_argument("--focus-t", type=float, default=780.0, help="focus timestamp in route seconds (default ~= 12:11 local event)")
  args = ap.parse_args()

  route = Path(args.route)
  rows, src_counts, dist_events = iter_route(route)
  if not rows:
    raise SystemExit("No rows parsed; verify route path and rlogs")

  print(f"route={route}")
  print(f"frames={len(rows)} t_end={rows[-1].t:.1f}s")

  stops = find_stops(rows)
  print(f"stops={len(stops)}")

  scenario_no_lead_red_stop(rows, stops)
  scenario_no_lead_green_takeoff(rows, stops)
  scenario_low_speed_engage_no_lead(rows)
  scenario_lead_cutin_near_standstill(rows)
  summarize_bus_source_counts(src_counts)
  summarize_distance_events(dist_events, args.focus_t)


if __name__ == "__main__":
  main()
