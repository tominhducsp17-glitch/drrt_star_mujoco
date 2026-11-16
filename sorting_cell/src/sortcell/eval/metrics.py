"""Tổng hợp kết quả nhiều cảnh theo spec mục 2."""
from __future__ import annotations

import numpy as np


def summarize(results: list[dict]) -> dict:
    n = len(results)
    thr = np.array([r["placed_correct"] / r["t_last_place"] * 60 for r in results if r["t_last_place"] > 0])
    thr_wall = np.array([r["placed_correct"] / (r["t_last_place"] + r["planning_wall"]) * 60 for r in results
                         if r["t_last_place"] > 0])
    coll = {}
    for r in results:
        for k, v in r["collision_steps"].items():
            coll[k] = coll.get(k, 0) + v
    picks = sum(r["picks"] for r in results)
    return {
        "scenes": n,
        "scenes_fully_sorted": sum(r["placed_correct"] == r["n_cubes"] for r in results),
        "cubes": sum(r["n_cubes"] for r in results),
        "placed_correct": sum(r["placed_correct"] for r in results),
        "placed_wrong": sum(r["placed_wrong"] for r in results),
        "not_in_tray": sum(r["not_in_tray"] for r in results),
        "scenes_with_collision": sum(sum(r["collision_steps"].values()) > 0 for r in results),
        "collision_steps_by_type": coll,
        "picks": picks,
        "pick_success": sum(r["pick_success"] for r in results),
        "pick_success_rate": sum(r["pick_success"] for r in results) / max(picks, 1),
        "throughput_per_min_sim": {"mean": float(thr.mean()), "median": float(np.median(thr)), "min": float(thr.min()),
                                   "max": float(thr.max())},
        "throughput_per_min_incl_planning": {"mean": float(thr_wall.mean()), "median": float(np.median(thr_wall))},
        "makespan_s": {"mean": float(np.mean([r["t_last_place"] for r in results])),
                       "max": float(np.max([r["t_last_place"] for r in results]))},
        "planning_wall_s_per_scene": {"mean": float(np.mean([r["planning_wall"] for r in results])),
                                      "max": float(np.max([r["planning_wall"] for r in results]))},
        "rounds": {"mean": float(np.mean([r["rounds"] for r in results])), "max": int(max(r["rounds"] for r in results))},
        "plan_failures": sum(r.get("plan_failures", 0) for r in results),
        "deferrals": sum(r.get("deferrals", 0) for r in results),
        "deadlocks": sum(r["deadlock"] for r in results),
        "max_tracking_error_rad": float(max(r["max_tracking_error"] for r in results)),
        "blocked": sum(r.get("blocked", 0) for r in results),
        "disturbed_events": sum(len(r.get("disturbed", [])) for r in results),
        "disturbed_max_mm": float(max((e["mm"] for r in results for e in r.get("disturbed", [])), default=0.0)),
        "arrivals": arrival_summary(results),
        "per_arm": per_arm_summary(results),
    }


def per_arm_summary(results: list[dict]) -> dict | None:
    """Mỗi tay: tỉ lệ thời gian không rảnh trên toàn thời gian mô phỏng của cảnh, và số khối gắp được; trung bình các cảnh."""
    rs = [r for r in results if r.get("busy_time")]
    if not rs:
        return None
    util = np.array([np.array(r["busy_time"]) / max(r["sim_time"], 1e-9) for r in rs])
    picks = np.array([r["picks_per_arm"] for r in rs])
    return {"utilization": util.mean(0).round(3).tolist(), "picks_per_scene": picks.mean(0).round(2).tolist(),
            "picks_per_min_per_arm": (picks.sum(0) / sum(r["t_last_place"] for r in rs) * 60).round(2).tolist()}


def arrival_summary(results: list[dict]) -> dict | None:
    """Khối thả thêm giữa chừng (M4 việc 2): số khối, độ trễ thả → phát hiện → giao việc → nhấc lên (s)."""
    arr = [a for r in results for a in r.get("arrivals", [])]
    if not arr:
        return None
    out = {"n": len(arr), "detected": sum("t_detect" in a for a in arr), "lifted": sum("t_lift" in a for a in arr)}
    for name, a_key, b_key in (("spawn_to_detect", "t_spawn", "t_detect"), ("spawn_to_assign", "t_spawn", "t_assign"),
                               ("detect_to_assign", "t_detect", "t_assign"), ("spawn_to_lift", "t_spawn", "t_lift")):
        v = [a[b_key] - a[a_key] for a in arr if a_key in a and b_key in a]
        out[name] = _pct(v, (50, 95, 100)) if v else {"n": 0}
    return out


def _pct(x, qs=(50, 95, 99, 100)) -> dict:
    x = np.asarray(x, float)
    if x.size == 0:
        return {"n": 0}
    return {"n": int(x.size), **{f"p{q}": float(np.percentile(x, q)) for q in qs}, "mean": float(x.mean())}


def perception_summary(results: list[dict]) -> dict:
    """Sai số pose (mục 5.1, đánh giá riêng): tại lúc quyết định gắp, và ở mọi khung hình khi cell đang chạy; tỉ lệ khối
    bị che (theo ảnh phân đoạn) khi cell đang chạy."""
    tasks = [t for r in results for t in r.get("tasks", []) if "err_xy" in t]
    frames = [f for r in results for f in (r.get("perception") or {}).get("frames", [])]
    trk = [tr for f in frames for tr in f["tracks"] if tr.get("truth") is not None]
    vis = np.array([c["vis"] for f in frames for c in f["cubes"]])
    return {
        "decision_err_xy_mm": _pct([t["err_xy"] * 1000 for t in tasks]),
        "decision_err_yaw_deg": _pct([np.degrees(t["err_yaw"]) for t in tasks]),
        "decision_color_ok": float(np.mean([t["color_ok"] for t in tasks])) if tasks else None,
        "frame_err_xy_mm": _pct([tr["err_xy"] * 1000 for tr in trk]),
        "frame_err_yaw_deg": _pct([np.degrees(tr["err_yaw"]) for tr in trk]),
        "frame_track_age_s": _pct([tr["age"] for tr in trk]),
        "tracks_without_truth": int(sum(tr.get("truth") is None for f in frames for tr in f["tracks"])),
        "track_frames": len(trk),
        "frames": len(frames),
        "cube_frames": int(vis.size),
        "occluded_partly": float(np.mean(vis < 0.85)) if vis.size else None,
        "occluded_fully": float(np.mean(vis == 0)) if vis.size else None,
        "tracks_per_scene": _pct([len((r.get("perception") or {}).get("tracks", [])) for r in results], (50, 100)),
        "lost_tracks": int(sum(t["state"] == "lost" for r in results for t in (r.get("perception") or {}).get("tracks", []))),
        "lift_disagree": int(sum(r.get("lift_disagree", 0) for r in results)),
    }
