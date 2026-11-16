"""Đánh giá tĩnh perception trên cảnh có khối sát nhau (M4 việc 1): các tay ở HOME, một ảnh mỗi cảnh, so với pose thật.

Ghép phát hiện với khối thật cùng màu gần nhất trong 2,5 cm (gán tham lam theo khoảng cách). Mỗi khối thật rơi vào một
trong ba loại: thấy rõ (phát hiện "full", được dùng để gắp), không chắc (phát hiện bị đánh dấu không thấy rõ: chưa gắp),
bỏ sót. Phát hiện không ghép được với khối nào là phát hiện thừa. Báo riêng các khối nằm trong vùng phải tách (khối cùng
màu chạm nhau) và lưu ảnh các cảnh có lỗi.

    MUJOCO_GL=egl ../.venv/bin/python experiments/perception_clusters.py
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import cv2  # noqa: E402
import mujoco as mj  # noqa: E402
import numpy as np  # noqa: E402

from sortcell.perception.overhead import OverheadCamera  # noqa: E402
from sortcell.perception.tracker import yaw_diff  # noqa: E402
from sortcell.scene.builder import CUBE, Layout, build_spec  # noqa: E402
from sortcell.scene.sampling import sample_cluster_scene, squares_gap  # noqa: E402
from sortcell.skills.ik import HOME, ArmKin  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "results/m4"
GATE = 0.025


def touching_same_color(cubes) -> set[int]:
    s = set()
    for i, a in enumerate(cubes):
        for j, b in enumerate(cubes):
            if i < j and a.color == b.color and squares_gap((a.x, a.y, a.yaw), (b.x, b.y, b.yaw)) < 0.004:
                s |= {i, j}
    return s


def evaluate(seed: int, L: Layout, save_dir: Path | None) -> dict:
    cubes = sample_cluster_scene(seed, L)
    m = build_spec(L, "exec", cubes=[c.as_tuple(L) for c in cubes]).compile()
    d = mj.MjData(m)
    for k in range(L.n_arms):
        d.qpos[ArmKin(m, k).idx] = HOME
    mj.mj_forward(m, d)
    cam = OverheadCamera(m, L)
    rgb, dep = cam.capture(d)
    dets, _ = cam.detect(d, rgb, dep)
    cam.close()
    pairs = sorted((float(np.hypot(c.x - det.xy[0], c.y - det.xy[1])), i, j)
                   for i, c in enumerate(cubes) for j, det in enumerate(dets) if det.color == c.color)
    match, used = {}, set()
    for dist, i, j in pairs:
        if dist > GATE or i in match or j in used:
            continue
        match[i] = j
        used.add(j)
    merged = touching_same_color(cubes)
    rows = []
    for i, c in enumerate(cubes):
        j = match.get(i)
        if j is None:
            rows.append({"i": i, "status": "missed", "merged": i in merged})
            continue
        det = dets[j]
        rows.append({"i": i, "status": "full" if det.full else "uncertain", "merged": i in merged, "split": det.split,
                     "err_mm": float(np.hypot(c.x - det.xy[0], c.y - det.xy[1]) * 1000),
                     "err_deg": float(np.degrees(abs(yaw_diff(det.yaw, c.yaw))))})
    extra = [j for j in range(len(dets)) if j not in used]
    bad = any(r["status"] != "full" for r in rows) or extra or any(r.get("err_mm", 0) > 3 for r in rows)
    if bad and save_dir is not None:
        img = cv2.resize(rgb, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST).copy()
        z = L.table_top + CUBE
        for det in dets:
            R = np.array([[np.cos(det.yaw), -np.sin(det.yaw)], [np.sin(det.yaw), np.cos(det.yaw)]])
            pts = []
            for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
                p = det.xy + R @ (np.array([sx, sy]) * CUBE / 2)
                u, v = cam.project(d, [p[0], p[1], z])
                pts.append((2 * u + 1, 2 * v + 1))
            cv2.polylines(img, [np.array(pts, np.int32)], True, (255, 255, 0) if det.full else (255, 128, 0), 2)
        for c in cubes:
            u, v = cam.project(d, [c.x, c.y, z])
            cv2.drawMarker(img, (2 * u + 1, 2 * v + 1), (0, 0, 0), cv2.MARKER_CROSS, 12, 2)
        save_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(save_dir / f"scene_{seed}.png"), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    return {"seed": seed, "cubes": rows, "extra": len(extra)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", type=int, default=50)
    ap.add_argument("--first-seed", type=int, default=2000)
    ap.add_argument("--out", default=str(OUT / "perception_clusters.json"))
    ap.add_argument("--save-failures", action="store_true")
    a = ap.parse_args()
    L = Layout()
    res = [evaluate(s, L, OUT / "perception_failures" if a.save_failures else None)
           for s in range(a.first_seed, a.first_seed + a.scenes)]
    rows = [r for s in res for r in s["cubes"]]

    def stats(sel):
        n = len(sel)
        full = [r for r in sel if r["status"] == "full"]
        return {"n": n, "full": len(full), "uncertain": sum(r["status"] == "uncertain" for r in sel),
                "missed": sum(r["status"] == "missed" for r in sel),
                "err_mm_p50": float(np.median([r["err_mm"] for r in full])) if full else None,
                "err_mm_p99": float(np.percentile([r["err_mm"] for r in full], 99)) if full else None,
                "err_mm_max": float(max(r["err_mm"] for r in full)) if full else None,
                "err_deg_p99": float(np.percentile([r["err_deg"] for r in full], 99)) if full else None,
                "err_deg_max": float(max(r["err_deg"] for r in full)) if full else None}

    out = {"scenes": len(res), "all": stats(rows), "merged_same_color": stats([r for r in rows if r["merged"]]),
           "other": stats([r for r in rows if not r["merged"]]), "extra_detections": sum(s["extra"] for s in res),
           "scenes_all_full": sum(all(r["status"] == "full" for r in s["cubes"]) and not s["extra"] for s in res),
           "per_scene": res}
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(out, indent=1))
    print(json.dumps({k: v for k, v in out.items() if k != "per_scene"}, indent=1))


if __name__ == "__main__":
    main()
