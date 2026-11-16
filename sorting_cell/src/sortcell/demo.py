"""Demo một cảnh của cell phân loại, xem trực tiếp hoặc ghi video.

    ../.venv/bin/python -m sortcell.demo basic            # cảnh thường (seed 1000)
    ../.venv/bin/python -m sortcell.demo clusters         # khối sát nhau (seed 2000)
    ../.venv/bin/python -m sortcell.demo arrivals         # thả thêm 4 khối giữa chừng (seed 3000)
    ../.venv/bin/python -m sortcell.demo clusters --seed 2007 --speed 2
    MUJOCO_GL=egl ../.venv/bin/python -m sortcell.demo arrivals --video arrivals.mp4

Mặc định bộ điều phối dùng pose từ camera trên cao (perception); `--truth` để dùng pose thật của sim.
Lần chạy đầu dựng roadmap và bảng chi phí (vài phút), các lần sau đọc từ cache/.
"""
from __future__ import annotations

import argparse
import os

CASES = {"basic": 1000, "clusters": 2000, "arrivals": 3000}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("case", choices=list(CASES))
    ap.add_argument("--seed", type=int, default=None, help="mặc định: 1000 / 2000 / 3000 theo tình huống")
    ap.add_argument("--speed", type=float, default=1.0, help="tốc độ so với thời gian thực (cửa sổ hoặc video)")
    ap.add_argument("--video", default=None, help="ghi video thay vì mở cửa sổ (cần MUJOCO_GL=egl)")
    ap.add_argument("--truth", action="store_true", help="dùng pose thật thay cho perception")
    a = ap.parse_args()
    seed = a.seed if a.seed is not None else CASES[a.case]
    if a.video:
        os.environ.setdefault("MUJOCO_GL", "egl")
    from .control.video import LiveViewer, Recorder
    from .eval.run_m2 import make_cell
    from .scene.sampling import scene_for_seed

    cubes, arrivals = scene_for_seed(seed)
    cell = make_cell("level1", 0.3, perception=not a.truth)
    rec = Recorder(a.video, size=(960, 720), speed=a.speed) if a.video else LiveViewer(speed=a.speed)
    try:
        r = cell.run(cubes, seed=seed, recorder=rec, arrivals=arrivals)
    except KeyboardInterrupt:
        print("dừng (đóng cửa sổ hoặc Ctrl+C)")
        return
    finally:
        rec.close()
    print(f"cảnh {seed}: {r.placed_correct}/{r.n_cubes} khối vào đúng khay, va chạm {r.collision_steps or 0}, "
          f"gắp {r.pick_success}/{r.picks}, khối bị vây {r.blocked}, thời gian {r.t_last_place:.1f} s "
          f"({r.placed_correct / r.t_last_place * 60:.2f} khối/phút)")
    print("khối mỗi tay:", r.picks_per_arm, " tỉ lệ bận:", [round(b / r.sim_time, 2) for b in r.busy_time])
    for x in r.arrivals:
        print(f"  khối thả thêm {x['cube']}: thả {x['t_spawn']:.1f} s, thấy {x.get('t_detect', float('nan')):.1f} s, "
              f"giao việc {x.get('t_assign', float('nan')):.1f} s, nhấc lên {x.get('t_lift', float('nan')):.1f} s")


if __name__ == "__main__":
    main()
