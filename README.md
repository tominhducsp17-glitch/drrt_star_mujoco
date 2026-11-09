# dRRT\* trên MuJoCo

Cài đặt dRRT\* (Shome et al. 2019, arXiv:1903.00994) cho nhiều tay máy UR10e trong MuJoCo. Kế hoạch và tiến độ: `PLAN.md`.

Bài báo: <https://arxiv.org/abs/1903.00994>. Repo không kèm file PDF; tải về `paper/` nếu cần.

```bash
uv venv -p 3.12 .venv
uv pip install --python .venv/bin/python -e ".[dev,video]"
.venv/bin/python -m pytest -q        # khoảng 2–3 phút
```

- `src/drrt/`: roadmap, đồ thị tensor ngầm, cây, planner (dRRT, ao-dRRT, dRRT\*, Implicit A\*, RRT\* hợp). Planner không phụ thuộc MuJoCo.
- `src/drrt/mujoco_cell.py`, `cell_tasks.py`, `ik.py`, `execute.py`: cell UR10e, tác vụ, IK, thực thi bằng `mj_step`.
- `experiments/`: tái hiện thí nghiệm của bài; kết quả ở `results/`.
- `docs/`: ghi chú đọc bài, khảo sát robot, kết quả từng giai đoạn.

Kết quả chính: dRRT\* giải tác vụ 4 tay UR10e (24 chiều) trong vài chục ms, chạy được trong `mj_step` không va chạm, trong khi
Implicit A\* và RRT\* không giải được trong 120 s. Video: `results/p4/crossing2_demo.mp4`, `results/p5/rotate4_demo.mp4`.
