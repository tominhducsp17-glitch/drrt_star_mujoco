# dRRT\* trên MuJoCo

Cài đặt dRRT\* (Shome et al. 2019, arXiv:1903.00994) cho nhiều tay máy UR10e trong MuJoCo. Kế hoạch và tiến độ: `PLAN.md`.

Bài báo: <https://arxiv.org/abs/1903.00994>. Repo không kèm file PDF; tải về `paper/` nếu cần.

```bash
uv venv -p 3.12 .venv
uv pip install --python .venv/bin/python -e ".[dev]"
.venv/bin/python -m pytest -q        # khoảng 2–3 phút
```

- `src/drrt/`: roadmap, đồ thị tensor ngầm, cây, planner (dRRT, ao-dRRT, dRRT\*, Implicit A\*). Planner không phụ thuộc MuJoCo.
- `experiments/`: tái hiện thí nghiệm của bài; kết quả ở `results/`.
- `docs/`: ghi chú đọc bài, khảo sát robot, kết quả từng giai đoạn.
