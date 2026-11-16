# Cell phân loại vật với 4 tay UR10e

Bốn tay UR10e gắn Robotiq 2F-85 phân loại khối màu trên một bàn chung, trong MuJoCo. Thiết kế và kế hoạch:
`../docs/sorting_cell_plan.md`. Planner dRRT\* là package `drrt` ở thư mục cha (cùng repo, branch `sorting_cell_plan`).

## Cài

```bash
# ở gốc repo: một .venv dùng chung cho drrt và sorting_cell
uv venv -p 3.12 .venv
uv pip install --python .venv/bin/python -e . -e "sorting_cell[dev,video]"
cd sorting_cell && ../.venv/bin/python -m pytest -q
```

## Thí nghiệm

| Script | Ra |
| --- | --- |
| `experiments/measure_gripper.py` | khe má kẹp theo lệnh, vị trí TCP |
| `experiments/grasp_test.py --n 20 --shake` | thử kẹp 20 lần (nghiệm thu M0), `results/grasp_repo/` |
| `experiments/reach_check.py` | bản đồ số tay với tới, khay, `results/reach/` |
| `MUJOCO_GL=egl experiments/render_cameras.py` | ảnh camera trên cao và cổ tay, `results/cameras/` |
| `experiments/build_roadmaps.py` | dựng và cache roadmap 4 tay (`cache/`) |

## Chạy cell

| Lệnh | Mốc |
| --- | --- |
| `../.venv/bin/python -m sortcell.eval.run_cell --scenes 50` | M1: lập kế hoạch theo lượt, phân việc mức 0, `results/m1/` |
| `../.venv/bin/python -m sortcell.eval.run_m2 --alloc level1 --out results/m2_level1_v2` | M2: lệch giờ, lập kế hoạch lại theo sự kiện (bản cuối) |
| `MUJOCO_GL=egl ../.venv/bin/python -m sortcell.eval.run_m2 --alloc level1 --perception --out results/m3_perception` | M3: pose từ camera trên cao thay cho pose thật |
| `../.venv/bin/python -m sortcell.eval.run_m2 --alloc level0` | M2 với phân việc mức 0 (so sánh) |
| `experiments/allocation_reference.py` | CP-SAT (mô hình Oberthür et al.) so với mức 0 / mức 1, `results/allocation/` |
| `experiments/hitbox_accuracy.py` | hitbox elip so với va chạm thật giữa hai việc, `results/allocation/hitbox_accuracy.json` |
| `experiments/grasp_tolerance.py` | tỉ lệ gắp thành công của script theo sai số pose, `results/m3/grasp_tolerance.json` |
| `MUJOCO_GL=egl experiments/render_perception.py` | ảnh phát hiện khối khi tay che bàn, `results/m3/` |
| `experiments/summarize.py results/<dir>` | tổng hợp lại summary.json từ các scene_*.json |

Xem trực tiếp một cảnh trong cửa sổ MuJoCo: `../.venv/bin/python -m sortcell.eval.run_m2 --only 1000 --view` (`--view 2` = nhanh gấp
đôi; thêm `--perception` để dùng pose từ camera). Ghi video: `--only 1000 --video out.mp4` (và `MUJOCO_GL=egl`). dRRT\* chạy theo ngân sách thời gian nên
đường đi phụ thuộc tốc độ máy: số liệu lặp lại được về thống kê, không lặp lại từng bước.

`results/m2_level1`, `m2_level1_cap1`, `m2_level1_cap2`, `m2_level0`, `m2_level1_pen10` là các lượt chạy bằng code cũ
(trước khi đếm `held_env` ở M2 và trước khi sửa lỗi treo sau thất bại), giữ lại để đối chiếu với nhật ký quyết định.

## Mã nguồn

```
src/sortcell/
  scene/       builder.py (cell, gripper, camera), sampling.py (cảnh theo seed)
  planning/    models.py (tách va chạm, khối ảo), roadmaps.py (roadmap + đỉnh tạm), query.py (tay cố định, vùng
               dành riêng), service.py (PlannerService: dRRT* + rút gọn đường)
  perception/  overhead.py (RGB-D → khối: HSV, dải độ cao, minAreaRect), tracker.py (bản ghi vật, che khuất)
  skills/      ik.py, scripted.py (gắp/thả bằng script)
  allocation/  zones.py (mức 0), costs.py (bảng chi phí), hitbox.py, greedy.py (mức 1), milp.py (CP-SAT)
  control/     executor.py (mj_step, đo va chạm), arm_fsm.py (M1 theo lượt), staggered.py (M2/M3 lệch giờ),
               world.py (pose thật hoặc pose từ camera cho bộ điều phối)
  eval/        run_cell.py (M1), run_m2.py (M2), metrics.py
```

`third_party/robotiq_2f85`: model từ MuJoCo Menagerie, giữ nguyên (xem SOURCE.md, LICENSE).
