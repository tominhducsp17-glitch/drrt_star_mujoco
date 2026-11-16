# Cell phân loại vật với 4 tay UR10e

Bốn tay UR10e gắn gripper Robotiq 2F-85 cùng làm việc trên một bàn chung trong MuJoCo: camera RGB-D trên cao tìm các
khối màu, bộ phân việc chia khối cho các tay, planner dRRT\* lập kế hoạch chuyển động cho các tay đang di chuyển mỗi khi có
sự kiện, và mỗi tay gắp khối rồi bỏ vào khay đúng màu của mình. Không có học máy: perception, phân việc và lập kế hoạch đều
là phương pháp cổ điển, có đo đạc. Thiết kế, nhật ký quyết định và mọi số liệu: `../docs/sorting_cell_plan.md`.

Planner dRRT\* là package `drrt` ở thư mục cha (cùng repo, branch `sorting_cell_plan`).

## Chạy thử

```bash
# ở gốc repo: một .venv dùng chung cho drrt và sorting_cell
uv venv -p 3.12 .venv
uv pip install --python .venv/bin/python -e . -e "sorting_cell[dev,video]"
cd sorting_cell
../.venv/bin/python -m pytest -q                       # 26 test
../.venv/bin/python -m sortcell.demo basic             # cảnh thường, mở cửa sổ MuJoCo
../.venv/bin/python -m sortcell.demo clusters          # khối sát nhau
../.venv/bin/python -m sortcell.demo arrivals          # thả thêm 4 khối giữa chừng
```

`--seed` chọn cảnh khác, `--speed 2` chạy nhanh gấp đôi, `--truth` dùng pose thật thay cho camera,
`--video ra.mp4` (kèm `MUJOCO_GL=egl`) ghi video thay vì mở cửa sổ. Lần chạy đầu dựng roadmap và bảng chi phí (vài phút),
các lần sau đọc từ `cache/`. Lúc lập kế hoạch, mô phỏng tạm dừng nên hình đứng yên khoảng nửa giây.

## Kết quả

Mỗi dòng là 50 cảnh cố định theo seed, 12 tiến trình song song. Va chạm = số bước `mj_step` có tiếp xúc lún giữa hai tay,
tay với môi trường, tay với khối không phải khối nó đang làm, hoặc khối đang mang với vật khác.

| Tình huống | Cấu hình | Khối vào đúng khay | Va chạm | Gắp thành công | Khối/phút |
| --- | --- | --- | --- | --- | --- |
| Cảnh thường (seed 1000–) | M1: các tay đi theo lượt, chia bàn theo góc phần tư, pose thật | 400/400 | 0 | 400/400 | 5,56 |
| Cảnh thường | M2: lệch giờ, lập kế hoạch lại theo sự kiện, phân việc theo chi phí, pose thật | 400/400 | 0 | 400/400 | 7,22 |
| Cảnh thường | **M4: như M2, pose từ camera** | **400/400** | **0** | **400/400** | **7,05** |
| Khối sát nhau (seed 2000–) | **M4, pose từ camera** | **374/400** (374/377 khối gắp được về nguyên tắc) | **0** | **374/374** | **6,31** |
| Thả thêm 4 khối giữa chừng (seed 3000–) | **M4, pose từ camera** | **600/600** (gồm 200 khối thả thêm) | **0** | **600/600** | **7,49** |

- **Perception** (camera 640×480 trên cao, HSV + depth, tách khối dính nhau bằng hình học, khớp pose theo biên): sai số
  pose lúc quyết định gắp p99 khoảng 1–1,5 mm và 2°, màu đúng 100%. Đánh giá tĩnh 50 cảnh khối sát nhau: 400/400 khối
  thấy rõ, gồm 83 khối cùng màu chạm nhau.
- **Khối sát nhau:** ngón 2F-85 cần khoảng 4 cm trống để kẹp theo một phương. 23/400 khối nằm trong cụm 2 × 2 hay tam giác
  không gắp được theo thứ tự nào nếu không đẩy (`experiments/graspability.py`, pose thật); cell nhận ra và bỏ qua chúng
  (báo "bị vây") thay vì đâm vào. 3 khối nữa sát biên lề an toàn 5 mm nên cũng bị bỏ qua.
- **So sánh trên cảnh khối sát nhau:** tắt bước tách khối dính nhau → 34.091 bước va chạm tay–khối, gắp 356/391;
  tắt chọn hướng kẹp theo khối hàng xóm → 152.094 bước va chạm, gắp 385/457, 152 lần làm xê dịch khối hàng xóm.
- **Thả thêm khối:** camera thấy khối mới sau trung vị 0,12 s (p95 2,2 s khi bị tay che); bộ điều phối xử lý ở ranh giới
  đoạn đường đi kế tiếp (trung vị 1,4 s).
- **Lập kế hoạch lại:** trung vị khoảng 0,55 s, p99 khoảng 1,4 s (ngân sách tìm kiếm dRRT\* 1 s); thất bại thì huỷ việc
  vừa giao và thử lại sau 1 s mô phỏng.
- Mỗi tay bận khoảng 75–85% thời gian, gắp khoảng 1,5–1,9 khối/phút.

dRRT\* chạy theo ngân sách thời gian nên đường đi phụ thuộc tải máy: số liệu lặp lại được về thống kê, không lặp lại từng
bước.

## Giới hạn đã biết

- Chỉ trong mô phỏng; camera và robot có thông số chính xác.
- Khối bị vây (cụm 2 × 2, tam giác) cần đẩy hoặc gạt trước khi gắp; cell hiện chỉ gắp. Đây là ứng viên cho bài toán có
  học máy tiếp theo (`../docs/sorting_cell_plan.md`, M5).
- Các tay dừng ở mỗi đỉnh của đường đi và ở mỗi lần lập kế hoạch lại (chưa lập kế hoạch lại khi đang chuyển động).
- Planner không biết vị trí từng khối trên bàn; thay vào đó cấm thân tay và vật đang mang đi thấp qua vùng đặt vật.

## Thí nghiệm

| Lệnh (trong `sorting_cell/`) | Ra |
| --- | --- |
| `../.venv/bin/python -m sortcell.eval.run_m2 --perception --no-eval-frames --first-seed 1000 --out results/m4_basic` | 50 cảnh thường, pose từ camera |
| `... --first-seed 2000 --out results/m4_clusters` | 50 cảnh khối sát nhau (`--no-split`, `--no-neighbor-check` để so sánh) |
| `... --first-seed 3000 --out results/m4_arrivals` | 50 cảnh thả thêm khối |
| `../.venv/bin/python -m sortcell.eval.run_m2 --out results/m2_level1_v2` | M2, pose thật (`--alloc level0` để so sánh) |
| `../.venv/bin/python -m sortcell.eval.run_cell --scenes 50` | M1: lập kế hoạch theo lượt |
| `MUJOCO_GL=egl ../.venv/bin/python experiments/perception_clusters.py` | đánh giá tĩnh perception trên khối sát nhau |
| `../.venv/bin/python experiments/graspability.py` | khối nào gắp được về nguyên tắc |
| `experiments/hitbox_accuracy.py`, `experiments/allocation_reference.py` | hitbox elip và CP-SAT (phân việc) |
| `experiments/grasp_tolerance.py` | tỉ lệ gắp của script theo sai số pose |
| `experiments/grasp_test.py`, `reach_check.py`, `render_cameras.py`, `build_roadmaps.py` | kiểm tra gripper, tầm với, camera, roadmap (M0) |

Bỏ `--out` thì chỉ in kết quả, không ghi file. `--no-eval-frames` tắt việc ghi sai số và độ che khuất ở mỗi khung hình
(nhanh hơn khoảng 3 lần). Các thư mục `results/m2_level1*`, `m2_level0`, `m3_perception`, `m4_clusters_no*` là lượt chạy
bằng code hoặc roadmap cũ, giữ lại để đối chiếu với nhật ký quyết định.

## Mã nguồn

```
src/sortcell/
  scene/       builder.py (cell, gripper, camera, vùng cấm bay), sampling.py (cảnh thường, khối sát nhau, thả thêm)
  planning/    models.py (tách va chạm, khối ảo), roadmaps.py (roadmap + đỉnh tạm), query.py (tay cố định, vùng
               dành riêng), service.py (dRRT* có giới hạn ngân sách + rút gọn đường)
  perception/  overhead.py (RGB-D → khối: HSV, dải độ cao, tách khối dính nhau, khớp biên), tracker.py (bản ghi vật)
  skills/      ik.py, scripted.py (gắp/thả bằng script)
  allocation/  zones.py (chia góc phần tư), costs.py (bảng chi phí), hitbox.py, greedy.py (theo chi phí), milp.py (CP-SAT)
  control/     executor.py (mj_step, đo va chạm), arm_fsm.py (theo lượt), staggered.py (lệch giờ, sự kiện, thứ tự
               gắp, khối thả thêm), world.py (pose thật hoặc từ camera), video.py (ghi video, cửa sổ xem trực tiếp)
  eval/        run_cell.py, run_m2.py, metrics.py
  demo.py      demo một cảnh
```

`third_party/robotiq_2f85`: model từ MuJoCo Menagerie, giữ nguyên (xem SOURCE.md, LICENSE).
