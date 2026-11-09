# Kế hoạch implement dRRT* trên MuJoCo

**Bài báo:** Shome, Solovey, Dobson, Halperin, Bekris. *dRRT\*: Scalable and Informed Asymptotically-Optimal
Multi-Robot Motion Planning.* arXiv:1903.00994 (`paper/`). Tóm tắt và các chỗ bài viết chưa rõ: `docs/paper_notes.md`.

**Mục tiêu:** cài dRRT, ao-dRRT, dRRT\* cùng các baseline; tái hiện xu hướng kết quả của bài; chạy trên cell nhiều tay
máy công nghiệp trong MuJoCo, tính cả bước thực thi quỹ đạo bằng mô phỏng động lực học.

**Robot: đã chốt UR10e** từ MuJoCo Menagerie (lý do: `docs/robot_survey.md`). Nếu sau này có yêu cầu đổi robot thì xử lý
ở giai đoạn dự phòng P6; planner không phụ thuộc robot nên việc đổi chỉ gồm model và cấu hình.

Các tay gắn đế cố định và độc lập về động học, đúng trường hợp dRRT\* cơ bản (Mục 5 của bài). **da-dRRT\* (Mục 7) ngoài
phạm vi**, vì nó dành cho robot có khớp dùng chung.

**Tiến độ:** P0, P1 (`docs/p1_results.md`), P2 (`docs/p2_results.md`) xong. Tiếp theo: P3.

## 1. Số liệu hiện có

**Đo trên UR10e** (`experiments/p0_ur10e_probe.py`; MuJoCo 3.17, một luồng, 20.000 mẫu, khớp ±π, có sàn):

| Cell | µs/lần kiểm tra | Cấu hình tự do | Tự va chạm | Va sàn | Va giữa các tay |
| --- | --- | --- | --- | --- | --- |
| 1 tay | 4,0 | 37,2% | 15,8% | 55,4% | |
| 2 tay, đế cách 1,4 m | 7,2 | 14,1% | 28,6% | 79,9% | 0,4% |
| 2 tay, đế cách 1,0 m | 7,1 | 13,6% | 28,6% | 79,9% | 2,3% |
| 4 tay, đế cách 1,4 m | 14,0 | 1,7% | 48,7% | 96,0% | 9,3% |

Hệ quả cho P3:
- Sàn loại hơn nửa số mẫu của mỗi tay: lấy mẫu từ chối vẫn đủ nhanh, nhưng nên kê đế lên bệ để tăng không gian làm việc.
- 2 tay rất ít va nhau ngay cả ở 1,0 m, nên cần đế gần hơn hoặc tác vụ tập trung vào vùng chung. Tỉ lệ này tính trên mẫu ngẫu nhiên, chưa phải trên tác vụ.
- Khi ghép cell bằng `MjSpec.attach`, integrator của model cha được giữ lại: phải tự đặt `implicitfast` trước khi chạy `mj_step`.

**Tính theo Theorem 1** (n=1000, d=6, chưa trừ hiệu ứng biên): UR10e thu về ±π có r\* ≈ 3,09 rad, khoảng 74 láng giềng mỗi đỉnh,
tức 5,4·10³ tổ hợp với R=2 và 2,9·10⁷ với R=4.

Máy: 18 luồng, 30 GB RAM, không có GPU rời. Chạy thí nghiệm song song theo seed là đủ, thuật toán không cần GPU.

## 2. Quyết định thiết kế

| Vấn đề | Chọn | Lý do |
| --- | --- | --- |
| Ngôn ngữ | Python + NumPy; chỉ tối ưu (numba/C++) khi profile cho thấy cần | Thuật toán có nhiều chi tiết chưa rõ, cần lặp nhanh. Số đo thời gian tuyệt đối sẽ không so được với bài |
| Tách thuật toán khỏi mô phỏng và robot | `Roadmap`, `CollisionChecker`, cấu hình robot là interface; planner không import `mujoco` | Test trên đĩa 2D bằng checker giải tích; đổi UR10e sang FANUC không phải sửa planner |
| Lọc va chạm | Dùng mask `contype`/`conaffinity` thành hai cấu hình: **offline** (một tay + vật cản) và **online** (chỉ giữa các tay) | Đúng tinh thần bài: va chạm với môi trường đã xử lý trong roadmap, online chỉ kiểm giữa các robot |
| Giới hạn khớp UR10e | Thu từ ±2π về ±π | Cạnh ngắn hơn và không trùng lặp tư thế vật lý (q và q+2π). Xem `docs/robot_survey.md` |
| Heuristic H | Dijkstra ngược từ T_i trên mỗi G_i | Bài dùng Johnson all-pairs; khi mục tiêu cố định thì Dijkstra đủ và rẻ hơn |
| Roadmap | Hỗ trợ cả theo bán kính r\*(n) và k-PRM\* (k≈22 ở n=1000) | r\*(n) là cái Theorem 1 chứng minh, nhưng dày. k-PRM\* nhẹ hơn nhưng **không nằm trong chứng minh của bài**, phải ghi rõ khi báo cáo |
| Hàm chi phí | Tổng độ dài cung (mặc định) và độ dài Euclid hợp | Hai loại bài dùng trong thí nghiệm. Bỏ max: không cộng dồn được theo cạnh, cần cây lưu vector cost, và bài không dùng trong thí nghiệm |

## 3. Cấu trúc dự kiến

```
drrt_star_mujoco/
├── PLAN.md, README.md, pyproject.toml
├── paper/, docs/
├── third_party/                    # UR10e từ Menagerie (kèm SOURCE.md, giữ license gốc)
├── robots/                         # cấu hình: file model, tên khớp, giới hạn dùng cho planning
├── scenes/                         # cell 2 và 4 tay
├── src/drrt/
│   ├── roadmap.py                  # PRM*/k-PRM*, thêm s/t, Dijkstra H
│   ├── tensor_graph.py             # Adj() ngầm, tích tensor, cost
│   ├── tree.py                     # cây, rewire, lan truyền cost, tra láng giềng
│   ├── planners/                   # drrt, ao_drrt, drrt_star, implicit_astar, rrt_star, prm_star
│   ├── checkers/                   # disk2d (giải tích), mujoco
│   ├── ik.py                       # pose -> cấu hình khớp không va chạm
│   └── execute.py                  # tham số hoá thời gian, chạy mj_step, kiểm tra
├── experiments/                    # YAML + script, xuất CSV/hình theo từng hình trong bài
└── tests/
```

## 4. Các giai đoạn

Thời lượng là ước lượng thô cho một người làm toàn thời gian. Mỗi giai đoạn chỉ xong khi đạt tiêu chí nghiệm thu.

### P0. Khởi tạo (0,5 ngày)
- `uv` venv, `mujoco`, `numpy`, `scipy`, `pytest`, thêm model UR10e.
- **Xong khi:** load được model, không có va chạm ở tư thế home, tỉ lệ cấu hình tự do hợp lý trên mẫu ngẫu nhiên.

### P1. Lõi thuật toán trên 2 đĩa 2D (2–3 ngày)
Tái hiện Mục 8.1 và Hình 9, 10, Bảng 1. Đĩa bán kính 0,2 trong vùng 10×10, đổi chỗ (0,0) ↔ (9,9).
- Thứ tự cài: PRM\* → đồ thị tensor ngầm → Implicit A\* (làm mốc tối ưu) → dRRT (Alg. 1, 2, 5) → ao-dRRT (Alg. 3, 4)
  → dRRT\* (Alg. 6, 7, 8).
- Chép lại vật cản từ Hình 9 bằng mắt, vì bài không cho toạ độ. **Giá trị cost tuyệt đối vì vậy sẽ không trùng Hình 10.**
- **Xong khi:**
  - cost của dRRT\* và ao-dRRT hội tụ về đúng giá trị Implicit A\* trên cùng roadmap (đây là kiểm chứng chính của Theorem 2);
  - dRRT\* có nghiệm đầu nhanh hơn ao-dRRT và Implicit A\* rõ rệt với n = 50, 100, 200;
  - test: nghiệm không va chạm, không có chu trình sau rewire, cost lưu trong cây khớp cost tính lại từ đầu.

### P2. Khả năng mở rộng, R = 3…10 đĩa (1 ngày)
Tái hiện Hình 11: n = 50, 100k vòng lặp, 10 lần chạy, so với dRRT, ao-dRRT, RRT\* trong không gian hợp.
- Bước này sẽ lộ ngay điểm nghẽn `Adj(V) ∩ T` (xem R1).
- **Xong khi:** dRRT\* vẫn tìm được nghiệm tới R=10 trong khi các phương án khác kém dần. Xu hướng đúng như bài,
  tính cả tỉ lệ thành công và cost chia cho mốc lạc quan (tổng đường đi ngắn nhất của từng robot).

### P3. Cầu nối MuJoCo và cell UR10e (2–3 ngày)
- Hai cấu hình lọc va chạm; kiểm tra cạnh theo bước nội suy (bắt đầu 0,02–0,05 rad); clearance an toàn.
- IK kèm kiểm tra va chạm để tạo start/goal từ pose đầu công cụ.
- Thiết kế cell: 2 và 4 tay quanh bàn gá chung; quét khoảng cách đế sao cho không gian làm việc chồng lấn thật.
- Roadmap riêng cho từng tay, cache ra đĩa; song song theo tay.
- **Xong khi:**
  - lọc đúng: tổng va chạm của hai cấu hình trùng với va chạm của model đầy đủ trên ≥10k mẫu ngẫu nhiên;
  - `kin + collision` cho cùng kết quả với `mj_forward`;
  - có bảng đo thực tế: tốc độ collision, tỉ lệ va chạm giữa các tay, thời gian dựng roadmap, số thành phần liên thông theo n.

### P4. Cell 2 tay: benchmark chính (2–3 ngày)
Tương ứng Hình 12, 13, nhưng 12 chiều thay vì 14: dRRT\* so với Implicit A\* và RRT\*, n = 50…5000, 4 cặp roadmap × 5 seed.
- Thực thi: tham số hoá thời gian → bám bằng actuator → `mj_step`, ghi lại sai số bám và va chạm thực tế.
  (Bài chỉ phát lại open-loop, phần này là mở rộng của mình.)
- **Xong khi:** tỉ lệ thành công tăng theo n và cost giảm theo thời gian như Hình 13; quỹ đạo tốt nhất chạy trong
  `mj_step` không có va chạm; có video.

### P5. Cell 4 tay (1–2 ngày)
24 chiều, gần với mức 28 chiều trong bài. Kịch bản lấy ý từ Hình 18, 19: nhiều tay cùng tới vị trí làm việc trên bàn gá, và
luân phiên đi qua vùng hẹp. Cần `Adj ∩ T` đã tối ưu từ P2. Thử 6 tay chỉ khi còn thời gian.
- **Xong khi:** n = 1000 mỗi tay, tìm được nghiệm trong thời gian chấp nhận được, chạy `mj_step` không va chạm, có video.

**Tổng P0–P5:** khoảng 9–12 ngày.

### P6. Đổi robot (dự phòng, chỉ làm khi có yêu cầu)
Ứng viên đã khảo sát ở `docs/robot_survey.md`, ví dụ FANUC M-10iD/12. Tiêu chí: không phải sửa code planner.

## 5. Không thể tái hiện chính xác

- **Thời gian tuyệt đối:** khác máy (bài dùng Xeon E5-4650, 128 GB) và khác ngôn ngữ. Chỉ so xu hướng và tỉ lệ giữa các thuật toán.
- **Giá trị cost:** khác robot, khác cảnh. Mốc đúng là Implicit A\* trên cùng roadmap.
- **Benchmark tay máy trong bài:** bài dùng tay 7 DoF, còn ở đây là 6 DoF đế cố định; đây là thích ứng theo bối cảnh nhà máy, không phải tái hiện y nguyên.

## 6. Rủi ro

| # | Rủi ro | Cách xử lý |
| --- | --- | --- |
| R1 | `Adj(V) ∩ T` bùng nổ: ~74 láng giềng mỗi tay → 5,4·10³ tổ hợp với 2 tay, 2,9·10⁷ với 4 tay | Duyệt từ phía cây: chỉ mục ngược theo đỉnh từng robot; dùng k-PRM\*; giới hạn số ứng viên (ghi rõ nếu làm mất đảm bảo lý thuyết) |
| R2 | Python chậm ở vòng lặp nóng | Profile từ P2; cache kết quả va chạm theo cặp cạnh; numba/C++ cho đúng phần cần |
| R3 | Tay 6 DoF không có bậc dư, start/goal bị ràng buộc chặt | IK kèm kiểm tra va chạm ở P3, thử nhiều nghiệm IK cho mỗi pose |
| R4 | Cell quá thoáng (không khác biệt) hoặc quá chật (không có nghiệm) | Quét khoảng cách đế ở P3, chọn theo tỉ lệ va chạm giữa các tay đo được |
| R5 | Nghiệm hình học hợp lệ nhưng bám có sai số, gây va chạm khi chạy động lực học | Clearance an toàn ở P3; đánh giá lại bằng `mj_step` ở P4 |
| R6 | Bài không định nghĩa rõ một số phần (`Connect_to_Target`, ...) | Xem `docs/paper_notes.md`. Mỗi lựa chọn là một tuỳ chọn cấu hình, có ghi chú trong code |

## 7. Điểm mở

Không chặn tiến độ, sẽ hỏi lại khi tới giai đoạn liên quan:
- **Tác vụ thực tế** (hàn, gắp đặt, lắp ráp): ảnh hưởng kịch bản P5. Mặc định dùng kịch bản lấy ý từ Hình 18, 19.
- **Đưa lên robot thật hay không:** nếu có thì thêm giới hạn vận tốc/gia tốc và kết nối bộ điều khiển của hãng.
