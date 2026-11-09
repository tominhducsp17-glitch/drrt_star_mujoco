# Khảo sát robot cho implement dRRT\* (bối cảnh nhà máy, không humanoid)

Ngày khảo sát: 2026-11-09. Số liệu model đọc trực tiếp từ file trong các repo nguồn. Chưa tải và chạy thử model nào,
nên tốc độ collision check của các robot dưới đây **chưa được đo**.

**Quyết định (2026-11-09): chốt UR10e.** Phần còn lại giữ làm tài liệu tham khảo nếu sau này phải đổi robot.

## Đề xuất ban đầu

| Vai trò | Robot | Lý do chính |
| --- | --- | --- |
| **Phát triển và benchmark chính** | **UR10e** (MuJoCo Menagerie) | MJCF chạy được ngay, có actuator; collision hoàn toàn bằng 9 khối hình học cơ bản |
| **Trình diễn sát nhà máy** | **FANUC M-10iD/12** (repo chính thức của FANUC) | Robot công nghiệp 6 trục thật, có quán tính từ hãng, Apache-2.0. Phải tự chuyển URDF → MJCF |
| Thay thế nếu biết robot của nhóm | Robot thật của nhà máy | Planner độc lập với robot, chỉ đổi file cấu hình và model |

Cell đề xuất: 2 → 4 → 6 tay gắn cố định quanh một bàn gá chung (12 → 24 → 36 chiều). Bài có thí nghiệm thật ở 28 chiều.

Lưu ý về tên robot: `m10_12_14d` được hiểu là M-10iD/12 (tầm với 1441 mm) dựa trên tên file; cần đối chiếu với datasheet.

## Tiêu chí, suy ra từ bài

1. **Đế cố định, không có khớp dùng chung.** Đây là trường hợp dRRT\* cơ bản (Mục 5, 6). Robot có khớp thân chung như
   Motoman SDA10F cần da-dRRT\* với heuristic yếu hơn.
2. **Collision nhanh.** Roadmap xử lý robot–vật cản offline, còn va chạm robot–robot được kiểm tra online liên tục. Đây là
   vòng lặp nóng nhất, nên hình học càng đơn giản càng có lợi.
3. **Có quán tính và actuator đáng tin.** Cần cho bước chạy `mj_step`. Chỉ lập kế hoạch hình học thì không cần.
4. **d nhỏ và khoảng khớp hợp lý.** Theorem 1 phụ thuộc d của **một** robot.
5. **Không bị vướng license khi chia sẻ code.**

## So sánh ứng viên

| Robot | DoF | Định dạng | Collision | Quán tính | License | Ghi chú |
| --- | --- | --- | --- | --- | --- | --- |
| **UR10e** (Menagerie) | 6 | MJCF + actuator | 9 khối cơ bản | Có | BSD-style (ROS Industrial) | Assets ~32 MB. Khớp ±2π cần thu hẹp |
| UR5e (Menagerie) | 6 | MJCF + actuator | Khối cơ bản | Có | BSD-style | Giống UR10e nhưng nhỏ hơn, cell chật hơn |
| FANUC CRX-5iA (Menagerie) | 6 | MJCF + actuator | 7 mesh | Có, từ FANUC | Apache-2.0 | Cobot, ~6 MB. Dùng được ngay nhưng collision bằng mesh |
| KUKA iiwa 14 (Menagerie) | 7 | MJCF + actuator | 47 sphere | Có | BSD-3 | d=7 đúng như bài, tiện đối chiếu, nhưng tay dư bậc không phải dạng phổ biến nhất trong nhà máy |
| **FANUC M-10iD/12**, LR Mate 200iD, M-20 (FANUC) | 6 | URDF/xacro | 7 STL | **Có, từ hãng** | Apache-2.0 | Công nghiệp thật. Phải tự chuyển, tự thêm actuator |
| ABB IRB 2400 (ROS-Industrial) | 6 | URDF/xacro | 7 STL | **Không có** | Apache-2.0 | Lập kế hoạch được; muốn chạy `mj_step` phải ước lượng quán tính |
| KUKA KR6 R900 (ROS-Industrial) | 6 | URDF/xacro | 7 STL | **Không có** | Apache-2.0 | Như trên |
| Yaskawa GP7 (ROS-Industrial) | 6 | URDF/xacro | 7 STL | **Không có** | BSD-3 | Như trên |
| Motoman SDA10F | 15 | URDF | | | | Robot trong bài, **loại** vì có thân dùng chung |
| OpenArm | 7 | MJCF | 23 mesh | Có | Apache-2.0 | **Loại**: tay cobot trên thân cố định, không hợp bối cảnh nhà máy |

## Ý nghĩa của d và khoảng khớp

Tính theo Theorem 1 với η→0, μ = thể tích hộp khớp (bỏ qua vùng va chạm và hiệu ứng biên), n = 1000. Đây là ước lượng trên
giấy, chưa đo trên roadmap thật.

| Robot | d | r\*(1000) | Láng giềng/đỉnh | Tổ hợp Adj, R=2 | Tổ hợp Adj, R=4 |
| --- | --- | --- | --- | --- | --- |
| UR10e, ±2π nguyên bản | 6 | 5,51 rad | ~74 | 5,4·10³ | 2,9·10⁷ |
| UR10e, thu về ±π | 6 | 3,09 rad | ~74 | 5,4·10³ | 2,9·10⁷ |
| FANUC CRX-5iA | 6 | 3,23 rad | ~74 | 5,4·10³ | 2,9·10⁷ |
| ABB IRB 2400 | 6 | 2,58 rad | ~74 | 5,4·10³ | 2,9·10⁷ |
| KUKA iiwa 14 | 7 | 3,06 rad | ~126 | 1,6·10⁴ | 2,5·10⁸ |

Rút ra:

- **Số láng giềng chỉ phụ thuộc d và n, không phụ thuộc khoảng khớp.** Khi dùng r\*(n), thu hẹp khớp không giảm số láng giềng.
- **Từ 7 xuống 6 DoF giảm khoảng 9 lần số tổ hợp ở R=4.** Dù vậy 2,9·10⁷ vẫn quá lớn để duyệt hết, nên vẫn cần chỉ mục cho `Adj ∩ T`.
- **Khoảng khớp quyết định độ dài cạnh.** UR ±2π cho cạnh dài tới 5,5 rad: dễ va chạm, tốn nhiều bước kiểm tra, và mỗi tư thế
  vật lý xuất hiện nhiều lần (q và q + 2π). Vì vậy nên thu về ±π.
- **r\* lớn nói chung** (2,6–3,2 rad so với đường chéo 15–18 rad) cho thấy Theorem 1 khá bảo thủ ở n nhỏ. Nên đo cả k-PRM\* (k≈22),
  nhưng phải ghi rõ k-PRM\* không nằm trong chứng minh của bài.

## Rủi ro khi đổi sang robot 6 DoF

- **Không có bậc dư:** mỗi pose đầu công cụ chỉ có một số hữu hạn nghiệm IK, nên start/goal bị ràng buộc chặt hơn. Goal phải là
  cấu hình khớp, nên cần bước IK kèm kiểm tra va chạm ở P3.
- **Không gian làm việc phải chồng lấn thật:** khoảng cách đế chọn sai thì cell quá dễ, hoặc không có nghiệm. Cần quét khoảng cách
  đế khi thiết kế cảnh.
- **Mesh của FANUC dùng convex hull trong MuJoCo:** link dài có thể bị phồng ra và chặn khe hẹp. Nếu vậy thì tách mesh hoặc thay bằng capsule.
- **Khảo sát chưa tìm thấy công bố nào dùng dRRT\* trên cell công nghiệp 6 trục.** Hai tìm kiếm thì chưa đủ để kết luận là chưa có ai làm,
  nhưng nghĩa là hiện không có số liệu để đối chiếu trực tiếp.

## Nguồn

- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie): UR5e, UR10e, CRX-5iA, iiwa 14
- [FANUC-CORPORATION/fanuc_description](https://github.com/FANUC-CORPORATION/fanuc_description): LR Mate, M-10, M-20, R-2000, CRX
- ROS-Industrial: [abb](https://github.com/ros-industrial/abb), [kuka_experimental](https://github.com/ros-industrial/kuka_experimental), [motoman](https://github.com/ros-industrial/motoman), [fanuc](https://github.com/ros-industrial/fanuc)
- [Hartmann et al. 2025, benchmark nhiều robot nhiều mục tiêu](https://arxiv.org/abs/2503.03509): có thể tham khảo khi thiết kế kịch bản
