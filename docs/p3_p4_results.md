# Kết quả P3–P4: cell UR10e trong MuJoCo

Chạy lại:

```bash
.venv/bin/python experiments/p3_cell_check.py
MUJOCO_GL=egl .venv/bin/python experiments/p4_two_arm.py            # benchmark + demo, kết quả ở results/p4/
```

## P3: cầu nối MuJoCo

Code: `src/drrt/mujoco_cell.py`, `ik.py`, `cell_tasks.py`, `execute.py`, `shortcut.py`.

**Cell 2 tay:** hai UR10e trên bệ cao 0,6 m, đối diện nhau cách 1,5 m qua bàn gá; một cột Ø8 cm giữa bàn. Mỗi tay có dụng cụ
hình trụ dài 15 cm. Khớp giới hạn ±π. Lề an toàn 2 cm với môi trường và giữa các tay; bước nội suy 0,05 rad.

| Đo (20.000 mẫu ngẫu nhiên) | Kết quả |
| --- | --- |
| Kiểm tra offline 1 tay (tay + môi trường + tự va chạm) | 8,2 µs |
| Kiểm tra online (chỉ giữa các tay) | 5,0 µs |
| Model đầy đủ, để so | 53 µs |
| Một tay tự do với môi trường | 43,3% |
| Va chạm giữa hai tay, khi cả hai đã tự do với môi trường | 0,1% |
| Phép tách lệch so với model đầy đủ | 14/20.000, **đều theo hướng an toàn** |
| Dựng roadmap r\*(n), 1 tay | n=300: 0,5 s; n=1000: 2,5 s (bậc TB 24) |
| IK (đầu dụng cụ chỉ xuống) | 40–48 nghiệm không va chạm mỗi điểm |

Hai điều phát hiện khi làm:

- **MuJoCo cộng margin của hai geom trong cặp** (không lấy max). Model online vì vậy đặt clearance/2 cho mỗi geom của tay. Có test khoá giả định này.
- **14 trường hợp lệch** đều là cặp hình trụ–capsule của cùng một tay ở sát biên: bộ va chạm trả khoảng cách khác nhau vài mm tuỳ margin.
  Phép tách báo va chạm trong khi model đầy đủ báo tự do, nên an toàn. Test kiểm tra rằng chiều ngược lại không bao giờ xảy ra.

## P4: tác vụ đi chéo, 2 tay (12 chiều)

Mỗi tay đưa dụng cụ từ một bên cột sang bên kia, ở nửa bàn phía tay đối diện (`cell_tasks.crossing_two_arms`).
5 bộ roadmap cho mỗi n, ngân sách 60 s mỗi lần chạy.

| Thuật toán | n | Thành công | Nghiệm đầu | Cost đầu | Cost cuối | Tối ưu trên Ĝ |
| --- | --- | --- | --- | --- | --- | --- |
| dRRT\* | 300 | 5/5 | 0,001 s | 11,35 | 10,92 | 10,92 |
| dRRT\* | 1000 | 5/5 | 0,003 s | 13,84 | 9,46 | 9,31 |
| dRRT\* | 3000 | 5/5 | 0,001 s | 9,47 | 8,61 | 8,34 |
| Implicit A\* | 300 / 1000 / 3000 | 5/5 mỗi cỡ | 0,9 / 1,2 / 2,4 s | | tối ưu | |
| RRT\* 12 chiều | | **0/5** | | | | |

Cost là tổng độ dài đường trong không gian khớp (rad).

### Nhận xét thẳng thắn

- **Tác vụ này chưa đủ khó để A\* thất bại.** Đường ngắn nhất riêng lẻ không va nhau ở 5/5 bộ (n=300), 2/5 (n=1000), 3/5 (n=3000).
  Heuristic gần như chính xác nên A\* chỉ mở rộng 7–9 đỉnh và có nghiệm tối ưu trong 1–2 s. dRRT\* có nghiệm đầu nhanh hơn
  400–2400 lần, nhưng ở n=3000 sau 60 s vẫn còn cao hơn tối ưu 3%.
- **RRT\* trong không gian hợp không giải được lần nào trong 60 s**, phù hợp với nhận định của bài.
- **Lợi thế thật của dRRT\* so với A\* chỉ thể hiện khi nhiều tay**: xem P5.

### Demo và thực thi (n=1000, 60 s)

Video: `results/p4/crossing2_demo.mp4`. Số liệu: `results/p4/demo.json`.

| | Đường dRRT\* (dùng cho video) | Sau rút ngắn (tham khảo, ngoài bài) |
| --- | --- | --- |
| Cost (rad) | 8,59 (tối ưu trên Ĝ ≥ 7,55) | **4,77** |
| Số cạnh | 2 | 19 |
| Thời lượng (vmax 1 rad/s, amax 2 rad/s²) | 6,1 s | 6,9 s |
| Sai số bám lớn nhất | 0,017 rad | 0,018 rad |
| Bước có va chạm trong `mj_step` | 0 | 0 |

Số liệu lấy từ lần chạy demo gần nhất. dRRT\* chạy theo ngân sách thời gian nên mỗi lần chạy có thể ra cost hơi khác.

- **Đường thẳng trong không gian khớp** (cost 1,29) va chạm, nên tác vụ thật sự cần lập kế hoạch.
- **Rút ngắn đường giảm cost 44%** vì roadmap 6 chiều thưa nên đường trên Ĝ đi vòng. Bước này ngoài bài và có thể xoá các đoạn
  một tay đứng chờ, nên chỉ ghi lại để tham khảo, không dùng cho video.
- **Feedforward vận tốc** (ctrl = q_ref + (kd/kp)·q̇_ref) giảm sai số bám từ 0,10 xuống 0,017 rad. Không có nó, robot trễ khoảng 0,1 s
  và đầu dụng cụ lệch tới cỡ 10 cm, lớn hơn lề an toàn 2 cm.
- **Thực thi đúng như bài:** mỗi cạnh của đường trên Ĝ là một bước di chuyển đồng thời (đa thức bậc năm, dừng ở mỗi đỉnh);
  tay nào không đổi đỉnh trên cạnh đó thì đứng yên chờ. Video dùng đường gốc này. Đã thử làm mượt bằng spline + TOPP-RA
  (nhanh hơn 1,6–2,3 lần) rồi bỏ để giữ đúng hành vi của bài.

## P5: bốn tay (24 chiều)

Bốn UR10e quanh bàn vuông; mỗi tay chuyển dụng cụ giữa hai góc bàn phía mình, và mỗi góc là điểm đến của tay này đồng thời là
điểm đi của tay bên cạnh (`cell_tasks.rotate_four_arms`). n = 1000 mỗi tay, 3 bộ roadmap, ngân sách 120 s.

```bash
MUJOCO_GL=egl .venv/bin/python experiments/p4_two_arm.py --task rotate4 --sizes 1000 --seeds 3 --budget 120 --memory-gb 4 --out results/p5
```

| Thuật toán | Thành công | Nghiệm đầu | Cost đầu → cuối (rad) |
| --- | --- | --- | --- |
| dRRT\* | **3/3** | 9–24 ms | 17,5–63,5 → 6,8–8,7 |
| Implicit A\* | 0/3 | | chưa xong nổi **một** lần mở rộng trong 120 s |
| RRT\* 24 chiều | 0/3 | | |

Mốc lạc quan là 2,94 rad (từng tay đi thẳng). Đường thẳng đó va chạm, và đường ngắn nhất riêng lẻ của các tay cũng va nhau ở cả 3 bộ.

Đây là chỗ thấy rõ ý tưởng chính của bài: mỗi đỉnh có khoảng 24⁴ ≈ 330.000 láng giềng trong Ĝ, nên A\* liệt kê láng giềng là không
khả thi, còn dRRT\* chỉ đi một láng giềng mỗi vòng.

Demo (`results/p5/rotate4_demo.mp4`, `results/p5/demo.json`): cost 6,68 rad, 4 cạnh, 7,7 s, sai số bám 0,016 rad, 0 bước va chạm
trong `mj_step`. **11 lần một tay đứng yên chờ** trong khi các tay khác di chuyển (danh sách `waits` trong `demo.json`).

Chưa làm: chi phí cuối vẫn gấp 2,3–3 lần mốc lạc quan, nên chưa biết còn cách tối ưu bao xa (không có A\* để so).

## Tác vụ 2 tay khó hơn

Mục tiêu: tìm tác vụ 2 tay mà A\* không còn giải nhanh, để so với dRRT\* có ý nghĩa. Script: `experiments/task_difficulty.py`,
số liệu đầy đủ: `results/difficulty/two_arm_variants.txt` (3 bộ roadmap, n = 1000 và 3000, ngân sách 60 s).

Biến thể mới `swap2` (`cell_tasks.swap_two_arms`): hai dụng cụ đổi chỗ cho nhau ở giữa bàn, gần như trên cùng một đường, với một
thanh ngang ở độ cao 1,30 m hạn chế cách vượt qua bằng nhấc cao.

| Biến thể | Đi riêng lẻ va nhau | Tối ưu / mốc lạc quan | A\* | dRRT\* nghiệm đầu |
| --- | --- | --- | --- | --- |
| Đi chéo qua cột (cũ) | 3/6 | 1,0–1,1 | 0,5–4,4 s | 0–2 ms |
| Đổi chỗ, không vật cản | 6/6 | 1,9–2,9 | 0,4–3,4 s | 2–13 ms |
| **Đổi chỗ, thanh 1,30 m** | **6/6** | **2,9–7,0** | 0,1–1,4 s | 5–42 ms |
| Đổi chỗ, thanh 1,40 m + cột | 0/6 | 1,0–1,1 | 0,5–1,3 s | 0–2 ms |

**Kết luận: với 2 tay, không tìm được tác vụ nào làm A\* thất bại.** Biến thể đổi chỗ buộc phối hợp thật sự (đi riêng lẻ luôn
va nhau, đường tối ưu dài gấp 3–7 lần mốc), nhưng A\* vẫn giải trong vài giây vì mỗi đỉnh chỉ có 24²–40² láng giềng. dRRT\* có
nghiệm đầu nhanh hơn 10–1000 lần và trong 60 s đạt đúng tối ưu ở cả 12/12 lần chạy của hai biến thể đổi chỗ trong bảng, nhưng
nghiệm đầu có thể tệ (gấp tới gần 8 lần tối ưu).

Demo (`results/swap2/swap2_demo.mp4`): cost 4,61 rad, 3 cạnh, 5,8 s, 0 va chạm. Ở **mỗi** cạnh có một tay đứng yên nhường tay
kia: đúng kiểu phối hợp chờ mà Adj(v) chứa chính v cho phép.

Hệ quả thực tế:

- **2 tay, có vài giây để lập kế hoạch:** dùng A\* trên Ĝ để có nghiệm tối ưu.
- **Cần phản hồi tức thì, hoặc từ 3–4 tay trở lên:** dùng dRRT\*. Ở 4 tay A\* không xong nổi một lần mở rộng (mục P5).
- Chưa đo điểm chuyển ở 3 tay.

Trong lúc làm phát hiện một lỗi: cache roadmap chỉ dựa trên tên tác vụ nên một biến thể đã dùng nhầm roadmap của cell khác.
Đã sửa (khoá cache gồm mô tả cell và start/goal) và thêm test. Kết quả P4 và P5 ở trên không bị ảnh hưởng vì cell của chúng không
đổi giữa các lần chạy.
