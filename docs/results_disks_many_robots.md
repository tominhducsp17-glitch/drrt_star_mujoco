# Kết quả: 3 đến 10 robot đĩa (Hình 11)

Chạy lại:

```bash
.venv/bin/python experiments/disks_many_robots.py && .venv/bin/python experiments/disks_many_robots_report.py
```

Thiết lập: mỗi robot xuất phát trên biên và đi tới điểm đối xứng qua tâm, roadmap n = 50 mỗi robot, 10 lần chạy cho mỗi R.
Mỗi lần chạy dùng chung roadmap và truy vấn cho mọi thuật toán. Giới hạn 100k vòng **và** 300 giây mỗi lần chạy (bài không
giới hạn thời gian). Implicit A\* chỉ chạy tới R = 6 và bị giới hạn 1,4 GB RAM mỗi tiến trình.
"Mốc" là tổng đường ngắn nhất riêng lẻ của từng robot, bỏ qua va chạm giữa các robot, giống bài.

Dữ liệu: `results/disks_many_robots/summary.csv`, bảng đầy đủ `results/disks_many_robots/tables.md`, hình `results/disks_many_robots/fig11.png`.

## Kết luận

**Xu hướng chính của Hình 11 được tái hiện:** dRRT\* giải được 10/10 ở mọi R tới 10, nghiệm đầu nhanh nhất và chi phí tốt nhất
với khoảng cách lớn; các phương án trong không gian hợp (RRT\*, Implicit A\*) thất bại khi R tăng.

| R | dRRT\* thành công | dRRT\* nghiệm đầu (s) | dRRT\* cost cuối / mốc | ao-dRRT cost cuối / mốc | dRRT cost / mốc | RRT\* thành công | Implicit A\* thành công |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 3 | 10/10 | 0,001 | 1,00 | 1,14 | 1,59 | 10/10 | 10/10 |
| 4 | 10/10 | 0,003 | 1,00 | 1,35 | 1,76 | 6/10 | 10/10 |
| 5 | 10/10 | 0,003 | 1,01 | 1,43 | 2,15 | 0/10 | 2/10 (8 hết RAM) |
| 6 | 10/10 | 0,014 | 1,06 | 1,89 | 2,32 | 0/10 | 0/10 (hết RAM) |
| 8 | 10/10 | 0,061 | 1,27 | 2,08 | 2,75 | 0/10 | không chạy |
| 10 | 10/10 | 0,275 | 1,50 | 2,61 | 2,78 | 0/10 | không chạy |

## Khác với bài

1. **dRRT và ao-dRRT luôn giải được** (trừ 1 lần dRRT ở R=9), trong khi bài báo cáo cả hai giảm mạnh từ R ≥ 4–5 và không giải
   được ở R = 9, 10. Giả thuyết: `Connect_to_Target` tự cài (đi theo đường ngắn nhất tới đích, xem `paper_notes.md`) mạnh hơn bản
   của bài. Vì vậy **ở đây không tái hiện được** lợi thế về tỉ lệ thành công của dRRT\* so với dRRT/ao-dRRT; lợi thế còn lại là thời
   gian (nhanh hơn 10–100 lần) và chi phí (thấp hơn rõ rệt ở mọi R).
2. **ao-dRRT thường không chạy hết 100k vòng:** chạm giới hạn 300 s ở 6/10 (R=5), 9/10 (R=6), 7/10 (R=7), 4/10 (R=8), 3/10 (R=9).
   Chi phí cuối của ao-dRRT vì vậy hơi bi quan so với nếu cho chạy đủ.
3. **Implicit A\* trụ được tới R = 4** (bài: thất bại ngay R = 3). Ở R = 5–6 thất bại do hết bộ nhớ, không phải do hết thời gian.
4. **RRT\* thất bại từ R = 5** (bài: từ R = 6). Tham số RRT\* (bước steer √R, goal bias 5%) do mình chọn; bài không cho.

## Giới hạn

- Chỉ 10 lần chạy mỗi R, n = 50, như bài.
- Tham số planner chưa được tinh chỉnh; khác biệt so với bài ở mục trên chưa được kiểm chứng riêng.
