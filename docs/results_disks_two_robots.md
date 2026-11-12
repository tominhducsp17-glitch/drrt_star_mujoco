# Kết quả: 2 robot đĩa 2D (Mục 8.1, Hình 10, Bảng 1)

Chạy lại:

```bash
.venv/bin/python experiments/disks_two_robots.py && .venv/bin/python experiments/disks_two_robots_report.py
.venv/bin/python experiments/disks_two_robots_long_run.py
```

Dữ liệu và hình ở `results/disks_two_robots/`. Mỗi n có 10 cặp roadmap, dRRT\* và ao-dRRT chạy 5 lần trên mỗi cặp, 100.000 vòng lặp.
N²-PRM\* dựng 10 lần ở n = 50, 100 và 3 lần ở n = 200.

## Kết luận

Các tiêu chí nghiệm thu đều đạt:

| Tiêu chí | Kết quả |
| --- | --- |
| dRRT\* hội tụ đúng tối ưu của Implicit A\* | **10/10 cặp roadmap ở n=200, khớp tới 4 chữ số thập phân** khi chạy 1 triệu vòng; lần cải thiện cuối muộn nhất ở vòng 268.348. Ở 100k vòng: 50/50 (n=50), 50/50 (n=100), 34/50 (n=200, chênh TB 0,08%) |
| ao-dRRT hội tụ | Đạt đúng tối ưu trên roadmap n=30 trong test (cần tới ~520k vòng). Ở 100k vòng và n=200 còn chênh TB 6,06% |
| dRRT\* có nghiệm đầu nhanh hơn | Nhanh hơn ao-dRRT 14–18 lần và Implicit A\* 9–22 lần |
| Test | 35/35 pass, gồm kiểm tra nghiệm không va chạm, cây không có chu trình, cost trong cây khớp cost tính lại |

## Thời gian (giây)

Số trong ngoặc là của bài. Thời gian đã bỏ qua bước tính H, đúng như bài coi đó là tính trước.

| | n=50 | n=100 | n=200 |
| --- | --- | --- | --- |
| N²-PRM\* dựng | 6,17 (3,43) | 27,93 (13,29) | 109,63 (69,55) |
| N²-PRM\* truy vấn | 0,002 (0,002) | 0,005 (0,005) | 0,032 (0,019) |
| 2 PRM\* cỡ n dựng | 0,043 (0,135) | 0,111 (0,274) | 0,297 (0,558) |
| Implicit A\* | **0,006 (0,886)** | **0,018 (4,214)** | **0,039 (15,468)** |
| ao-dRRT nghiệm đầu | **0,012 (1,309)** | **0,015 (0,999)** | **0,025 (0,638)** |
| dRRT\* nghiệm đầu | 0,0007 (0,003) | 0,0009 (0,002) | 0,0018 (0,002) |

## Chi phí

| n | Tối ưu A\* | dRRT\* đầu | dRRT\* 100k | ao-dRRT đầu | ao-dRRT 100k | N²-PRM\* |
| --- | --- | --- | --- | --- | --- | --- |
| 50 | 32,95 | 33,75 | 32,95 | 52,48 | 33,09 | 32,55 |
| 100 | 34,61 | 35,41 | 34,61 | 51,28 | 35,05 | 31,27 |
| 200 | 31,72 | 33,22 | 31,75 | 48,30 | 33,64 | 29,80 |

## Khác với bài

Nguyên nhân dưới đây là giả thuyết, chưa được kiểm chứng riêng, trừ chỗ ghi rõ.

1. **Implicit A\* nhanh hơn bài 150–400 lần.** Hướng kết luận vẫn đúng (dRRT\* nhanh hơn), nhưng độ chênh chỉ 9–22 lần so với
   300–7.700 lần trong bài. Giả thuyết: heuristic ΣH_i gần như hoàn hảo khi hai đĩa ít tương tác, còn bài không mô tả A\* của họ.
   **Lợi thế thật của dRRT\* so với A\* trên kịch bản này chưa được chứng minh đầy đủ**; thí nghiệm nhiều robot (`results_disks_many_robots.md`) mới là phép thử thật,
   vì bài báo cáo A\* thất bại ngay từ R=3.
2. **ao-dRRT có nghiệm đầu nhanh hơn bài 25–110 lần.** Bài không định nghĩa `Connect_to_Target`; bản của mình cho robot đi thẳng
   theo đường ngắn nhất tới đích, chính nó tạo ra nghiệm đầu. Đổi lại nghiệm đầu kém (~50, so với tối ưu ~33).
3. **Cost thấp hơn và không giảm đều theo n.** Bài (đọc từ Hình 10) khoảng 40, 38, 36 giảm dần; của mình 33, 35, 32. Mình chấp nhận
   lệch cost tuyệt đối vì vật cản chỉ đo từ hình. Riêng n=100 cao hơn n=50 có thể do hai điều: chỉ có 10 cặp roadmap với độ lệch
   chuẩn lớn (3,56), và do lọc roadmap nối được s–t (trung bình phải thử 44 lần ở n=50, 10 lần ở n=100, 4 lần ở n=200).
4. **N²-PRM\* tốt hơn Ĝ tới 10%, trong khi bài nói hai bên tương đương.** Có thể vì PRM\* trong không gian 4 chiều không phải đi theo
   roadmap riêng của từng đĩa. Cần lưu ý nếu so PRM\* với dRRT\* về chất lượng nghiệm.

## Giới hạn của kết quả

- Vật cản đo từ hình và cách hiểu "inflated by the radius" là của mình.
- Chưa cài focused sampling kiểu BIT\* mà Mục 5.2 nhắc tới.
- Thí nghiệm này không dùng MuJoCo; planner chạy trên checker giải tích.
