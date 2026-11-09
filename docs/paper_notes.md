# Ghi chú đọc bài: dRRT\*

Shome, Solovey, Dobson, Halperin, Bekris, arXiv:1903.00994v1 (2019). 24 trang. Mở rộng từ Dobson et al. 2017 (International Symposium on Multi-Robot and Multi-Agent Systems).

## Ý chính

Bài toán: R robot (đặc biệt là tay máy nhiều bậc tự do) chuyển động đồng thời trong không gian làm việc chung. Lập
kế hoạch trong không gian hợp thì số chiều rất lớn (4 tay 7-DoF → 28 chiều), roadmap tường minh không khả thi.

Ý tưởng: **dựng PRM riêng cho từng robot, rồi tìm kiếm ngầm trên tích tensor của chúng.**

- Roadmap hợp Ĝ không bao giờ được dựng ra. Đỉnh của Ĝ là bộ R đỉnh; hai đỉnh kề nhau nếu với mọi robot, đỉnh tương
  ứng trùng nhau hoặc kề nhau trong G_i. (Chú thích 1: khác dRRT gốc, cho phép robot đứng yên.)
- |V̂| = ∏|V_i|: lớn theo cấp số mũ nhưng không cần lưu.
- Va chạm robot–vật cản đã xử lý trong G_i, online chỉ cần kiểm robot–robot.

Hai kết quả lý thuyết:

1. **Theorem 1:** Ĝ chứa đường đi gần tối ưu nếu r(n) ≥ r\*(n) = γ(log n / n)^(1/d), với d là số chiều **của một robot**,
   không phải không gian hợp. Đúng với tổng độ dài, max độ dài, và độ dài Euclid hợp.
2. **Theorem 2:** dRRT\* và ao-dRRT tìm được đường tối ưu trên Ĝ khi số vòng lặp → ∞ (dùng chuỗi Markov hấp thụ).

## Ba thuật toán

| | Chọn đỉnh mới | Rewire | Heuristic | Đảm bảo |
| --- | --- | --- | --- | --- |
| dRRT (Alg. 1, 2, 5) | Oracle O_d: mỗi robot chọn láng giềng có góc nhỏ nhất so với hướng Q_rand | Không | Không | Đầy đủ theo xác suất |
| ao-dRRT (Alg. 3, 4) | Như dRRT | Khi đỉnh đã có trong cây | Không | Tối ưu tiệm cận |
| dRRT\* (Alg. 6, 7, 8) | I_d: hướng về đích theo H, hoặc láng giềng ngẫu nhiên | Đầy đủ theo láng giềng | Đường ngắn nhất trên từng G_i | Tối ưu tiệm cận, nghiệm đầu rất nhanh |

Những điểm làm dRRT\* nhanh:

- **Greedy child propagation (Alg. 7, dòng 19):** nếu V_new có H nhỏ hơn cha, vòng sau mở rộng tiếp từ V_new với Q_rand = T.
  Nếu đường đi ngắn nhất riêng lẻ không va nhau, nghiệm được tìm gần như ngay.
- **Branch-and-bound (dòng 11):** bỏ đỉnh có cost vượt nghiệm tốt nhất.
- **Chọn cha tốt nhất trong láng giềng (dòng 8–10) và rewire láng giềng (dòng 16–18)**, như RRT\*.
- **Focused sampling:** một phần mẫu lấy đích của một robot, giúp hội tụ (Mục 5.2).

## Số liệu đáng chú ý trong bài

- 2 đĩa, n=200: nghiệm đầu dRRT\* 0,002 s; Implicit A\* 15,5 s; ao-dRRT 0,64 s (Bảng 1).
- R = 3…10 đĩa, n = 50: dRRT\* tìm được nghiệm tới R = 10; RRT\* không còn tìm được nghiệm từ R ≥ 6; Implicit A\* thất bại ngay từ R = 3 (Hình 11).
- 2 tay 7-DoF: RRT\* không có nghiệm trong 100k vòng lặp; Implicit A\* luôn thất bại với N ≥ 500; dRRT\* đạt 100% (Hình 13).
- 28 chiều thật: nghiệm đầu trong một phần giây, 1000 vòng lặp để cải thiện, phát lại open-loop (Mục 8.3).

## Chỗ bài chưa rõ, phải tự quyết khi cài

| # | Vị trí | Vấn đề | Cách làm dự kiến |
| --- | --- | --- | --- |
| 1 | `Connect_to_Target` (Alg. 1, 3, 6) | Không định nghĩa | Kiểm tra T có trong cây; thử thêm nối cục bộ đi theo đường ngắn nhất riêng lẻ, kiểm va chạm (bật/tắt được) |
| 2 | Alg. 7 dòng 8 | `Adj(V_new) ∩ T` bùng nổ theo cấp số mũ, bài không nói cách tính | Bảng băm cho R nhỏ, chỉ mục ngược theo robot cho R lớn |
| 3 | Alg. 7 dòng 8–9 | Adj có chứa chính đỉnh đó (Mục 5.2), nên V_new có thể là cha của chính nó; I_d cũng có thể trả về V_near nếu mọi robot đứng yên | Loại V_new khỏi N; bỏ qua vòng lặp nếu V_new = V_near |
| 4 | Alg. 7 dòng 9 | argmin với va chạm "ẩn" trong L(·) | Sắp ứng viên theo cost, kiểm va chạm lần lượt, dừng ở ứng viên hợp lệ đầu tiên |
| 5 | Alg. 7 dòng 11 | Viết cost(V_new) nhưng V_new có thể chưa ở trong cây | Dùng cost(V_best) + cost(L(V_best, V_new)). Có thể cộng thêm H để cắt mạnh hơn: đây là mở rộng, không phải của bài |
| 6 | Alg. 7 dòng 15, 18 | Rewire không nói việc cập nhật cost cho nhánh con | Lan truyền cost xuống cây con |
| 7 | Alg. 7 dòng 19 | H hợp "suy ra từ các không gian thành phần" | Cost tổng: ΣH_i. Cost Euclid: √(ΣH_i²). Hai dạng đều không vượt quá cost thật |
| 8 | Mục 5.2 | Tỉ lệ focused sampling và goal bias không cho | Tham số cấu hình, quét thử ở P1 |
| 9 | Alg. 6 dòng 6, Alg. 7 dòng 17 | Ký hiệu `∩` nghĩa là "và" | Đọc là AND logic |
| 10 | Mục 4.1 | Không nói cách nối s_i, t_i vào G_i | Chèn thành đỉnh mới, nối theo cùng luật kết nối, kiểm tra cùng thành phần liên thông |
| 11 | Mục 6 | Chứng minh cho r(n) ≥ r\*(n); μ là thể tích C_f | k-PRM\* ngoài phạm vi chứng minh, phải ghi rõ khi báo cáo. μ ước lượng bằng thể tích hộp khớp × tỉ lệ tự do |

## Quyết định đã áp dụng khi cài đặt

- **Vật cản Hình 9:** đo từ ảnh render của PDF. Hai chấm trong hình rơi đúng đường lưới ứng với (9, 9) và (4,5, 4,5), nên
  khung hình là [-0,45; 9,45]. Bài viết "inflated by the radius", nên coi hình vẽ đã phình sẵn: tâm đĩa chỉ cần nằm ngoài
  hình; va chạm giữa hai đĩa vẫn dùng 2r = 0,4. Đây là cách hiểu của mình, bài không nói rõ.
- **Roadmap không nối được s–t:** khá thường gặp với r\*(n) nhỏ (ở n=50 chỉ khoảng một nửa roadmap nối được). Bài không nói cách
  xử lý; ở đây lấy mẫu lại và ghi số lần thử.
- **Oracle góc O_d (ghi chú 3):** góc tới chính đỉnh đó không xác định, nên mỗi robot đứng yên với xác suất 1/(bậc+1). Nếu
  không, ao-dRRT sẽ không bao giờ xét được các nghiệm có robot chờ, và không thể hội tụ về tối ưu trên Ĝ.
- **Connect_to_Target (ghi chú 1):** dRRT\* chỉ kiểm tra T có trong cây. dRRT và ao-dRRT thử cho mọi robot cùng đi theo đường
  ngắn nhất từ đỉnh vừa thêm tới T, sau mỗi 100 vòng; bản này đơn giản hơn local connector của dRRT gốc.
- **Focused sampling (Mục 5.2):** mới cài phần goal bias theo từng robot (10%). Chưa cài lấy mẫu trong tập informed kiểu BIT\*;
  branch-and-bound vẫn có, và test cho thấy vẫn hội tụ đúng.

## Lưu ý khi đọc phần lý thuyết

Chứng minh yêu cầu robot **holonomic** và nghiệm **robust** (có clearance δ > 0). Tay máy không xoay trọn vòng phù hợp vì
khoảng khớp bị chặn, nên khoảng cách Euclid trong khớp dùng trực tiếp được. Không xét động lực học: nghiệm là đường hình
học, phải tham số hoá thời gian và bám sau đó.

## Đọc thêm

- Solovey et al. 2015, dRRT gốc: định nghĩa local connector mà bài này lược đi (mục 1 ở bảng trên). arXiv:1305.2889.
- Dobson et al. 2017, phiên bản đầu của dRRT\*.
- Shome & Bekris 2017, da-dRRT\* (ngoài phạm vi hiện tại, chỉ cần nếu robot có khớp dùng chung).
- Karaman & Frazzoli 2011, RRT\*/PRM\*: baseline và công thức k-PRM\*. arXiv:1105.1186.
