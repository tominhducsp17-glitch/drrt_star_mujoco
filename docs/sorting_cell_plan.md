# Cell phân loại vật với 4 tay UR10e: thiết kế và kế hoạch

> **Nguồn sự thật duy nhất của project.** Mọi quyết định mới ghi vào mục 12, tiến độ cập nhật ở mục 0.
> Khi bắt đầu một phiên làm việc mới: đọc mục 0, rồi mục của mốc đang làm ở phần C.

Tạo ngày 2026-11-16. Mô phỏng hoàn toàn trong MuJoCo, không có phần cứng.

---

## 0. Trạng thái hiện tại

| Mốc | Trạng thái |
| --- | --- |
| Khảo sát và thiết kế | Xong |
| M0: cảnh và gripper | Xong (2026-11-16). Kẹp 20/20; bàn 1,30 × 1,30 m; mọi ô ≥ 2 tay |
| M1: baseline với pose thật | Xong (2026-11-16). 50/50 cảnh, 0 va chạm, **thông lượng baseline 5,56 khối/phút** |
| M2: lập kế hoạch lệch giờ và phân việc | Xong (2026-11-16). Bản cuối: 50/50 cảnh, 0 va chạm, **7,22 khối/phút** (+30% so với M1); độ trễ lập kế hoạch lại trung vị 543 ms, p99 1,39 s |
| M3: perception và baseline không learning | Xong phần đo (2026-11-16). Sai số pose lúc quyết định gắp p99 1,25 mm và 3,4°; script gắp **400/400** với pose ước lượng, 7,07 khối/phút. **Điểm quyết định đã kích hoạt** (≥ 95%): chờ chọn cách tăng độ khó trước M4 |
| M4: learning cho kỹ năng gắp | Chưa bắt đầu |
| M5: ghép và đánh giá toàn hệ thống | Chưa bắt đầu |
| M6: báo cáo | Chưa bắt đầu |

Code: thư mục `sorting_cell/` trên branch `sorting_cell_plan` của repo `drrt_star_mujoco` (đưa lên sau M3, 2026-11-16); dùng chung `.venv` ở gốc repo; CI: `.github/workflows/sorting_cell.yml`.
Số liệu chi tiết: `results/` của repo đó và mục 12.

**Việc tiếp theo:** chọn cách tăng độ khó ở điểm quyết định M3 (mục 12; đề xuất: nhiễu perception có căn cứ, ở mức
script còn khoảng 60–90%), rồi sang M4. Còn nợ: so sánh bốn tay song song với thay phiên (để M5); tìm thêm tài liệu về
lập kế hoạch nhiều tay không đồng bộ trước khi nói mở rộng dRRT\* là mới (mục 6, "Đối chiếu sau M3").

---

# Phần A. Hiểu hệ thống

## 1. Bài toán trong một đoạn

Trên một bàn chung có 8 khối lập phương 3 màu, đặt ngẫu nhiên. Bốn tay UR10e gắn gripper Robotiq 2F-85 đứng quanh bàn, cùng lúc gắp từng khối và bỏ vào khay đúng màu ở phía mình. Yêu cầu:
- không bao giờ va chạm;
- gắp thành công cả khi pose của vật đo bằng camera bị sai lệch hoặc bị che;
- xong càng nhanh càng tốt.

Mỗi tay làm việc độc lập: không có tác vụ nhiều tay cùng giữ một vật. Phối hợp chỉ gồm hai việc:
1. tránh va chạm trong không gian chung;
2. phân việc hợp lý (tay nào lấy khối nào, theo thứ tự nào).

## 2. Spec

| Chỉ số | Yêu cầu | Định nghĩa đo |
| --- | --- | --- |
| Va chạm | 0 | Số bước `mj_step` có tiếp xúc lún (`dist < 0`) giữa hai tay khác nhau, hoặc giữa tay và môi trường. Không tính tiếp xúc gripper–vật được gắp, và vật–bàn |
| Gắp thành công | ≥ 95% | Số lần nhấc được vật lên 5 cm / số lần thử gắp, trên cảnh chưa thấy, có nhiễu perception |
| Phân loại đúng | 100% vật được gắp | Vật nằm trong khay đúng màu khi kết thúc |
| Thông lượng | Đo baseline ở M1, đặt mục tiêu sau | Số vật đặt đúng khay / thời gian mô phỏng từ lúc bắt đầu tới vật cuối |
| Độ trễ lập kế hoạch lại | Đo ở M2 | Thời gian từ lúc có sự kiện tới lúc có quỹ đạo mới |

**Bộ cảnh đánh giá:** 50 cảnh cố định theo seed, mỗi cảnh 8 khối. Khối được lấy mẫu trong vùng giữa bàn, cách nhau tối thiểu 8 cm.

## 3. Kiến trúc tổng thể

```
 camera RGB-D trên cao ──► [1. Perception] ──► danh sách vật: màu, pose, thời điểm thấy rõ lần cuối
                                                          │
                                                          ▼
                             [2. Phân việc] ◄── bảng chi phí tính trước từ roadmap của từng tay
                                                          │ (tay, vật) và đích của tay
                                                          ▼
                             [3. dRRT*] ──► quỹ đạo khớp cho các tay đang di chuyển
                                                          │
 camera cổ tay (×4) ─────► [4. Policy gắp] ◄── bàn giao khi tay tới tư thế chuẩn bị gắp
                                                          │
                             [Máy trạng thái từng tay] ──► bộ thực thi ──► mj_step
```

**Mỗi khâu dùng gì, và vì sao:**

| Khâu | Dùng | Vì sao không làm khác |
| --- | --- | --- |
| Perception | Ngưỡng HSV, contour, `minAreaRect`, depth | Cách phổ biến của các hệ phân loại theo màu; đơn giản, giải thích được, đủ cho khối màu. Học bộ phát hiện chỉ cần khi vật phức tạp (giai đoạn 2) |
| Phân việc | Gán tham lam theo sự kiện, chi phí từ roadmap, phạt tranh chấp kiểu hitbox | Cần quyết định trong vài ms mỗi khi có tay rảnh; MILP chỉ dùng làm tham chiếu offline |
| Motion planning | dRRT\* (tự viết, repo `drrt_star_mujoco`) | Ở 4 tay (24 chiều), A\* trên cùng roadmap và RRT\* không ra lời giải trong 120 s, còn dRRT\* ra trong 9–24 ms |
| Policy gắp | Policy cục bộ, camera cổ tay, học từ expert trong sim | Đúng chỗ planner yếu: pose có nhiễu, cần phản hồi hình ảnh ở vài cm cuối |

## 4. Máy trạng thái của mỗi tay

| Pha | Điều khiển | Camera | Chuyển pha khi |
| --- | --- | --- | --- |
| `IDLE` (chờ việc) | — | Trên cao | Được giao vật |
| `TRANSIT_TO_PICK` (tới vật) | dRRT\* | — | Tới tư thế chuẩn bị gắp: cách vật d_h theo phương đứng, theo pose ước lượng |
| `HANDOFF` (bàn giao) | — | Bật camera cổ tay | Thấy vật trong camera cổ tay → `GRASP`; không thấy → lùi lại, trả vật về danh sách chờ |
| `GRASP` (gắp) | Script (baseline) hoặc policy | Cổ tay | Gripper đóng và kẹp được vật → `LIFT_CHECK`; quá thời gian, mất dấu vật, hoặc ra khỏi vùng dành riêng → `ABORT` |
| `LIFT_CHECK` (nhấc và kiểm tra) | Script | Cổ tay | Vật vẫn trong gripper → tắt camera cổ tay, sang `TRANSIT_TO_PLACE`; rơi → báo khâu phân việc |
| `TRANSIT_TO_PLACE` (mang tới khay) | dRRT\* | — | Tới trên khay đúng màu |
| `PLACE` (thả) | Script | — | Mở gripper → về `IDLE` |
| `ABORT` (bỏ cuộc) | Script | — | Lùi lên tư thế chuẩn bị gắp, trả vật về danh sách chờ, tăng số lần thử |

Trong các pha `HANDOFF`, `GRASP`, `LIFT_CHECK`, tay được coi là **đứng yên trong Ĝ** và giữ một **vùng dành riêng**: hình trụ quanh vật, từ tư thế chuẩn bị gắp xuống mặt bàn. dRRT\* lập kế hoạch cho các tay khác tránh vùng này.

## 5. Từng khâu

### 5.1 Perception

**Đầu vào:** ảnh RGB và depth từ camera cố định trên cao, dự kiến 640×480. Camera đặt đủ cao để thấy cả bàn.

**Chuỗi xử lý:**
1. Đổi RGB sang HSV; tạo mặt nạ cho từng màu theo ngưỡng.
2. Lấy contour; loại contour có diện tích ngoài khoảng hợp lý của một khối.
3. `cv2.minAreaRect` cho tâm (u, v) và góc xoay; khối lập phương đối xứng 90° nên góc quy về khoảng [0°, 90°).
4. Đổi (u, v) và depth tại tâm sang toạ độ cell bằng thông số camera. Trong sim, thông số biết chính xác; có thể thêm sai số hiệu chuẩn để thử độ bền.

**Theo dõi và che khuất:**
- Mỗi vật lưu một bản ghi, gồm màu, pose, thời điểm thấy rõ lần cuối và trạng thái.
- Ghép phát hiện mới với bản ghi cũ theo màu và khoảng cách gần nhất.
- Chỉ cập nhật pose khi contour có diện tích gần đủ, tức không bị che một phần.
- Vật đang được gắp hoặc đã thả thì không cập nhật từ camera.

**Đánh giá riêng:** so với pose thật lấy từ sim. Kết quả gồm sai số vị trí và góc, và tỉ lệ thời gian vật bị che khi cell đang chạy. Phân bố sai số này là mức nhiễu dùng khi huấn luyện policy (5.4).

### 5.2 Phân việc

**Bài toán gần nhất trong tài liệu:** Oberthür et al. (IFAC 2025).
- 4 tay khác loại, không gian chung, mỗi vật do đúng một tay lấy rồi đặt vào khu thả của tay đó, mục tiêu tối thiểu makespan.
- Mô hình MILP dạng lập lịch nhiều máy không đồng nhất, một công đoạn.
- Va chạm dự đoán bằng hitbox hình elip bao đoạn nối đế tay với vật; elip tốt nhất cho F1 = 0,931 so với mô phỏng quỹ đạo.
- Giải trong dưới 0,4 s, so với trên một giờ của mô hình mô phỏng quỹ đạo của El Ghazi et al.
- **Họ chưa kiểm chứng lịch bằng chuyển động thật.** Đây là chỗ project có đóng góp riêng.

**Bảng chi phí tính trước (mỗi tay):**
- Chia vùng đặt vật trên bàn thành lưới.
- Với mỗi ô, tính IK cho tư thế chuẩn bị gắp và đỉnh roadmap gần nhất.
- Thời gian tới ô ≈ H (đường ngắn nhất trên roadmap) / vận tốc khớp tối đa.
- Tương tự cho đường từ ô tới từng khay.
- Ô không có nghiệm IK không va chạm thì đánh dấu là tay đó không với tới.

**Ba mức:**
1. **Mức 0 (baseline):** mỗi tay phụ trách góc phần tư gần mình; vật nằm sát ranh giới giao cho tay có chi phí nhỏ hơn.
2. **Mức 1 (làm chính):** mỗi khi có tay rảnh, gán vật hợp lệ có chi phí nhỏ nhất. Chi phí = thời gian tới vật + thời gian tới khay + phạt tranh chấp.
   - **Vật hợp lệ:** chưa giao, pose đủ mới, tay đó với tới được.
   - **Phạt tranh chấp:** cộng một hằng số cho mỗi tay khác đang hoạt động mà hitbox elip (đế tay–vật) của nó giao với hitbox của lựa chọn đang xét, hoặc chứa vật của lựa chọn đó.
3. **Tham chiếu offline:** cài mô hình MILP của Oberthür et al. bằng OR-Tools CP-SAT, giải khi đã biết cả 8 vật từ đầu. Dùng để đo khoảng cách từ mức 1 tới lịch tối ưu theo mô hình đó.

### 5.3 Motion planning

**Dùng lại từ repo `drrt_star_mujoco`:**
- `DRRTStar`, `TensorGraph`, `Roadmap`;
- cách tách va chạm: offline gồm tay với môi trường và tự va chạm; online chỉ xét giữa các tay.

**Roadmap từng tay** dựng một lần cho cảnh cố định (bàn, bệ, khay), lưu cache. Các khối trên bàn không đưa vào roadmap.

**Ba mở rộng so với paper:**
1. **Các tay xong việc lệch giờ.** Paper giả định mọi robot cùng xuất phát và cùng tới đích. Cách làm:
   - khi có sự kiện, lấy cấu hình hiện tại của các tay đang di chuyển, chèn vào roadmap tương ứng thành đỉnh tạm, nối với láng giềng;
   - lập kế hoạch lại cho tất cả các tay đang ở pha `TRANSIT`, với đích hiện tại của chúng và đích mới của tay vừa được giao việc;
   - tay đang ở pha gắp được giữ đứng yên;
   - ngân sách thời gian cố định; dùng nghiệm tốt nhất có được, nhờ tính anytime của dRRT\*.
2. **Vùng dành riêng.** Hình trụ của tay đang gắp được thêm vào bộ kiểm tra online như vật cản tạm thời.
3. **Vật đang mang.** Hình hộp của vật gắn vào gripper trong bộ kiểm tra online, và cả offline khi lập đường tới khay.

**Phương án dự phòng** nếu mở rộng 1 quá phức tạp: đồng bộ theo lượt. Mọi tay chờ nhau xong rồi cùng lập kế hoạch một lần (như M1), chấp nhận thông lượng thấp hơn.

### 5.4 Learning: policy gắp cục bộ

**Phạm vi:** chỉ pha `GRASP`, từ tư thế chuẩn bị gắp tới khi kẹp xong. Một policy dùng chung cho 4 tay.

**Thiết kế,** dựa trên ManipGen (Dalal et al., arXiv 2410.22332) và OPTIMUS (Dalal et al., CoRL 2023):

| Quyết định | Lựa chọn | Căn cứ |
| --- | --- | --- |
| Khoảng bàn giao d_h | Thử 8 cm và 16 cm | ManipGen: vùng khởi đầu 8 / 16 / 32 / 64 cm cho 97,4 / 95,7 / 89,6 / 72,5% thành công |
| Đầu vào | Ảnh depth cổ tay 84×84, cắt ở 30 cm, chuẩn hoá về [0, 1]; ảnh depth và mặt nạ vật đích ở bước đầu; tư thế đầu dụng cụ so với bước đầu, và vận tốc; độ mở gripper | ManipGen; bỏ vận tốc hoặc dùng tư thế tuyệt đối làm thành công giảm từ 94,3% xuống khoảng 90–91% |
| Đầu ra | Δ vị trí, Δ góc của đầu dụng cụ trong hệ đế của chính tay đó; lệnh đóng/mở gripper | Cả hai paper: không gian tác vụ tốt hơn không gian khớp cho policy dùng ảnh |
| Đổi sang khớp | IK vi phân bằng mink, rồi đưa xuống actuator vị trí của UR10e | |
| Mô hình | ResNet-18, spatial softmax, MLP, loss MSE | ManipGen: ngang ACT, LSTM, Transformer mà huấn luyện nhanh hơn 5–10 lần |
| Mô hình so sánh | Diffusion Policy, chỉ khi thấy dữ liệu đa mode | |
| Tần số | 20–30 Hz trong sim | |

**Data engine (không teleop):**
1. **Expert có trạng thái thật:**
   - lấy pose thật của khối, tính tư thế gắp từ trên xuống, góc kẹp theo cạnh khối;
   - đi thẳng xuống theo phương đứng, đóng gripper, nhấc lên;
   - nếu expert script gắp chưa đủ chắc: huấn luyện expert bằng RL (PPO), như ManipGen.
2. **Ngẫu nhiên hoá:**
   - vị trí khởi đầu trong vùng bàn giao;
   - nhiễu pose đích theo phân bố đo ở 5.1, và rộng hơn;
   - kích thước, khối lượng, ma sát của khối trong một khoảng nhỏ;
   - vật gây nhiễu và tay giả trong khung hình.
3. **Thêm nhiễu khi expert thực thi (DART):** cộng nhiễu vào hành động được thực thi, nhưng nhãn vẫn là hành động đúng của expert từ trạng thái đó.
4. **Lọc dữ liệu theo OPTIMUS:**
   - chỉ giữ episode thành công;
   - bỏ quỹ đạo dài hơn trung bình quá 2 độ lệch chuẩn;
   - bỏ quỹ đạo có đầu dụng cụ ra khỏi vùng dành riêng.

   OPTIMUS vượt chính expert TAMP của nó (87% so với 52%); tác giả giải thích một phần là nhờ chỉ học từ episode thành công.
5. **Tăng cường ảnh depth** như ManipGen: nhiễu ở mép vật, lỗ thủng ngẫu nhiên, cắt ảnh ngẫu nhiên.
6. **DAgger có replay buffer:** chạy policy, expert gán nhãn cho các trạng thái policy đi tới, huấn luyện lại. ManipGen thấy buffer cỡ 100 tốt hơn on-policy (1) và off-policy (1000).

**Quy mô và tài nguyên:**
- Bắt đầu 1 nghìn episode, tăng theo đường cong tỉ lệ thành công.
- Dung lượng ước khoảng 1,4 GB (1 nghìn episode × khoảng 100 bước × ảnh 84×84 dạng 16 bit).
- Sinh dữ liệu và chạy DAgger trên máy local.
- Huấn luyện trên Colab hoặc Kaggle miễn phí; lưu checkpoint và chạy tiếp được, vì hai nền tảng giới hạn thời lượng phiên và giờ GPU mỗi tuần.
- Thuê GPU chỉ khi một lượt huấn luyện cần quá khoảng 5 giờ.

### 5.5 Thực thi

- Quỹ đạo dRRT\* chạy theo từng cạnh: đa thức bậc năm mỗi cạnh, có feedforward vận tốc, như repo cũ. Tay không đổi đỉnh trên cạnh thì đứng yên.
- Pha gắp: lệnh của policy (hoặc script) qua IK vi phân ở 20–30 Hz.
- Ghi log theo từng tay: pha, thời điểm chuyển pha, lý do bỏ cuộc, tiếp xúc.

## 6. Bối cảnh tài liệu

| Paper | Lấy gì | Khác gì so với project |
| --- | --- | --- |
| Shome et al. 2019, dRRT\* | Planner cho nhiều tay | Paper giả định mọi robot cùng xuất phát và tới đích; project mở rộng cho lệch giờ và vùng dành riêng |
| Oberthür et al. 2025 | Mô hình phân việc 4 tay, hitbox elip | Họ lập lịch offline và chưa kiểm chứng bằng chuyển động thật; project phân việc online và đo va chạm thật |
| ManipGen 2025 | Policy cục bộ, camera cổ tay, chưng cất bằng DAgger | Họ huấn luyện expert bằng RL cho hàng nghìn vật, một tay; project dùng expert script, khối đơn giản, 4 tay dùng chung |
| OPTIMUS 2023 | Dùng planner sinh demo, quy tắc lọc | Họ học cả chuỗi dài end-to-end; project chỉ học kỹ năng gắp |
| DART 2017 | Thêm nhiễu khi expert thực thi | |

**Đối chiếu sau M3 (2026-11-16): project đã có gì mới so với các paper trên?**
- **Chưa có phương pháp mới.** Những gì đã làm là ghép và mở rộng có đo đạc các phương pháp sẵn có.
- **Hitbox của Oberthür et al.: có kết quả thực nghiệm mới, phạm vi hẹp.** Paper chọn elip bằng F1 so với mô phỏng chia
  khối thô của El Ghazi et al. (F1 0,931 với d = 0,162), và tự viết ở phần kết luận rằng hitbox là "xấp xỉ của một xấp xỉ",
  lịch của họ cần được kiểm tra lại. Project đo bằng hình học chính xác (MuJoCo, UR10e + 2F-85, lề 2 cm) và định thời thật:
  F1 tốt nhất 0,888 (W = 0,20 m; 0,876 ở W = 0,15 m) nếu tính va chạm ở bất kỳ độ lệch giờ nào, nhưng chỉ ≤ 0,70 nếu hai
  việc bắt đầu cùng lúc. Trong hệ thống chạy thật có dRRT\* và vùng dành riêng, mức phạt hitbox không làm tăng thông lượng
  đo được (khoảng tin cậy 95% chứa 0), vì tranh chấp được giải với thời gian chờ khoảng 0,1 s. Giới hạn: một cell, một
  loại tay, chỉ trong sim, khác bố cục của paper.
- **dRRT\* (Shome et al.): mở rộng đã chạy được và đo được**: lập kế hoạch lại theo sự kiện khi các tay lệch giờ, vùng
  dành riêng = tập quét cục bộ, khối ảo cho vật đang mang; +30% thông lượng so với theo lượt, p99 độ trễ 1,4 s, 0 va chạm
  trên 400 khối. Chưa thể nói là mới: mới chỉ đọc 4 paper, trong khi có các hướng về lập kế hoạch nhiều tay không đồng bộ
  chưa được đọc. Cần tìm tài liệu trước khi viết điều này trong báo cáo.
- **Phần learning (ManipGen, OPTIMUS, DART): chưa có gì**, đó là M4–M5. Kết quả M3 cho thấy cần làm bài toán khó hơn thì
  learning mới có lý do (mục 12).

## 7. Câu hỏi phỏng vấn thường gặp

**Vì sao không dùng một planner cho cả không gian 24 chiều (RRT\*, PRM\*)?**
Số chiều lớn làm lấy mẫu trực tiếp không hiệu quả. Trong repo cũ, RRT\* trên không gian gộp không giải được từ 5 robot đĩa, và không giải được cell 4 tay UR10e trong 120 s. dRRT\* dựng roadmap riêng cho từng tay (6 chiều) rồi tìm kiếm ngầm trên tích tensor, nên không bao giờ phải lấy mẫu trong không gian 24 chiều.

**Vì sao không dùng A\* trên cùng roadmap tensor, vì nó cho nghiệm tối ưu?**
Với 2 tay, A\* chạy tốt (vài giây). Với 4 tay, mỗi đỉnh có khoảng 24⁴ ≈ 330 nghìn láng giềng, nên A\* không xong nổi một lần mở rộng trong 120 s. dRRT\* chỉ đi một láng giềng mỗi vòng, nhờ oracle hướng về đích theo heuristic.

**dRRT\* có tối ưu không?**
Tối ưu tiệm cận (Theorem 2 của paper): chạy đủ lâu thì chi phí hội tụ về tối ưu trên roadmap tensor. Repo cũ kiểm chứng điều này: 10/10 cặp roadmap khớp với A\* tới 4 chữ số thập phân.

**Vì sao không học end-to-end cả bài toán?**
- Chuỗi dài nhiều bước và phối hợp nhiều tay cần lượng dữ liệu rất lớn.
- Mất bảo đảm không va chạm.
- Khi hỏng thì không biết khâu nào hỏng.

Chia theo khâu cho phép đánh giá và thay từng phần. Learning chỉ đặt ở chỗ phương pháp cổ điển yếu nhất.

**Vì sao cần learning, khi gắp bằng IK theo pose cũng được?**
Khi pose ước lượng có sai số hoặc vật bị che, gắp mở (open-loop) theo pose sẽ hỏng; policy dùng camera cổ tay sửa sai số ở vài cm cuối. Project kiểm tra điều này bằng số liệu ở M3. Nếu script đã đạt 95% thì phải làm bài toán khó hơn chứ không thêm learning cho có.

**Vì sao camera cổ tay mà không dùng camera trên cao cho đoạn gắp?**
- Lúc gắp, chính gripper và các tay khác che vật khỏi camera trên cao.
- Camera cổ tay nhìn theo hệ toạ độ gripper, nên khối ở đâu trên bàn thì ảnh cũng gần giống nhau; policy dễ tổng quát hơn.

**Làm sao bảo đảm policy học được không gây va chạm?**
- Trong pha gắp, tay đó có vùng dành riêng mà dRRT\* bắt các tay khác tránh.
- Policy bị giới hạn trong vùng đó; ra ngoài thì bỏ cuộc và trả quyền cho planner.
- Mọi lần chạy đều đếm va chạm thật trong `mj_step`.

**Không có teleop thì dữ liệu ở đâu ra?**
- Sim biết trạng thái thật, nên một expert script dùng pose thật sinh demo tự động.
- Student chỉ được nhìn camera.
- Thêm nhiễu khi expert thực thi để có dữ liệu sửa sai; DAgger để sửa lệch phân bố.

**Covariate shift là gì, DAgger giải quyết thế nào?**
Policy học từ demo của expert chỉ thấy các trạng thái expert đi qua. Khi chạy thật, sai một chút là policy rơi vào trạng thái chưa từng thấy, và lỗi tích luỹ. DAgger cho policy tự chạy, nhờ expert gán nhãn ở các trạng thái đó rồi huấn luyện lại. Trong sim làm được vì expert trả lời được ở mọi trạng thái; với teleop thì rất tốn người.

**Vì sao đầu ra là dịch chuyển của đầu dụng cụ, không phải góc khớp?**
Từ ảnh suy ra góc khớp phải học cả phép chiếu camera lẫn IK. Cả ManipGen và OPTIMUS đều thấy không gian tác vụ cho kết quả tốt hơn. Đầu ra theo hệ đế của chính tay đó giúp một policy dùng chung được cho 4 tay.

**Hạn chế của project?**
- Chỉ trong mô phỏng, chưa kiểm chứng sim-to-real.
- Vật đơn giản ở giai đoạn đầu.
- Không có tác vụ nhiều tay cùng giữ một vật.

---

# Phần B. Thực hiện

## 8. Cấu trúc repo mới

Thực tế (2026-11-16): project nằm ở thư mục `sorting_cell/` trên branch `sorting_cell_plan` của repo `drrt_star_mujoco`, main giữ nguyên. Cài: `uv pip install -e . -e sorting_cell` ở gốc repo. Không cài `drrt` bằng `pip install git+...`, vì thư mục `third_party/` không đi kèm. Cây dưới đây là đề xuất ban đầu; tên package vẫn là `sortcell`.

```
multiarm_sorting_cell/
├── README.md
├── pyproject.toml
├── configs/                  # cell.yaml (hình học, camera), learning.yaml
├── third_party/              # Robotiq 2F-85 từ Menagerie (kèm SOURCE.md, LICENSE)
├── src/sortcell/
│   ├── scene/                # dựng cell: tay, gripper, bàn, khay, khối, camera; lấy mẫu cảnh
│   ├── perception/           # overhead.py (HSV → pose), tracker.py (bản ghi vật, che khuất)
│   ├── allocation/           # costs.py (bảng chi phí), zones.py, greedy.py, hitbox.py, milp.py
│   ├── planning/             # service.py (gọi dRRT*, lệch giờ, vùng dành riêng, vật đang mang)
│   ├── skills/               # ik.py (mink), scripted_grasp.py, policy_grasp.py, place.py
│   ├── control/              # arm_fsm.py (máy trạng thái), executor.py (mj_step, log)
│   ├── learning/             # expert.py, datagen.py, dataset.py, model.py, train.py, dagger.py
│   └── eval/                 # scenarios.py (50 cảnh), metrics.py, run_cell.py
├── experiments/              # script cho từng so sánh ở mục 11
├── results/
└── tests/
```

## 9. Giao diện giữa các module

```python
@dataclass
class ObjectRecord:              # perception → phân việc
    id: int
    color: str                   # "red" | "green" | "blue"
    pose: np.ndarray             # x, y, z, yaw trong hệ cell
    last_seen: float             # thời điểm thấy rõ lần cuối (giây mô phỏng)
    status: str                  # "on_table" | "assigned" | "held" | "placed" | "dropped"
    attempts: int

@dataclass
class Assignment:                # phân việc → máy trạng thái
    arm: int
    object_id: int
    pregrasp_q: np.ndarray       # cấu hình khớp ở tư thế chuẩn bị gắp
    place_q: np.ndarray          # cấu hình khớp trên khay đúng màu

class PlannerService:            # máy trạng thái → planning
    def plan(self, current_q: list[np.ndarray], goals: list[np.ndarray | None],
             reserved: list[Region], held: list[ObjectRecord | None], budget_s: float) -> CompositePath: ...
    # goals[k] = None nghĩa là tay k đứng yên (đang gắp hoặc rảnh)

class GraspSkill(Protocol):      # chung cho script và policy
    def reset(self, arm: int, target: ObjectRecord) -> None: ...
    def act(self, obs: dict) -> tuple[np.ndarray, float]: ...   # (Δ tư thế đầu dụng cụ, lệnh gripper)
    def done(self) -> str | None: ...                           # None | "grasped" | "abort"
```

## 10. Các mốc và việc cụ thể

Thời lượng là ước lượng thô. Không sang mốc sau khi chưa qua điều kiện nghiệm thu.

### M0: cảnh và gripper (tuần 1)
- [x] Tạo repo, cài `drrt` editable, CI chạy test. *(thư mục `sorting_cell/` trên branch `sorting_cell_plan`, đưa lên sau M3)*
- [x] Thêm Robotiq 2F-85 từ Menagerie, gắn vào mặt bích UR10e; bỏ dụng cụ hình trụ cũ.
- [x] Dựng cell 4 tay: bệ, bàn (kích thước chốt theo tầm với), 3 khay mỗi tay đặt cạnh đế.
- [x] Kiểm tra tầm với: mọi ô trên vùng đặt vật có IK từ ít nhất 2 tay; mọi khay có IK từ tay sở hữu.
- [x] Camera trên cao và 4 camera cổ tay; render thử.
- [x] Lấy mẫu cảnh: 8 khối 5 cm, 3 màu, cách nhau tối thiểu 8 cm.
- [x] Cập nhật tách va chạm cho gripper; kiểm chứng với model đầy đủ như repo cũ.
- [x] Dựng và cache roadmap 4 tay.

**Nghiệm thu:**
- gripper kẹp, nhấc và giữ khối 5 cm ổn định trong 10 giây, 20/20 lần;
- phép tách va chạm không bao giờ báo tự do khi model đầy đủ báo va chạm.

### M1: chạy hết chuỗi với pose thật (tuần 2)
- [x] Máy trạng thái từng tay, phân việc mức 0, gắp và thả bằng script.
- [x] Lập kế hoạch theo lượt: các tay đang di chuyển xuất phát cùng lúc, lập kế hoạch một lần, chờ nhau xong.
- [x] Bộ đo: thông lượng, va chạm, gắp thành công, log từng tay.
- [x] Chạy 50 cảnh đánh giá.

**Nghiệm thu:** 50/50 cảnh phân loại xong, 0 va chạm. Ghi số thông lượng baseline vào mục 0.

### M2: lập kế hoạch lệch giờ và phân việc mức 1 (tuần 3–4)
- [x] `PlannerService` với cấu hình hiện tại làm đỉnh tạm, ngân sách thời gian, giữ tay đang gắp đứng yên.
- [x] Vùng dành riêng và vật đang mang trong bộ kiểm tra online.
- [x] Bảng chi phí tính trước; hitbox elip; phân việc mức 1.
- [x] Phân việc tham chiếu bằng CP-SAT (offline).

**Nghiệm thu:**
- 0 va chạm trên 50 cảnh;
- thông lượng cao hơn M1;
- ghi độ trễ lập kế hoạch lại (trung vị và phân vị 95).

### M3: perception, baseline không learning (tuần 4–5)
- [x] Phát hiện bằng HSV, `minAreaRect`, depth; bản ghi vật và xử lý che khuất.
- [x] Đánh giá perception so với pose thật: sai số vị trí và góc, tỉ lệ bị che khi cell đang chạy.
- [x] Thay pose thật bằng pose ước lượng trong toàn hệ thống; chạy 50 cảnh.

**Nghiệm thu:** có phân bố sai số pose, và có tỉ lệ gắp thành công của script khi dùng pose ước lượng.

**Điểm quyết định:** nếu script đã đạt ≥ 95%, khâu learning chưa có lý do tồn tại. Khi đó tăng độ khó trước khi sang M4:
- nhiễu perception lớn hơn;
- khối nhỏ hơn so với độ mở gripper;
- khối nằm sát nhau;
- hoặc chuyển sớm sang vật giai đoạn 2.

Ghi quyết định vào mục 12.

### M4: learning cho kỹ năng gắp (tuần 6–8)
- [ ] Môi trường một tay cho pha `GRASP`, có tay giả và vật gây nhiễu.
- [ ] Expert dùng trạng thái thật, nhiễu kiểu DART, lọc episode; sinh 1 nghìn episode.
- [ ] Dataset, model, script huấn luyện có checkpoint, chạy trên Colab hoặc Kaggle.
- [ ] DAgger có replay buffer.
- [ ] Đánh giá riêng kỹ năng: tỉ lệ thành công theo mức nhiễu pose, với d_h = 8 cm và 16 cm.

**Nghiệm thu:** ≥ 95% gắp thành công trên vị trí chưa thấy, ở mức nhiễu đo được ở M3, và cao hơn script ở cùng điều kiện.

### M5: ghép và đánh giá toàn hệ thống (tuần 9–10)
- [ ] Đưa policy vào máy trạng thái; bật và tắt camera cổ tay theo pha.
- [ ] Chạy 50 cảnh; đo theo spec mục 2.
- [ ] Chạy các so sánh ở mục 11.

**Nghiệm thu:** đạt spec mục 2.

### M6: báo cáo (tuần 11)
- [ ] README, video cell chạy, bảng kết quả, các so sánh.
- [ ] Cập nhật mục 7 bằng số liệu thật.

### Sau M5 (tuỳ chọn): vật giai đoạn 2
Hình dạng, màu và texture đa dạng, vật nằm sát nhau. Cần bộ phát hiện học được cho camera trên cao.

## 11. Các so sánh để chứng minh quyết định

| So sánh | Chứng minh điều gì | Mốc |
| --- | --- | --- |
| Phân việc mức 0 / mức 1 / CP-SAT offline | Phạt tranh chấp có ích; mức 1 cách tối ưu bao xa | M2 |
| Hitbox dự đoán tranh chấp so với lần dRRT\* phải bắt tay chờ | Độ chính xác thật của mô hình Oberthür et al. | M2 |
| Bốn tay song song có dRRT\* so với thay phiên từng tay | Giá trị của motion planning nhiều robot | M2 |
| Lập kế hoạch theo lượt so với lệch giờ | Giá trị của mở rộng dRRT\* | M2 |
| Gắp bằng script theo pose ước lượng so với policy, cùng mức nhiễu | Lý do cần khâu learning | M4–M5 |
| d_h = 8 cm so với 16 cm; có và không có DAgger | Cấu hình bàn giao và huấn luyện | M4 |

---

# Phần C. Quyết định và rủi ro

## 12. Nhật ký quyết định

| Ngày | Quyết định | Lý do |
| --- | --- | --- |
| 2026-11-15 | 4 tay UR10e thay vì OpenArm | dRRT\* có lợi rõ nhất khi nhiều tay; OpenArm hạn chế không gian làm việc |
| 2026-11-15 | Gripper Robotiq 2F-85, model từ MuJoCo Menagerie | Mã nguồn mở, là gripper hay đi với UR ngoài đời |
| 2026-11-15 | 1 camera trên cao cho high-level, 4 camera cổ tay cho learning, bật theo pha | Camera trên cao thấy toàn bàn nhưng bị gripper che lúc gắp; camera cổ tay nhìn gần và cho policy cục bộ |
| 2026-11-15 | Learning chỉ cho pha gắp, không end-to-end | Giữ bảo đảm không va chạm; dữ liệu ít; dễ tìm lỗi |
| 2026-11-15 | Một policy dùng chung cho 4 tay | Gắp là việc cục bộ; phối hợp do dRRT\* lo; mỗi episode cho dữ liệu gấp 4 |
| 2026-11-15 | Phân việc là module cổ điển riêng, không do dRRT\* hay learning đảm nhận | Cần quyết định nhanh và giải thích được |
| 2026-11-15 | Không teleop: expert có trạng thái thật, DART, DAgger | Không có phần cứng; sim biết trạng thái thật |
| 2026-11-16 | Perception bằng HSV, contour, `minAreaRect` | Cách phổ biến của hệ phân loại theo màu; đơn giản, tỉ lệ thành công cao |
| 2026-11-16 | 8 khối, 3 màu, một cỡ, đặt rời nhau | Bắt đầu đơn giản; vật phức tạp để sau |
| 2026-11-16 | Mô hình ResNet-18 với loss MSE; Diffusion Policy chỉ để so sánh | ManipGen: ngang ACT và Transformer, huấn luyện nhanh hơn 5–10 lần |
| 2026-11-16 | Colab và Kaggle miễn phí trước, thuê GPU khi một lượt quá khoảng 5 giờ | Tiết kiệm chi phí |
| 2026-11-16 | Repo mới; cài `drrt` editable từ repo cũ | Giữ repo implement paper gọn |
| 2026-11-16 | **M0 – gripper:** dùng `robotiq_2f85` gốc của Menagerie (commit 370c439), không dùng v4. Phải đặt lại `cone=elliptic, impratio=10` ở model cha vì `attach` bỏ `<option>` của model con | Thử kẹp khối 5 cm, 0,1 kg, vị trí và góc ngẫu nhiên, nhấc 10 cm, giữ 10 s: **20/20**, trượt tương đối tối đa 0,081 mm trong 10 s. Bản v4: 20/20, 0,120 mm. Khối 0,3 kg và 0,5 kg: 20/20. Nếu quên đặt lại option (cone pyramidal): vẫn 20/20 nhưng trượt 1,5 mm trong 10 s, nhiều hơn khoảng 18 lần. Lắc vai 90° trong 1 s: không rơi lần nào, khối xê dịch tối đa 4,2 mm |
| 2026-11-16 | **M0 – bố cục:** đế cách tâm bàn 0,85 m, cao 0,75 m (ngang mặt bàn), bệ 0,30 × 0,30 m. **Bàn 1,30 × 1,30 m**, mặt bàn cao 0,75 m. **Vùng đặt vật 0,70 × 0,70 m** ở giữa | Quét đế cách tâm 0,75–1,15 m và đế cao 0,75 / 0,90 m (bàn = đế − 0,20 m). 0,85 m là khoảng cách lớn nhất mà mọi ô trên bàn còn ≥ 2 tay; từ 0,95 m góc bàn chỉ còn 1 tay; ở 1,15 m tâm bàn không đủ 2 tay. Đế cao 0,90 m làm giảm tầm với ở mọi khoảng cách. Kiểm tra cuối (lưới 5 cm, 4 góc khối, d_h = 16 cm, cách môi trường ≥ 2 cm, đi thẳng xuống được tới tư thế kẹp): **625/625 ô trên bàn ≥ 2 tay; 225/225 ô trong vùng đặt vật ≥ 3 tay** (196 ô có 4 tay). Với d_h = 8 cm: mọi ô ≥ 2 tay |
| 2026-11-16 | **M0 – khay:** mỗi tay 3 khay 0,30 × 0,30 m, thành cao 6 cm, đáy cao 0,60 m, tâm cách đế 0,55 m ở các góc 115°, 180°, 245° (0° = hướng vào tâm bàn), thứ tự đỏ / xanh lá / xanh dương. Mỗi khay 5 chỗ thả (tâm và 4 góc cách tâm 6,5 cm) | **60/60** chỗ thả (12 khay × 5) có IK không va chạm từ tay sở hữu, ở tư thế tiếp cận (cao hơn đáy 30 cm) và đi thẳng xuống được tới tư thế thả (cao hơn đáy 4,5 cm) |
| 2026-11-16 | **M0 – camera:** camera trên cao ở z = 1,962 m, fovy 60°, 640×480, nhìn thẳng xuống; 4 góc bàn rơi vào pixel (97–543, 17–463). Camera cổ tay 84×84, fovy 60°, lệch khỏi trục gripper 8,5 cm theo phương vuông góc với mặt phẳng đóng mở, nhìn về điểm dưới TCP 10 cm. Giảm ánh sáng để màu khối không bị cháy | Ở d_h = 16 cm khối chiếm khoảng 12 × 12 px giữa ảnh cổ tay, hai má kẹp thấy ở mép dưới; ở d_h = 8 cm khối chiếm phần lớn ảnh. Ảnh: `results/cameras/` |
| 2026-11-16 | **M0 – lập kế hoạch có vật đang mang:** model lập kế hoạch luôn có một khối ảo 5 cm giữa hai má kẹp, va với môi trường và với tay khác. Gripper trong model lập kế hoạch để mở hẳn | Bảo thủ hơn "gắn vật khi đang mang" của mục 5.3, nhưng không cần đổi model theo pha, và làm luôn phần "vật đang mang" của M2. Phép tách va chạm (offline từng tay + online giữa các tay) khớp model đầy đủ: 3.000 cấu hình 4 tay, 0 lần bỏ sót, 0 lần lệch |
| 2026-11-16 | **M0 – roadmap:** mỗi tay 1.500 mẫu "tác vụ" (nghiệm IK ở tư thế chuẩn bị gắp trên bàn và tiếp cận khay) + 1.500 mẫu đều, nối k-PRM\*. Cấu hình hiện tại và đích chèn vào thành đỉnh tạm mỗi truy vấn | PRM đều trong 6 chiều quá thưa ở vùng gripper chúc xuống. Mỗi tay khoảng 2.920 đỉnh và 46.000 cạnh trong thành phần chứa HOME; dựng khoảng 40 s mỗi tay |
| 2026-11-16 | **M1 – gắp và thả:** gripper mở trước tới ctrl 50 (khe 70 mm, đo bằng `measure_gripper.py`). Trước khi gắp/thả, kiểm tra trên trạng thái sim hiện tại rằng đoạn đi xuống không tới gần (< 5 mm) khối nào khác; thử lần lượt 4 hướng kẹp tương đương | Không có kiểm tra này, khâu nối của ngón kẹp quệt khối bên cạnh cách 9,5 cm khi hướng kẹp trùng phương nối hai khối (cảnh 1000: 30 bước va chạm `arm_cube`) |
| 2026-11-16 | **Lỗi IK đã sửa:** sai số hướng ½Σ Rc_i × R_i cũng bằng 0 ở hướng quay π quanh trục dụng cụ; IK hội tụ về nghiệm lật làm cổ tay nhảy khoảng π giữa hai điểm liền nhau của đoạn đi xuống. Đổi sang vector quay của R·Rcᵀ; đoạn thẳng từ chối bước nhảy khớp > 0,3 rad | Phát hiện qua sai số bám 0,44 rad ở cảnh 1035 của M1. Mọi số liệu M1/M2 ở đây là sau khi sửa |
| 2026-11-16 | **Lập kế hoạch:** chỉ các tay đang di chuyển nằm trong Ĝ. Tay đứng yên là tập cấu hình cố định; kiểm tra "tay di chuyển với tay cố định" được cache theo cạnh của từng tay | Khi cả 4 tay nằm trong Ĝ, truy vấn chỉ có 2 tay di chuyển có lần không ra lời giải trong 15 s (dRRT\* mở rộng cả các tay đứng yên). Sau khi đổi: mỗi lần 0,6–1 s với ngân sách 0,5 s |
| 2026-11-16 | **Rút gọn đường đi** (nối thẳng tới đỉnh xa nhất còn nối được, cùng các checker), rồi **chia cạnh thành đoạn ≤ 0,8 rad** khi chạy lệch giờ. Khác với repo `drrt_star_mujoco`, nơi đã bỏ rút gọn | Đường thô của dRRT\* trên roadmap đi vòng (lượt đầu cảnh 1000: 15 đỉnh, 35 s mô phỏng). Sau khi rút gọn, đa số đường còn 2–3 đỉnh. Chia đoạn: sự kiện giữa cạnh được xử lý sau tối đa khoảng 1,5 s; với đa thức bậc năm bị giới hạn vận tốc, tổng thời gian gần như không đổi. Cảnh 1000 (M2): makespan 79,2 → 65,2 s |
| 2026-11-16 | **M2 – lệch giờ:** lập kế hoạch lại tại ranh giới đoạn (mọi tay đang di chuyển dừng ở đó). Tay đang gắp/thả đứng ngoài Ĝ với **vùng dành riêng = mọi cấu hình trên đoạn quét cục bộ của nó**, chứ không phải hình trụ quanh vật như mục 5.3. Đích mới chỉ được giao khi tập quét ở đích không va tập đích của mọi tay khác | Tập cấu hình bao cả thân tay lẫn vùng quanh vật, và kiểm tra bằng cùng checker. Ràng buộc đích bảo đảm một tay tới nơi không chặn đích của tay khác. Tay chờ được đi xuống trung bình 1,8 s mỗi cảnh (cộng 4 tay) |
| 2026-11-16 | **M2 – phân việc mức 1:** chi phí = (H tới vật + H từ vật về khay đúng màu) / 1 rad/s từ bảng chi phí (lưới 5 cm trên vùng đặt vật) + 3 s cho mỗi tay đang hoạt động có hitbox giao. Hitbox: elip có tiêu điểm là đế và vật, bán trục lớn = nửa khoảng cách + 0,15 m, bán trục nhỏ 0,15 m | Phạt 10 s cho kết quả gần như nhau trong sim (6,94 so với 6,96 khối/phút) dù trong mô hình lập lịch có tốt hơn một chút |
| 2026-11-16 | **Kết quả M1 và M2** (50 cảnh seed 1000–1049, pose thật, 12 tiến trình song song trên 18 nhân) | M1 (theo lượt, mức 0): 50/50 cảnh, 400/400 khối đúng khay, **0 va chạm**, gắp 400/400, 5,56 khối/phút (5,22 nếu tính cả thời gian lập kế hoạch), makespan TB 88,6 s. M2 lệch giờ + mức 0: 5,61 khối/phút, tức **lệch giờ một mình không giúp gì** vì mức 0 chia bàn cố định. **M2 lệch giờ + mức 1: 50/50, 0 va chạm, 400/400, 6,96 khối/phút (+25%), 6,26 tính cả lập kế hoạch (+20%), makespan TB 70,4 s.** Độ trễ lập kế hoạch lại (563 lần): trung vị 478 ms, p95 1.086 ms, p99 9,5 s. 2 lần dRRT\* không ra lời giải (ngân sách gấp đôi tới tổng 18,9 s rồi huỷ việc); cả hai cảnh vẫn xong. Sai số bám tối đa 0,023 rad |
| 2026-11-16 | **Tham chiếu CP-SAT** (mô hình Oberthür et al.: thời lượng việc từ bảng chi phí + pha cục bộ, hai việc có hitbox giao không được chạy chồng, cực tiểu makespan) | Giải tối ưu 50/50 cảnh, TB 0,55 s (tối đa 3,1 s). Trong cùng mô hình: mức 1 kém tối ưu TB 50,7% (trung vị 48,8%), mức 0 kém 72,3%. Mức phạt 10 s: 47,8%. Mô hình cấm chạy chồng khi tranh chấp, còn trong sim dRRT\* cho phép vòng tránh, nên các con số này chỉ so sánh tương đối |
| 2026-11-16 | Đo va chạm thêm hai loại: `arm_cube` (tay chạm khối không phải khối nó đang làm, gồm cả khối do tay khác mang) và `held_env` (khối đang mang chạm bàn, khay, khối khác). Cả hai tính là va chạm | Mục 2 không nói rõ; tính chặt. **Đính chính:** M1 đếm cả hai loại, nhưng bộ điều phối M2 bản đầu chưa bật `held_env`, nên các số M2 ở dòng "Kết quả M1 và M2" chưa gồm loại này. Lượt chạy lại M2 (dòng "M2 bản cuối" bên dưới) và M3 có gồm, vẫn 0 va chạm |
| 2026-11-16 | Kết quả chỉ lặp lại được về mặt thống kê | dRRT\* chạy theo ngân sách thời gian nên đường đi phụ thuộc tải CPU; độ trễ đo khi 12 tiến trình chạy song song (một tiến trình riêng: khoảng 0,4–0,6 s) |
| 2026-11-16 | Chưa làm: so sánh 4 tay song song với thay phiên từng tay (mục 11) | Để làm cùng M5. So sánh hitbox đã làm ở đầu M3 (dòng bên dưới) |
| 2026-11-16 | **Ngân sách lập kế hoạch lại:** không ra lời giải thì tìm tiếp trên cùng cây dRRT\* (thay vì gấp đôi ngân sách và dựng lại cây), **tổng tối đa 1 s**, rồi huỷ việc vừa giao | Bản đầu M2: p99 9,5 s, lâu nhất 19,2 s (2 lần thất bại, mỗi lần 18,9 s). Thử 50 cảnh mức 1: tổng 2 s cho p99 2,3 s, 6,43 khối/phút tính cả lập kế hoạch; tổng 1 s cho p99 1,34 s, 6,45 khối/phút. Chọn 1 s. Số lần thất bại tăng (2 → 20–40 mỗi 50 cảnh) nhưng lần nào cũng gỡ được |
| 2026-11-16 | **Lỗi treo đã sửa:** sau một lần lập kế hoạch thất bại cho tay đang mang vật, vòng lặp thử lại ngay, mỗi lần tốn 1 s thời gian thật mà sim chỉ chạy 0,05 s. Sửa: chờ 1 s mô phỏng rồi mới thử lại; từ lần thất bại thứ hai liên tiếp, các tay rảnh không ở HOME được cho về HOME | Lộ ra ở lượt M3 đầu tiên: một cảnh chạy hơn 30 phút không xong (chạy riêng cảnh đó thì xong, vì dRRT\* phụ thuộc thời gian). Không đọc được stack của tiến trình đó; nguyên nhân suy từ code. Sau khi sửa: mọi lượt 50 cảnh xong bình thường |
| 2026-11-16 | **M2 bản cuối** (sau hai sửa trên, đếm cả `held_env`; 50 cảnh, pose thật) | 50/50, 400/400, **0 va chạm**, gắp 400/400, **7,22 khối/phút** (6,59 tính cả lập kế hoạch), makespan TB 68,5 s. Độ trễ lập kế hoạch lại (564 lần): trung vị 543 ms, p95 1,23 s, p99 1,39 s, lâu nhất 1,46 s; 41 lần thất bại, đều gỡ được. Hai lượt cùng cấu hình chênh nhau trung bình 0,14 khối/phút mỗi cảnh: đó là độ nhiễu giữa các lượt |
| 2026-11-16 | **Hitbox (mục 11), đo offline:** mỗi việc = pha vươn tới vật (HOME → chuẩn bị gắp theo đường ngắn nhất của riêng tay đó → xuống → kẹp → lên, định thời như khi chạy). Tranh chấp thật của hai việc: chạy độc lập, tay thứ hai lệch giờ δ (lưới 0,25 s), có va nhau không | 50 cảnh, 16.654 cặp. 70% cặp va nhau ở ít nhất một δ. Hitbox W = 0,15 m: **precision 0,92, recall 0,84, đúng 83,5%**; W = 0,20 m: 0,84 / 0,95 / 83,4%. Nếu hai việc bắt đầu cùng lúc (δ = 0): precision chỉ 0,57, recall 0,90. Cặp hitbox bỏ sót là cặp va ở ít độ lệch hơn (trung vị 26% số δ, so với 53% ở cặp bắt được). Vật cách nhau trên 0,6 m vẫn va nhau ở 54% cặp, vì đường đi qua giữa bàn |
| 2026-11-16 | **Hitbox, khi chạy thật và giá trị của mức phạt** | Trong M2 bản cuối, việc gắp có hitbox giao với việc đang làm của tay khác phải chờ (vùng dành riêng hoặc dRRT\* bắt chờ) 8,2% số lần (TB 0,08 s); có tay khác đang gắp nhưng không giao: 7,0% (0,09 s); không có tay nào khác đang gắp: 10,9% (0,10 s). Tranh chấp được dRRT\* và vùng dành riêng xử lý gần như không mất thời gian. Mức 1 với phạt 0 s: 6,99 khối/phút; phạt 3 s hơn +0,22 khối/phút, khoảng tin cậy 95% (bootstrap theo cặp cảnh) [−0,08; +0,51], **không phân biệt được với 0**. Phần tăng của mức 1 so với mức 0 đến từ chi phí theo bảng (gần, đúng khay), không phải từ phạt tranh chấp. Giữ phạt 3 s vì không hại |
| 2026-11-16 | **M3 – perception:** depth → điểm 3D; chỉ giữ điểm trên bàn ở độ cao mặt trên khối ±1 cm (bỏ khay, cánh tay, mặt bên khối); HSV (đỏ H 0–8 và 172–179, xanh lá 50–75, xanh dương 106–120, S ≥ 60, V ≥ 90); `minAreaRect` trên toạ độ xy thật thay vì pixel. Diện tích ≥ 85% kỳ vọng mới được cập nhật pose | Xanh của cánh tay UR có H ≈ 101–104, khay cùng màu khối nhưng thấp hơn: dải độ cao loại cả hai. Khối bị che một phần cho tâm lệch tới vài cm (ảnh `results/m3/perception_overlay.png`: 6,8 mm và 21 mm) nên không dùng để cập nhật |
| 2026-11-16 | **M3 – tracker và tích hợp:** ghép theo màu và khoảng cách < 4 cm; chỗ của bản ghi mà camera thấy mặt bàn trống 3 khung liên tiếp thì bỏ bản ghi. Camera 5 Hz thời gian mô phỏng. Bộ điều phối chỉ thấy bản ghi: pose, màu, kiểm tra đoạn đi xuống (khối đặt theo pose ước lượng). Gắp được hay không: bộ điều khiển đọc khớp driver của gripper (< 0,70 rad = đang kẹp vật; kẹp khối 5 cm cho 0,36 rad); pose thật chỉ dùng để đánh giá | Không để pose thật lọt vào quyết định. Hai má kẹp chạm nhau khi đóng hụt không tính là va chạm `self` |
| 2026-11-16 | **Kết quả M3** (50 cảnh, mức 1, pose từ camera) | 50/50, 400/400 đúng khay, **0 va chạm, gắp 400/400 (100%)**, 7,07 khối/phút (khác pose thật −0,14, khoảng tin cậy 95% [−0,13; +0,41]: không phân biệt được). Sai số pose lúc quyết định gắp (400): vị trí trung vị 0,12 mm, p95 0,78 mm, p99 1,25 mm, lớn nhất 4,6 mm; góc trung vị 0,02°, p95 1,7°, p99 3,4°, lớn nhất 5,8°; màu đúng 100%. Trên mọi khung hình (33.271 lần bản ghi–khối): p99 1,43 mm và 3,1°. Khi cell chạy, 20,4% lần khối xuất hiện trong ảnh bị che một phần (< 85% pixel), 6,4% bị che hẳn. Không bản ghi nào bị mất hay thừa; bộ điều khiển và pose thật không lần nào kết luận khác nhau về việc đang giữ vật |
| 2026-11-16 | **Độ chịu sai số của script gắp** (`experiments/grasp_tolerance.py`: 40 lần mỗi ô, cùng seed cho mọi ô, pose lệch đúng e mm theo hướng ngẫu nhiên và đúng a độ) | Tỉ lệ thành công, hàng = e (mm), cột = a (0°/10°/20°/30°/45°): 0: 1,00/1,00/1,00/1,00/0,03; 5: 1,00/1,00/0,60/0,33/0; 10: 0,95/0,62/0,35/0,20/0; 15: 0,62/0,35/0,33/0,25/0; 20: 0,40/0,28/0,20/0,03/0; 25: 0,23/0,23/0,10/0/0; 30: 0,28/0,10/0/0/0. Lỗi kết hợp nặng hơn từng lỗi riêng: lệch góc làm khối "rộng" hơn theo phương kẹp (50(cos a + sin a) mm, 64 mm ở 20° so với khe 70 mm) nên chỉ còn vài mm cho lệch vị trí. Không lần nào va chạm |
| 2026-11-16 | **Điểm quyết định M3: kích hoạt.** Script đạt 100% (≥ 95%) với perception trong sim, vì sai số perception (p99 1,25 mm / 3,4°) nằm sâu trong vùng 100% của bảng trên. Learning chưa có lý do ở độ khó hiện tại | **Chưa chọn cách tăng độ khó** (chờ quyết định). Đề xuất: nhiễu perception có căn cứ (sai số hiệu chuẩn camera, nhiễu depth, trễ) ở mức mà script còn khoảng 60–90%, theo bảng trên tương ứng sai số khoảng 10–15 mm hoặc 5 mm kèm 20°; đúng câu chuyện "policy dùng camera cổ tay sửa sai số vài cm cuối" của mục 7 |

## 13. Rủi ro

| Rủi ro | Dấu hiệu | Xử lý |
| --- | --- | --- |
| Gripper trượt, kẹp không chắc | M0 không đạt 20/20 | Đã qua ở M0 (20/20, mục 12). Còn lưu ý: phải đặt lại cone/impratio khi attach |
| Lập kế hoạch lệch giờ quá phức tạp | M2 kéo dài quá 2 tuần | Dùng đồng bộ theo lượt (phương án dự phòng 5.3) |
| Vật đang mang va vào tay khác | Có va chạm ở M2 | Kiểm tra lại phần gắn hình học vật vào gripper |
| Script đã đủ tốt, learning không có lý do | M3 ≥ 95% | **Đã xảy ra** (M3: 100%). Tăng độ khó trước M4 (mục 12, điểm quyết định M3) |
| Render ảnh chậm | Sinh 1 nghìn episode quá một đêm | Ảnh 84×84; chỉ render camera cổ tay ở pha gắp; chạy song song nhiều tiến trình |
| Giới hạn phiên của Colab và Kaggle | Huấn luyện bị ngắt | Checkpoint định kỳ; chạy tiếp từ checkpoint |
