Bản obstacle-line-v3 + SLAM frontier/Manhattan

Mục tiêu bản này:
- AUTO chỉ lùi khi sonar đang nhìn thẳng trước và đo vật cản <= 30 cm.
- Frontend dùng occupancy grid để vẽ vùng trống, vật cản và đường robot đã đi.
- Vật cản sonar được vẽ thành các line nhỏ và lưu theo trường obstacleLines.
- Thêm frontier exploration: web tìm vùng ranh giữa ô đã biết và ô chưa biết.
- Thêm obstacle inflation: phóng to vật cản theo bán kính robot để tránh đi sát tường.
- Thêm BFS/A* nhẹ trên lưới đã biết để chọn waypoint đầu tiên rồi gửi lệnh goto cho ESP32.
- Thêm Manhattan wall alignment: nếu nhiều vệt tường tạo thành hai hướng vuông góc, web xoay nhẹ map từng bước để map thẳng/vuông hơn.
- Khi đi theo waypoint, firmware đi đoạn ngắn hơn rồi quét lại để giảm overshoot.
- Tốc độ AUTO cố định: autoSpeed = 30, AUTO_MIN_SPEED = 30.
- Thanh chỉnh tốc độ trên web giới hạn 30-40, mặc định 30.
- Tốc độ lùi AUTO: BACK_SPEED = 20.
- Sửa lỗi lùi oan sau khi quét: AUTO chỉ lùi khi có ít nhất 2 mẫu sonar mới, đúng góc 90 độ, và đều cho thấy vật cản <=30 cm. Số đo 999 hoặc >30 cm sẽ xóa trạng thái vật cản và cho robot tiến tiếp.
- AUTO mặc định không dùng cảm biến cliff để kích lùi vì chân này dễ nhiễu/floating; nếu cần dùng lại, đổi USE_CLIFF_GUARD=True trong firmware/main.py.
- Chạy mượt hơn: bỏ kick PWM 255, dùng ramp PWM mềm, giảm PID giữ thẳng và tắt phanh đảo chiều mạnh.
- Tốc độ web 30-40 là tốc độ logic; firmware quy đổi sang PWM thật có sàn lực: FWD_PWM_MIN=60, BACK_PWM_MIN=58, TURN_PWM_MIN=64. Nếu xe vẫn yếu, tăng từng số này thêm 5.
- Né vật cản tốt hơn: nếu trái/phải còn thoáng thì xoay tại chỗ, không lùi trước; chỉ lùi khi trước/trái/phải đều bí và phía sau trống; nếu sau cũng bí thì đứng chờ/quét lại.
- IR trước: debounce rồi chỉ dừng/cho quét lại, không tự kích lùi. IR sau: debounce rồi chặn lùi để không đâm về sau.
- Giảm lệch map: HEADING_GAIN = 1.0, bỏ qua góc gyro nhỏ khi xe đi thẳng/lùi, ODOM_CM_PER_TICK = 0.35 và bỏ qua tick encoder rất nhỏ. Nếu map vẫn dài/ngắn so với thực tế, chỉnh ODOM_CM_PER_TICK trong frontend/index.html.

File cần nạp ESP32: firmware/main.py
File giao diện: frontend/index.html
Chạy server: uvicorn server:app --host 0.0.0.0 --port 8000

Ghi chú:
- Đây vẫn là mapping bằng 1 sonar trên servo, không phải SLAM đầy đủ kiểu LiDAR.
- Map sẽ vuông hơn nhờ line fitting/Manhattan alignment, nhưng còn phụ thuộc gyro, encoder, mặt sàn và nhiễu sonar.
- Nếu map bị xoay quá tay, giảm MANHATTAN_MAX_STEP hoặc tăng MANHATTAN_MIN_LINES trong frontend/index.html.
- Đã kiểm trên dữ liệu thật trong zip: 585 session, frontier tìm hướng được 396 session; 181 session có đủ điểm sonar line; 43 session đủ tin cậy để tự chỉnh Manhattan. Các session/map quá ít vật cản hoặc robot gần như đứng yên sẽ không bị ép xoay để giữ ổn định.
