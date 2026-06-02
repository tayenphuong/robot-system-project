#include <WiFi.h>

#include <WebServer.h>

#include <ESP32Servo.h>

#include <Wire.h>

#include <WebSocketsServer.h> // Cần cài thêm thư viện này



// ========== WiFi AP Settings ==========

const char* apSsid     = "ESP32_SLAM_Car";

const char* apPassword = "123456789";



WebServer server(80);

WebSocketsServer webSocket = WebSocketsServer(81); // WebSocket chạy ở cổng 81



// ========== Chân ngoại vi ==========

#define SERVO_PIN 13    

#define IN1 14 // Left Motor

#define IN2 27

#define IN3 26 // Right Motor

#define IN4 25

#define ENA_PIN 33

#define ENB_PIN 32

#define TRIG_PIN 4

#define ECHO_PIN 5

#define IR_PIN 34

#define ENCODER_PIN 35



const int MPU_ADDR = 0x68;

const int PWM_FREQ = 20000;  

const int PWM_RES  = 8;      



uint8_t manualSpeed = 220;    

volatile long encoderTicks = 0;

long lastEncoderTicks = 0;



void IRAM_ATTR ISR_countEncoder() {

  encoderTicks++;

}



// Điều khiển lệnh

bool forwardCmd = false, backCmd = false, leftCmd = false, rightCmd = false;

unsigned long lastDataSend = 0;

const unsigned long SEND_INTERVAL = 100; // Gửi dữ liệu lên Web mỗi 100ms (10Hz)



// ========== Giao diện Web tích hợp Canvas vẽ MAP SLAM ==========

const char index_html[] PROGMEM = R"=====(

<!DOCTYPE html>

<html>

<head>

  <meta charset="utf-8" />

  <title>ESP32 SLAM Monitor</title>

  <meta name="viewport" content="width=device-width, initial-scale=1">

  <style>

    body { font-family: Arial, sans-serif; text-align: center; background: #1a1a1a; color: #eee; margin: 0; padding: 10px; }

    .container { display: flex; flex-wrap: wrap; justify-content: center; gap: 20px; }

    #mapCanvas { background: #000; border: 2px solid #444; border-radius: 8px; box-shadow: 0 4px 10px rgba(0,0,0,0.5); }

    .btn { padding: 15px 25px; margin: 5px; font-size: 16px; border-radius: 8px; border: none; cursor: pointer; background: #333; color: #fff; min-width: 90px;}

    .btn:active { background: #555; }

    #status { color: #00ff00; font-weight: bold; }

  </style>

</head>

<body>

  <h1>ESP32 Real-time SLAM Map</h1>

  <p>Status: <span id="status">Connecting...</span> | Ticks: <span id="lblTicks">0</span></p>

 

  <div class="container">

    <div>

      <canvas id="mapCanvas" width="500" height="500"></canvas>

      <br>

      <button class="btn" onclick="resetMap()" style="background: #b71c1c;">Xóa bản đồ</button>

    </div>



    <div style="margin-top: 40px;">

      <h3>Control Pad</h3>

      <div><button class="btn" onmousedown="sendCmd('f',1)" onmouseup="sendCmd('f',0)" ontouchstart="sendCmd('f',1)" ontouchend="sendCmd('f',0)">▲ Forward</button></div>

      <div>

        <button class="btn" onmousedown="sendCmd('l',1)" onmouseup="sendCmd('l',0)" ontouchstart="sendCmd('l',1)" ontouchend="sendCmd('l',0)">◀ Left</button>

        <button class="btn" onmousedown="sendCmd('s',1)" onmouseup="sendCmd('s',0)">■ Stop</button>

        <button class="btn" onmousedown="sendCmd('r',1)" onmouseup="sendCmd('r',0)" ontouchstart="sendCmd('r',1)" ontouchend="sendCmd('r',0)">Right ▶</button>

      </div>

      <div><button class="btn" onmousedown="sendCmd('b',1)" onmouseup="sendCmd('b',0)" ontouchstart="sendCmd('b',1)" ontouchend="sendCmd('b',0)">▼ Backward</button></div>

    </div>

  </div>



  <script>

    let ws;

    const canvas = document.getElementById('mapCanvas');

    const ctx = canvas.getContext('2d');

   

    // Tọa độ và trạng thái gốc của xe (Nằm ở giữa Canvas)

    let carX = canvas.width / 2;

    let carY = canvas.height / 2;

    let carAngle = -Math.PI / 2; // Hướng thẳng đứng lên trên

   

    let obstaclePoints = []; // Mảng lưu các điểm vật cản đã quét được

    const CM_TO_PX = 1.5;    // Tỷ lệ quy đổi: 1cm ngoài đời = 1.5 pixel trên màn hình



    function initWebSocket() {

      // Kết nối tới WebSocket của ESP32 qua cổng 81

      ws = new WebSocket('ws://' + window.location.hostname + ':81/');

     

      ws.onopen = () => { document.getElementById('status').innerText = "Connected"; };

      ws.onclose = () => { document.getElementById('status').innerText = "Disconnected"; setTimeout(initWebSocket, 2000); };

     

      ws.onmessage = (event) => {

        const data = JSON.parse(event.data);

        document.getElementById('lblTicks').innerText = data.ticks;

       

        // 1. Cập nhật vị trí xe dựa trên độ lệch góc Gyro và xung Encoder

        // Quy đổi thô: 1 tick encoder tương đương di chuyển khoảng 0.5cm (tùy thông số bánh xe của bạn)

        let distanceMoved = data.dL * 0.5 * CM_TO_PX;

       

        // Cập nhật hướng quay (đổi từ deg/s sang radian cho mỗi chu kỳ 100ms)

        let angleSpeedRad = (data.gz / 131.0) * (Math.PI / 180.0);

        carAngle += angleSpeedRad * 0.1;



        // Tính toán vị trí x, y mới của xe trên lưới tọa độ phẳng

        carX += distanceMoved * Math.cos(carAngle);

        carY += distanceMoved * Math.sin(carAngle);



        // 2. Nếu cảm biến siêu âm phát hiện vật cản, tính toán tọa độ điểm vật cản đó

        if (data.dist > 0 && data.dist < 50) {

          let obsDistPx = data.dist * CM_TO_PX;

          // Vật cản nằm phía trước xe dọc theo hướng góc carAngle

          let obsX = carX + obsDistPx * Math.cos(carAngle);

          let obsY = carY + obsDistPx * Math.sin(carAngle);

          obstaclePoints.push({x: obsX, y: obsY});

        }

       

        drawMap();

      };

    }



    function drawMap() {

      // Xóa màn hình nền cũ

      ctx.clearRect(0, 0, canvas.width, canvas.height);

     

      // Vẽ lưới tọa độ ô vuông nền (Grid)

      ctx.strokeStyle = '#222';

      ctx.lineWidth = 1;

      for(let i=0; i<canvas.width; i+=25) {

        ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, canvas.height); ctx.stroke();

        ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(canvas.width, i); ctx.stroke();

      }



      // Vẽ tất cả các điểm vật cản đã quét được (Màu đỏ)

      ctx.fillStyle = '#ff3333';

      obstaclePoints.forEach(p => {

        ctx.beginPath();

        ctx.arc(p.x, p.y, 2, 0, 2 * Math.PI);

        ctx.fill();

      });



      // Vẽ quỹ đạo đường đi của xe (Màu xanh lá nhạt)

      ctx.fillStyle = '#00ff00';

      ctx.beginPath();

      ctx.arc(carX, carY, 6, 0, 2 * Math.PI);

      ctx.fill();



      // Vẽ mũi tên chỉ hướng của xe

      ctx.strokeStyle = '#fff';

      ctx.lineWidth = 3;

      ctx.beginPath();

      ctx.moveTo(carX, carY);

      ctx.lineTo(carX + 15 * Math.cos(carAngle), carY + 15 * Math.sin(carAngle));

      ctx.stroke();

    }



    function sendCmd(dir, state) {

      if(ws && ws.readyState === WebSocket.OPEN) {

        ws.send(JSON.stringify({cmd: dir, st: state}));

      }

    }



    function resetMap() {

      obstaclePoints = [];

      carX = canvas.width / 2;

      carY = canvas.height / 2;

      carAngle = -Math.PI / 2;

      drawMap();

    }



    window.onload = () => { initWebSocket(); drawMap(); };

  </script>

</body>

</html>

)=====";



// ========== Điều khiển Động cơ ==========

void setMotorSpeed(uint8_t leftSpeed, uint8_t rightSpeed) { ledcWrite(ENA_PIN, leftSpeed); ledcWrite(ENB_PIN, rightSpeed); }

void driveStop() { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); setMotorSpeed(0, 0); }

void driveForward(uint8_t speed) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); setMotorSpeed(speed, speed); }

void driveBackward(uint8_t speed) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); setMotorSpeed(speed, speed); }

void driveLeft(uint8_t speed) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); setMotorSpeed(speed, speed); }

void driveRight(uint8_t speed) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); setMotorSpeed(speed, speed); }



float getDistanceCm() {

  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 20000UL);

  if (duration == 0) return -1.0;

  return (duration * 0.0343) / 2.0;

}



// ========== Xử lý gói tin WebSocket nhận từ Web xuống ==========

void webSocketEvent(uint8_t num, WSType_t type, uint8_t * payload, size_t length) {

  if (type == WSType_TEXT) {

    String text = String((char*)payload);

    // Phân tích cú pháp thô chuỗi JSON lệnh để tối ưu tốc độ phản hồi xe

    if (text.indexOf("\"cmd\":\"f\"") > 0) { forwardCmd = text.indexOf("\"st\":1") > 0; if(forwardCmd) backCmd=leftCmd=rightCmd=false; }

    else if (text.indexOf("\"cmd\":\"b\"") > 0) { backCmd = text.indexOf("\"st\":1") > 0; if(backCmd) forwardCmd=leftCmd=rightCmd=false; }

    else if (text.indexOf("\"cmd\":\"l\"") > 0) { leftCmd = text.indexOf("\"st\":1") > 0; if(leftCmd) forwardCmd=backCmd=rightCmd=false; }

    else if (text.indexOf("\"cmd\":\"r\"") > 0) { rightCmd = text.indexOf("\"st\":1") > 0; if(rightCmd) forwardCmd=backCmd=leftCmd=false; }

    else if (text.indexOf("\"cmd\":\"s\"") > 0) { forwardCmd=backCmd=leftCmd=rightCmd=false; driveStop(); }

  }

}



void setup() {

  Serial.begin(115200);

  Wire.begin(21, 22);

  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);



  pinMode(IR_PIN, INPUT);

  pinMode(ENCODER_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), ISR_countEncoder, RISING);



  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);

  myServo.setPeriodHertz(50); myServo.attach(SERVO_PIN, 500, 2400); myServo.write(90);



  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(ENA_PIN, OUTPUT); pinMode(ENB_PIN, OUTPUT);

  ledcAttach(ENA_PIN, PWM_FREQ, PWM_RES); ledcAttach(ENB_PIN, PWM_FREQ, PWM_RES);

  driveStop();



  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);



  WiFi.mode(WIFI_AP);

  WiFi.softAP(apSsid, apPassword);



  server.on("/", []() { server.send_P(200, "text/html", index_html); });

  server.begin();



  webSocket.begin();

  webSocket.onEvent(webSocketEvent);

}



void loop() {

  server.handleClient();

  webSocket.loop();



  // Xử lý di chuyển bằng tay từ lệnh nhận được qua WebSocket

  if (forwardCmd)       driveForward(manualSpeed);

  else if (backCmd)     driveBackward(manualSpeed);

  else if (leftCmd)     driveLeft(manualSpeed);

  else if (rightCmd)    driveRight(manualSpeed);

  else if (!forwardCmd && !backCmd && !leftCmd && !rightCmd) driveStop();



  // GỬI DỮ LIỆU ĐỊNH VỊ SLAM LÊN WEB ĐỊNH KỲ

  if (millis() - lastDataSend >= SEND_INTERVAL) {

    float d = getDistanceCm();

   

    // Đọc trục Z của Gyro (để tính góc quay xe)

    Wire.beginTransmission(MPU_ADDR); Wire.write(0x47); Wire.endTransmission(false);

    Wire.requestFrom(MPU_ADDR, 2, true);

    int16_t rawGyZ = Wire.read() << 8 | Wire.read();



    // Tính toán độ lệch xung Encoder trong chu kỳ qua

    long deltaTicks = encoderTicks - lastEncoderTicks;

    lastEncoderTicks = encoderTicks;



    // Đóng gói JSON chuỗi ngắn gọn để truyền nhanh qua WebSocket

    String json = "{\"ticks\":" + String(encoderTicks) +

                  ",\"dL\":" + String(deltaTicks) +

                  ",\"gz\":" + String(rawGyZ) +

                  ",\"dist\":" + String(d) + "}";

                 

    webSocket.broadcastTXT(json); // Phát tín hiệu tới trình duyệt Web

    lastDataSend = millis();

  }

  delay(1);

}
