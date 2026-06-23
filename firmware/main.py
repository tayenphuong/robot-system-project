# ============================================================================
#  ESP32 SLAM CAR  -  MicroPython (Thonny)  -  CHẾ ĐỘ STA + WEBSOCKET CLIENT
#  Robot nối router (có internet) và kết nối RA server FastAPI -> dùng được
#  qua web công khai, và server ghi lại phiên chạy.
#  Giữ nguyên: 2 encoder + PID đi thẳng, median filter, tránh vật cản mềm,
#  frontTrust (không mù), mv (chiều), chỉnh tốc độ từ web.
#  Nạp DUY NHẤT file này lên ESP32 bằng Thonny (đặt tên main.py).
# ============================================================================

import network, time, usocket as socket, ustruct as struct, ubinascii, uos as os, uselect
from machine import Pin, PWM, I2C, time_pulse_us
try:
    import ujson as json
except ImportError:
    import json

# ===================== CẤU HÌNH =====================
WIFI_SSID = "ESP_SLAM_CAR"       # <-- tên hotspot của laptop
WIFI_PASS = "123456789"          # <-- mật khẩu hotspot laptop
SERVER_HOST = "192.168.137.1"    # <-- IP laptop trên hotspot (xác nhận bằng ipconfig)
SERVER_PORT = 8000
SERVER_PATH = "/ws/robot"

# ===================== WebSocket CLIENT (tự chứa) =====================
class WSClient:
    def __init__(self, host, port, path):
        self.host, self.port, self.path = host, port, path
        self.sock = None; self.poll = None; self.connected = False
    def connect(self):
        self.close()
        ai = socket.getaddrinfo(self.host, self.port)[0][-1]
        self.sock = socket.socket(); self.sock.settimeout(8); self.sock.connect(ai)
        key = ubinascii.b2a_base64(os.urandom(16)).strip().decode()
        req = ("GET %s HTTP/1.1\r\nHost: %s:%d\r\nUpgrade: websocket\r\n"
               "Connection: Upgrade\r\nSec-WebSocket-Key: %s\r\n"
               "Sec-WebSocket-Version: 13\r\n\r\n") % (self.path, self.host, self.port, key)
        self.sock.send(req.encode())
        resp = b""
        while b"\r\n\r\n" not in resp:
            c = self.sock.recv(128)
            if not c: raise OSError("handshake fail")
            resp += c
            if len(resp) > 2048: raise OSError("header too long")
        if b"101" not in resp.split(b"\r\n", 1)[0]: raise OSError("upgrade refused")
        self.sock.settimeout(None); self.sock.setblocking(True)
        self.poll = uselect.poll(); self.poll.register(self.sock, uselect.POLLIN)
        self.connected = True
    def _rd(self, n):
        b = b""
        while len(b) < n:
            c = self.sock.recv(n - len(b))
            if not c: raise OSError("closed")
            b += c
        return b
    def send(self, data):
        if not self.connected: return
        p = data.encode() if isinstance(data, str) else data
        n = len(p); f = bytearray([0x81])
        if n < 126: f.append(0x80 | n)
        elif n < 65536: f.append(0x80 | 126); f.extend(struct.pack(">H", n))
        else: f.append(0x80 | 127); f.extend(struct.pack(">Q", n))
        m = os.urandom(4); f.extend(m)
        f.extend(bytes(p[i] ^ m[i & 3] for i in range(n)))
        try: self.sock.send(f)
        except OSError: self.connected = False; raise
    def recv(self):
        if not self.connected or not self.poll.poll(0): return None
        try:
            b1 = self._rd(1)[0]; b2 = self._rd(1)[0]
            op = b1 & 0x0F; masked = b2 & 0x80; ln = b2 & 0x7F
            if ln == 126: ln = struct.unpack(">H", self._rd(2))[0]
            elif ln == 127: ln = struct.unpack(">Q", self._rd(8))[0]
            mk = self._rd(4) if masked else None
            pl = self._rd(ln) if ln else b""
            if masked and pl: pl = bytes(pl[i] ^ mk[i & 3] for i in range(ln))
            if op == 0x8: self.connected = False; return None
            if op == 0x9:  # ping -> pong
                self.sock.send(bytes([0x8A, 0x80 | len(pl)]) + os.urandom(4)); return None
            if op == 0xA: return None
            return pl.decode()
        except OSError:
            self.connected = False; return None
    def close(self):
        if self.sock:
            try: self.sock.close()
            except Exception: pass
        self.sock = None; self.poll = None; self.connected = False

# ===================== Chân ngoại vi =====================
IN1=Pin(14,Pin.OUT); IN2=Pin(27,Pin.OUT); IN3=Pin(26,Pin.OUT); IN4=Pin(25,Pin.OUT)
ena=PWM(Pin(33),freq=1000); ena.duty_u16(0)
enb=PWM(Pin(32),freq=1000); enb.duty_u16(0)
trig=Pin(4,Pin.OUT); echo=Pin(5,Pin.IN)               # MỘT sonar duy nhất, gắn trên servo
ir_cliff=Pin(35,Pin.IN); ir_rear=Pin(34,Pin.IN)
encoderL=Pin(23,Pin.IN,Pin.PULL_UP); encoderR=Pin(19,Pin.IN,Pin.PULL_UP)
servo=PWM(Pin(13),freq=50)
i2c=I2C(0,scl=Pin(22),sda=Pin(21),freq=400000); MPU_ADDR=0x68
try: i2c.writeto_mem(MPU_ADDR,0x6B,b'\x00')
except Exception as e: print("MPU?",e)

# ===================== Tham số =====================
manualSpeed=200; autoSpeed=160; AUTO_MIN_SPEED=90; motorBalance=0
KP_STRAIGHT=6; STRAIGHT_CORR_MAX=45
SAFE_DISTANCE=20.0; SLOW_DISTANCE=45.0
AUTO_STOP_DIST=25.0; AUTO_CLEAR_DIST=35.0; STUCK_DIST=14.0
SERVO_MIN,SERVO_MAX,SERVO_STEP=30,150,6
SERVO_SETTLE=250; TURN_MIN=250; TURN_MAX=1800; BACKUP_SHORT=450; BACKUP_LONG=800
# Vùng góc HẸP coi là "thẳng trước" để quyết định đi/dừng (không khựng vì vật chéo)
FRONT_NARROW_LO=72; FRONT_NARROW_HI=108

ticksL=ticksR=0; lastTicksL=lastTicksR=0
frontDistance=999.0; frontTrust=999.0; frontGuard=999.0
cliffAhead=False; rearObstacle=False
servoAngle=90; servoDir=1; motionDir=0
autoMode=False; autoState=0; autoTimer=0; backupTime=0; distLeft=0.0; distRight=0.0; turnDir=1
forwardCmd=backCmd=leftCmd=rightCmd=False
_dist_buf=[]; _front_samples=[]
_straight_active=False; _baseL=0; _baseR=0
_prev_moving=False; _kick_until=0; KICK_MS=130   # "đá ga" 130ms khi khởi động chống kẹt

def _isr_L(p):
    global ticksL; ticksL+=1
def _isr_R(p):
    global ticksR; ticksR+=1
encoderL.irq(trigger=Pin.IRQ_RISING,handler=_isr_L)
encoderR.irq(trigger=Pin.IRQ_RISING,handler=_isr_R)

# ===================== Động cơ / servo =====================
def _duty(v):
    if v<0:v=0
    if v>255:v=255
    return v*257
def _kick_check(moving):
    # Khi vừa chuyển từ đứng yên -> chuyển động, đá ga đầy trong KICK_MS để thắng
    # ma sát tĩnh (chống stall/kêu ù mà không quay). Trả True khi đang trong cửa sổ kick.
    global _prev_moving,_kick_until
    now=time.ticks_ms()
    if moving and not _prev_moving:
        _kick_until=time.ticks_add(now,KICK_MS)
    _prev_moving=moving
    return time.ticks_diff(_kick_until,now)>0
def driveStop():
    global motionDir; motionDir=0
    _kick_check(False)
    IN1.value(0);IN2.value(0);IN3.value(0);IN4.value(0); ena.duty_u16(0); enb.duty_u16(0)
def _straight_reset():
    global _straight_active,_baseL,_baseR; _straight_active=True; _baseL=ticksL; _baseR=ticksR
def driveForwardStraight(speed):
    global motionDir; motionDir=1
    kick=_kick_check(True)
    dl=ticksL-_baseL; dr=ticksR-_baseR; err=dl-dr; corr=KP_STRAIGHT*err
    if corr>STRAIGHT_CORR_MAX:corr=STRAIGHT_CORR_MAX
    if corr<-STRAIGHT_CORR_MAX:corr=-STRAIGHT_CORR_MAX
    ls=speed-corr; rs=speed+corr
    if motorBalance>0: ls-=motorBalance
    elif motorBalance<0: rs+=motorBalance
    if kick: ls=rs=255
    IN1.value(1);IN2.value(0);IN3.value(1);IN4.value(0); ena.duty_u16(_duty(ls)); enb.duty_u16(_duty(rs))
def driveBackward(s):
    global motionDir; motionDir=-1
    if _kick_check(True): s=255
    ls=rs=s
    if motorBalance>0:ls=s-motorBalance
    elif motorBalance<0:rs=s+motorBalance
    IN1.value(0);IN2.value(1);IN3.value(0);IN4.value(1); ena.duty_u16(_duty(ls)); enb.duty_u16(_duty(rs))
def driveLeft(s):
    global motionDir; motionDir=0
    if _kick_check(True): s=255
    IN1.value(0);IN2.value(1);IN3.value(1);IN4.value(0); ena.duty_u16(_duty(s)); enb.duty_u16(_duty(s))
def driveRight(s):
    global motionDir; motionDir=0
    if _kick_check(True): s=255
    IN1.value(1);IN2.value(0);IN3.value(0);IN4.value(1); ena.duty_u16(_duty(s)); enb.duty_u16(_duty(s))
def setServoAngle(a):
    if a<0:a=0
    if a>180:a=180
    us=500+(a/180.0)*1900.0; servo.duty_u16(int(us/20000.0*65535))
setServoAngle(servoAngle)

def readDistanceOnce():
    trig.value(0); time.sleep_us(2); trig.value(1); time.sleep_us(10); trig.value(0)
    d=time_pulse_us(echo,1,8000)
    return -1.0 if d<0 else (d*0.0343)/2.0
def readGyroZ():
    try:
        d=i2c.readfrom_mem(MPU_ADDR,0x47,2); r=(d[0]<<8)|d[1]
        return r-65536 if r>32767 else r
    except Exception: return 0
def canMoveForward():
    return (frontGuard>SAFE_DISTANCE) and (not cliffAhead)

def handle_command(text):
    global autoMode,autoState,servoAngle,motorBalance,manualSpeed
    global forwardCmd,backCmd,leftCmd,rightCmd
    try: d=json.loads(text)
    except Exception: return
    c=d.get("cmd"); st=(d.get("st")==1)
    if c=="a":
        autoMode=st; forwardCmd=backCmd=leftCmd=rightCmd=False; autoState=0
        servoAngle=90; setServoAngle(90)
        if not autoMode: driveStop()
    elif c=="speed":
        try: manualSpeed=max(60,min(255,int(d.get("val",manualSpeed))))
        except Exception: pass
    elif c=="trim":
        try: motorBalance=max(-120,min(120,int(d.get("val",0))))
        except Exception: pass
    elif c=="f":
        forwardCmd=st
        if forwardCmd: backCmd=leftCmd=rightCmd=False
    elif c=="b":
        backCmd=st
        if backCmd: forwardCmd=leftCmd=rightCmd=False
    elif c=="l":
        leftCmd=st
        if leftCmd: forwardCmd=backCmd=rightCmd=False
    elif c=="r":
        rightCmd=st
        if rightCmd: forwardCmd=backCmd=leftCmd=False
    elif c=="s":
        forwardCmd=backCmd=leftCmd=rightCmd=False; driveStop()

def runAuto():
    global autoState,autoTimer,servoAngle,servoDir,distLeft,distRight,turnDir,backupTime,_straight_active
    now=time.ticks_ms()
    if autoState==0:
        if cliffAhead or frontGuard<AUTO_STOP_DIST:
            driveStop(); _straight_active=False
            servoAngle=SERVO_MAX; setServoAngle(servoAngle); autoTimer=now; autoState=1
        else:
            if frontGuard>=SLOW_DISTANCE: spd=autoSpeed
            else:
                r=(frontGuard-AUTO_STOP_DIST)/(SLOW_DISTANCE-AUTO_STOP_DIST)
                if r<0:r=0
                spd=int(AUTO_MIN_SPEED+r*(autoSpeed-AUTO_MIN_SPEED))
            if not _straight_active: _straight_reset()
            driveForwardStraight(spd)
    elif autoState==1:
        driveStop()
        if time.ticks_diff(now,autoTimer)>=SERVO_SETTLE:
            distLeft=frontDistance; servoAngle=SERVO_MIN; setServoAngle(servoAngle); autoTimer=now; autoState=2
    elif autoState==2:
        if time.ticks_diff(now,autoTimer)>=SERVO_SETTLE:
            distRight=frontDistance; servoAngle=90; setServoAngle(servoAngle); autoTimer=now; autoState=3
    elif autoState==3:
        if time.ticks_diff(now,autoTimer)>=SERVO_SETTLE:
            if distLeft>=AUTO_CLEAR_DIST:        # ƯU TIÊN TRÁI: trái thoáng -> rẽ trái
                turnDir=-1; autoState=5
            elif distRight>=AUTO_CLEAR_DIST:     # trái bí -> xét phải
                turnDir=1; autoState=5
            else:                                # cả hai bên đều bí -> LÙI rồi rẽ về bên đỡ bí
                turnDir=-1 if distLeft>=distRight else 1
                backupTime=BACKUP_LONG; autoState=4
            autoTimer=now
    elif autoState==4:
        if rearObstacle: driveStop(); autoState=5; autoTimer=now
        else:
            driveBackward(autoSpeed)
            if time.ticks_diff(now,autoTimer)>=backupTime: autoState=5; autoTimer=now
    elif autoState==5:
        # rẽ CHO TỚI KHI phía trước thật sự thoáng (servo ~90 nên frontGuard = thẳng trước),
        # rẽ tối thiểu TURN_MIN để không thoát sớm do nhiễu, tối đa TURN_MAX để không quay mãi.
        driveLeft(autoSpeed) if turnDir<0 else driveRight(autoSpeed)
        el=time.ticks_diff(now,autoTimer)
        if (frontGuard>AUTO_CLEAR_DIST and el>=TURN_MIN) or el>=TURN_MAX:
            autoState=0; servoDir=1; _straight_active=False   # vào FORWARD, đi 1 mạch dài để khám phá

# ===================== WiFi =====================
def wifi_connect():
    # Tắt AP (phòng khi trước đó nạp bản tự-phát-wifi) -> tránh xung đột trạng thái
    try:
        ap = network.WLAN(network.AP_IF); ap.active(False)
    except Exception:
        pass
    w = network.WLAN(network.STA_IF)
    # reset sạch trạng thái WLAN
    w.active(False); time.sleep_ms(300)
    w.active(True);  time.sleep_ms(300)
    try:
        w.disconnect()
    except Exception:
        pass
    time.sleep_ms(200)
    print("WiFi", end="")
    try:
        w.connect(WIFI_SSID, WIFI_PASS)
    except OSError:
        w.active(False); time.sleep_ms(400); w.active(True); time.sleep_ms(300)
        w.connect(WIFI_SSID, WIFI_PASS)
    t = time.ticks_ms()
    while not w.isconnected():
        print(".", end=""); time.sleep_ms(400)
        if time.ticks_diff(time.ticks_ms(), t) > 15000:
            # quá lâu -> reset rồi thử lại (KHÔNG gọi connect liên tục khi đang nối)
            w.active(False); time.sleep_ms(400); w.active(True); time.sleep_ms(300)
            try:
                w.connect(WIFI_SSID, WIFI_PASS)
            except OSError:
                pass
            t = time.ticks_ms()
    print("\nIP:", w.ifconfig()[0])
    return w

# ===================== MAIN LOOP =====================
def main():
    global frontDistance,frontTrust,frontGuard,cliffAhead,rearObstacle
    global servoAngle,servoDir,lastTicksL,lastTicksR,_straight_active,_dist_buf,_front_samples
    driveStop(); wifi_connect()
    ws=WSClient(SERVER_HOST,SERVER_PORT,SERVER_PATH)
    last_sonar=last_servo=last_send=time.ticks_ms()
    while True:
        if not ws.connected:
            try: print("Kết nối server..."); ws.connect(); print("OK")
            except Exception as e:
                print("Lỗi:",e); driveStop(); time.sleep_ms(2000); continue
        now=time.ticks_ms()
        # SERVO theo kiểu một-sonar: đang đi -> NHÌN THẲNG canh trước (không mù);
        # đứng yên -> quét rộng để dựng map. Auto đang ngó trái/phải -> FSM tự giữ servo.
        if time.ticks_diff(now,last_servo)>=55:
            fsm=autoMode and autoState!=0
            if not fsm:
                if motionDir!=0:                       # đang di chuyển -> thẳng 90
                    if servoAngle!=90: servoAngle=90; setServoAngle(90)
                    servoDir=1
                else:                                  # đứng yên -> quét rộng 30-150
                    servoAngle+=servoDir*SERVO_STEP
                    if servoAngle>=SERVO_MAX: servoAngle=SERVO_MAX; servoDir=-1
                    if servoAngle<=SERVO_MIN: servoAngle=SERVO_MIN; servoDir=1
                    setServoAngle(servoAngle)
            last_servo=now
        # sonar (một con trên servo) + median + frontTrust = vật gần nhất thẳng trước trong 600ms
        if time.ticks_diff(now,last_sonar)>=35:
            d=readDistanceOnce()
            if d>0:
                _dist_buf.append(d)
                if len(_dist_buf)>5: _dist_buf.pop(0)
            frontDistance=sorted(_dist_buf)[len(_dist_buf)//2] if _dist_buf else 999.0
            # khi đang đi servo nhìn thẳng (90) -> luôn nằm trong vùng hẹp -> frontTrust tin cậy
            if FRONT_NARROW_LO<=servoAngle<=FRONT_NARROW_HI and 0<frontDistance<400:
                _front_samples.append((now,frontDistance))
            _front_samples=[(t,v) for (t,v) in _front_samples if time.ticks_diff(now,t)<600]
            frontTrust=min(v for (t,v) in _front_samples) if _front_samples else 999.0
            frontGuard=frontTrust
            cliffAhead=(ir_cliff.value()==1)
            rearObstacle=(ir_rear.value()==0)
            last_sonar=now
        # điều khiển
        if autoMode:
            runAuto()
        else:
            spd=manualSpeed
            if SAFE_DISTANCE<frontGuard<SLOW_DISTANCE: spd=manualSpeed//2
            if forwardCmd and canMoveForward():
                if not _straight_active: _straight_reset()
                driveForwardStraight(spd)
            elif backCmd and (not rearObstacle): _straight_active=False; driveBackward(manualSpeed)
            elif leftCmd: _straight_active=False; driveLeft(manualSpeed)
            elif rightCmd: _straight_active=False; driveRight(manualSpeed)
            else: _straight_active=False; driveStop()
        # nhận lệnh
        msg=ws.recv()
        if msg is not None: handle_command(msg)
        # gửi telemetry
        if time.ticks_diff(now,last_send)>=150:
            gz=readGyroZ()
            dnL=ticksL-lastTicksL; lastTicksL=ticksL
            dnR=ticksR-lastTicksR; lastTicksR=ticksR
            dL=(dnL+dnR)//2
            if autoMode: safe=0 if (cliffAhead or frontGuard<AUTO_STOP_DIST) else 1
            else: safe=1 if canMoveForward() else 0
            pkt={"ticks":ticksL+ticksR,"dL":dL,"dnL":dnL,"dnR":dnR,"gz":gz,
                 "dist":round(frontDistance,1),"ang":servoAngle,"mv":motionDir,
                 "ast":autoState,"spd":manualSpeed,"safe":safe,"rear":1 if rearObstacle else 0,
                 "auto":1 if autoMode else 0}
            try: ws.send(json.dumps(pkt))
            except Exception: ws.connected=False
            last_send=now
        time.sleep_ms(2)

if __name__=="__main__":
    try: main()
    except KeyboardInterrupt: driveStop(); print("Dừng.")
