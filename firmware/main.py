# ============================================================================
#  ESP32 SLAM CAR  -  MicroPython (Thonny)  -  CHẾ ĐỘ STA + WEBSOCKET CLIENT
#  Robot nối router (có internet) và kết nối RA server FastAPI -> dùng được
#  qua web công khai, và server ghi lại phiên chạy.
#  Giữ nguyên: 2 encoder + PID đi thẳng, median filter, tránh vật cản mềm,
#  frontTrust (không mù), mv (chiều), chỉnh tốc độ từ web.
#  Nạp DUY NHẤT file này lên ESP32 bằng Thonny (đặt tên main.py).
#  Lưu map 
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
manualSpeed=30; autoSpeed=30; AUTO_MIN_SPEED=30; motorBalance=0   # AUTO giữ tốc độ chạy 30 để quét map ổn định hơn
# LƯU Ý: bản này khóa tốc độ chạy mặc định 30 và lùi 20 để robot quét map chậm, ít vọt hơn.
# Nếu motor yếu không quay nổi ở PWM 20 khi lùi, tăng BACK_SPEED từng bước 20 -> 25 -> 30.
BACK_SPEED=20
KP_STRAIGHT=4; STRAIGHT_CORR_MAX=12
STRAIGHT_RESET_MS=350
SAFE_DISTANCE=20.0; SLOW_DISTANCE=60.0
AUTO_STOP_DIST=30.0; AUTO_CLEAR_DIST=40.0; STUCK_DIST=12.0   # <=30cm mới coi là vật chắn phía trước
USE_CLIFF_GUARD=False        # False: AUTO chỉ lùi theo sonar <=30cm, không lùi vì cảm biến cliff nhiễu/floating
USE_FRONT_IR_STOP=True       # IR trước chỉ dừng/cho quét lại, không tự kích lùi
# STRICT_FRONT_ONLY=True: AUTO chỉ lùi khi sonar đang nhìn đúng hướng trước và đo <=30cm.
# Các số đo xa hơn 30cm được xem là ĐƯỜNG THOÁNG, không được giữ trong buffer để khỏi lỗi cứ lùi.
STRICT_FRONT_ONLY=True
SERVO_MIN,SERVO_MAX,SERVO_STEP=30,150,6
SERVO_SETTLE=300; TURN_MIN=360; TURN_MAX=1500; BACKUP_SHORT=180; BACKUP_LONG=320  # lùi ít hơn, ưu tiên xoay tại chỗ
DRIVE_MS=1300; SCAN_MS=1100
GOTO_DRIVE_MS=850; ALIGN_TURN_MAX=2200
BRAKE_MS=0
PWM_RAMP_STEP=5; KICK_MS=100; KICK_LOGIC_SPEED=40
FWD_PWM_MIN=60; BACK_PWM_MIN=58; TURN_PWM_MIN=64; MOTOR_PWM_CAP=95
GYRO_STRAIGHT_DEADBAND_DPS=3.0; GYRO_STILL_DEADBAND_DPS=1.2
IR_DEBOUNCE_COUNT=3
# Vùng góc RẤT HẸP coi là "thẳng trước" để quyết định đi/dừng.
# Thu hẹp từ 72-108 xuống 85-95 để khi servo quét lệch trái/phải không kích hoạt lùi nhầm.
FRONT_NARROW_LO=85; FRONT_NARROW_HI=95

ticksL=ticksR=0; lastTicksL=lastTicksR=0
_last_ticksL_loop=0; _last_ticksR_loop=0
_signed_dist_accum=0.0    # quãng đường (đơn vị tick) dồn tích CÓ DẤU, cộng theo motionDir TẠI ĐÚNG THỜI ĐIỂM phát sinh tick
                          # -> tránh lỗi "mv chụp nhanh lúc gửi gói không khớp với lúc tick thực sự phát sinh"
frontDistance=999.0; frontTrust=999.0; frontGuard=999.0
cliffAhead=False; rearObstacle=False
servoAngle=90; servoDir=1; motionDir=0
autoMode=False; autoState=0; autoTimer=0; backupTime=0; distLeft=0.0; distRight=0.0; turnDir=1
frontIrBlocked=False
_next_auto_side=1   # 1=ưu tiên phải trước, -1=trái; sau mỗi lần gặp vật cản thì luân phiên
_last_turn_dir=0; _same_turn_count=0     # theo dõi để phá vòng lặp khám phá (rẽ hoài 1 hướng)
_drive_target_ms=1300                    # thời gian đi thẳng mỗi đoạn, random hóa mỗi lần để tránh đi đúng 1 quỹ đạo lặp lại
_goto_rel_deg=None; _goto_ts=0; _GOTO_VALID_MS=6000   # gợi ý hướng khám phá từ web + hạn dùng (tránh dùng gợi ý cũ nếu mất kết nối)
_align_accum_deg=0.0                     # tích lũy góc đã quay được trong lúc đang xoay theo hướng gợi ý (state ALIGN)
forwardCmd=backCmd=leftCmd=rightCmd=False
_dist_buf=[]; _front_samples=[]
_dist_buf_angle=None      # góc servo mà lô mẫu _dist_buf hiện tại thuộc về (tránh trộn khoảng cách của các góc khác nhau)
_front_measure_ms=0       # thời điểm có mẫu sonar thật sự ở vùng thẳng trước
_heading_accum_deg=0.0    # góc quay dồn tích từ lần gửi telemetry trước, tính bằng tích phân LIÊN TỤC (không phụ thuộc chu kỳ gửi)
_last_gyro_ms=0
_bodyStationary=True      # thân robot có thực sự đứng yên hay không (khác motionDir=0: pivot-turn cũng =0 nhưng thân đang xoay)
_straight_active=False; _baseL=0; _baseR=0; _straight_last_reset=0
_prev_moving=False; _kick_until=0
_pwmL=0; _pwmR=0; _driveMode="stop"
_ir_front_cnt=0; _ir_rear_cnt=0

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
def _ramp_to(cur,target):
    if target<0: target=0
    if target>255: target=255
    if target>cur+PWM_RAMP_STEP: return cur+PWM_RAMP_STEP
    if target<cur-PWM_RAMP_STEP: return cur-PWM_RAMP_STEP
    return target
def _logic_to_pwm(v,mode):
    # Web/logic speed 30-40 is intentionally slow. DC motors usually need
    # a higher real PWM floor to overcome static friction, so map it here.
    if v<=0: return 0
    if mode=="back":
        pwm=BACK_PWM_MIN+(v-BACK_SPEED)*1.6
    elif mode=="left" or mode=="right":
        pwm=TURN_PWM_MIN+(v-30)*2.0
    else:
        pwm=FWD_PWM_MIN+(v-30)*2.0
        if v<30: pwm=FWD_PWM_MIN-(30-v)*0.8
    if pwm<0: pwm=0
    if pwm>MOTOR_PWM_CAP: pwm=MOTOR_PWM_CAP
    return int(pwm)
def _set_pwm_pair(ls,rs,mode):
    global _pwmL,_pwmR,_driveMode
    if _driveMode!=mode:
        _pwmL=0; _pwmR=0; _driveMode=mode
    _pwmL=_ramp_to(_pwmL,_logic_to_pwm(ls,mode))
    _pwmR=_ramp_to(_pwmR,_logic_to_pwm(rs,mode))
    ena.duty_u16(_duty(_pwmL)); enb.duty_u16(_duty(_pwmR))
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
    global motionDir,_bodyStationary,_pwmL,_pwmR,_driveMode; motionDir=0; _bodyStationary=True
    _kick_check(False)
    _pwmL=0; _pwmR=0; _driveMode="stop"
    IN1.value(0);IN2.value(0);IN3.value(0);IN4.value(0); ena.duty_u16(0); enb.duty_u16(0)
def driveBrake(ms):
    # Phanh chủ động: nếu đang tiến, đảo chiều ngắn (BRAKE_MS) để triệt bớt quán tính trước khi
    # cắt hẳn điện. Chỉ cắt điện đơn thuần (driveStop) khiến robot trôi thêm 1 đoạn theo trớn,
    # dễ vượt qua ngưỡng an toàn AUTO_STOP_DIST và đâm vào vật -> đây là phanh CÓ chủ đích.
    global motionDir
    if ms>0 and motionDir==1:
        IN1.value(0);IN2.value(1);IN3.value(0);IN4.value(1)
        ena.duty_u16(_duty(60)); enb.duty_u16(_duty(60))
        time.sleep_ms(ms)
    driveStop()
def _straight_reset():
    global _straight_active,_baseL,_baseR,_straight_last_reset
    _straight_active=True; _baseL=ticksL; _baseR=ticksR; _straight_last_reset=time.ticks_ms()
def driveForwardStraight(speed):
    global motionDir,_bodyStationary,_baseL,_baseR,_straight_last_reset
    motionDir=1; _bodyStationary=False
    now=time.ticks_ms()
    # Reset lại mốc so sánh tick mỗi STRAIGHT_RESET_MS: nếu không, sai lệch trái/phải dồn tích
    # KHÔNG GIỚI HẠN suốt cả đoạn DRIVE_MS (giống khâu tích phân), khiến hệ số KP cao dễ overshoot
    # sửa lố sang hướng ngược lại (từng thấy: chếch phải -> tăng KP -> hóa chếch trái).
    if time.ticks_diff(now,_straight_last_reset)>=STRAIGHT_RESET_MS:
        _baseL=ticksL; _baseR=ticksR; _straight_last_reset=now
    kick=_kick_check(True)
    dl=ticksL-_baseL; dr=ticksR-_baseR; err=dl-dr; corr=KP_STRAIGHT*err
    if corr>STRAIGHT_CORR_MAX:corr=STRAIGHT_CORR_MAX
    if corr<-STRAIGHT_CORR_MAX:corr=-STRAIGHT_CORR_MAX
    ls=speed-corr; rs=speed+corr
    if motorBalance>0: ls-=motorBalance
    elif motorBalance<0: rs+=motorBalance
    if kick:
        if ls<KICK_LOGIC_SPEED: ls=KICK_LOGIC_SPEED
        if rs<KICK_LOGIC_SPEED: rs=KICK_LOGIC_SPEED
    IN1.value(1);IN2.value(0);IN3.value(1);IN4.value(0); _set_pwm_pair(ls,rs,"fwd")
def driveBackward(s):
    global motionDir,_bodyStationary; motionDir=-1; _bodyStationary=False
    if _kick_check(True) and s<KICK_LOGIC_SPEED: s=KICK_LOGIC_SPEED
    ls=rs=s
    if motorBalance>0:ls=s-motorBalance
    elif motorBalance<0:rs=s+motorBalance
    IN1.value(0);IN2.value(1);IN3.value(0);IN4.value(1); _set_pwm_pair(ls,rs,"back")
def driveLeft(s):
    global motionDir,_bodyStationary; motionDir=0; _bodyStationary=False   # PIVOT xoay -> thân KHÔNG đứng yên, dù motionDir=0
    if _kick_check(True) and s<KICK_LOGIC_SPEED: s=KICK_LOGIC_SPEED
    IN1.value(0);IN2.value(1);IN3.value(1);IN4.value(0); _set_pwm_pair(s,s,"left")
def driveRight(s):
    global motionDir,_bodyStationary; motionDir=0; _bodyStationary=False
    if _kick_check(True) and s<KICK_LOGIC_SPEED: s=KICK_LOGIC_SPEED
    IN1.value(1);IN2.value(0);IN3.value(0);IN4.value(1); _set_pwm_pair(s,s,"right")
def setServoAngle(a):
    if a<0:a=0
    if a>180:a=180
    us=500+(a/180.0)*1900.0; servo.duty_u16(int(us/20000.0*65535))
setServoAngle(servoAngle)

def readDistanceOnce():
    trig.value(0); time.sleep_us(2); trig.value(1); time.sleep_us(10); trig.value(0)
    d=time_pulse_us(echo,1,8000)
    return -1.0 if d<0 else (d*0.0343)/2.0
def readGyroZRaw():
    try:
        d=i2c.readfrom_mem(MPU_ADDR,0x47,2); r=(d[0]<<8)|d[1]
        return r-65536 if r>32767 else r
    except Exception: return 0

GYRO_OFFSET=0.0
def calibrateGyro(samples=200):
    # Giữ robot đứng yên khi cấp nguồn để đo độ lệch (bias) của gyro Z.
    # Nếu không trừ offset này, mỗi lần tích phân góc sẽ trôi dần -> map bị cong/méo
    # dù robot đi đúng đường (đây là nguyên nhân chính khiến map hình chữ L bị bo tròn).
    global GYRO_OFFSET
    print("Đang hiệu chỉnh gyro, giữ robot đứng yên...")
    total=0
    for i in range(samples):
        total+=readGyroZRaw()
        time.sleep_ms(3)
    GYRO_OFFSET=total/samples
    print("Gyro offset =",GYRO_OFFSET)
def canMoveForward():
    return (frontGuard>SAFE_DISTANCE) and (not cliffAhead)

def isServoFront():
    return FRONT_NARROW_LO<=servoAngle<=FRONT_NARROW_HI

def isDistBufFront():
    return _dist_buf_angle is not None and FRONT_NARROW_LO<=_dist_buf_angle<=FRONT_NARROW_HI

def frontMeasureFresh(max_age_ms=220):
    return isDistBufFront() and time.ticks_diff(time.ticks_ms(),_front_measure_ms)<max_age_ms

def clearFrontGuard():
    # Xóa mẫu sonar thẳng cũ. Khi robot vừa lùi/rẽ/quét xong, mẫu <=30cm cũ
    # có thể còn nằm trong buffer 600ms làm AUTO tưởng trước mặt vẫn bị chắn,
    # nên nó cứ lùi -> quét -> lùi. Clear để AUTO phải đo lại ở góc 90° rồi mới quyết định.
    global _front_samples, frontTrust, frontGuard, frontDistance
    global _dist_buf, _dist_buf_angle, _front_measure_ms
    _front_samples=[]
    frontTrust=999.0
    frontGuard=999.0
    frontDistance=999.0
    _dist_buf=[]
    _dist_buf_angle=None
    _front_measure_ms=0

def frontBlocked30():
    # AUTO chỉ coi là vật chắn khi sonar đang nhìn ĐÚNG THẲNG trước và đo <=30cm.
    # Không dùng số đo xa hơn 30cm để quyết định lùi; xa hơn 30cm = còn thoáng, cứ tiến.
    return frontMeasureFresh() and len(_front_samples)>=2 and (0<frontTrust<=AUTO_STOP_DIST)

def normDist(d):
    # Sonar timeout/999 nghĩa là không thấy vật trong tầm gần -> coi là rất thoáng.
    if d<=0 or d>300: return 300.0
    return d

def sideClear(d):
    return normDist(d)>=AUTO_CLEAR_DIST

def updateIrSensors():
    global cliffAhead,rearObstacle,frontIrBlocked,_ir_front_cnt,_ir_rear_cnt
    if ir_cliff.value()==1:
        _ir_front_cnt+=1
    else:
        _ir_front_cnt=0
    if ir_rear.value()==0:
        _ir_rear_cnt+=1
    else:
        _ir_rear_cnt=0
    frontIrBlocked=_ir_front_cnt>=IR_DEBOUNCE_COUNT
    cliffAhead=frontIrBlocked
    rearObstacle=_ir_rear_cnt>=IR_DEBOUNCE_COUNT

def handle_command(text):
    global autoMode,autoState,autoTimer,servoAngle,motorBalance,manualSpeed,_next_auto_side,_drive_target_ms
    global forwardCmd,backCmd,leftCmd,rightCmd
    try: d=json.loads(text)
    except Exception: return
    c=d.get("cmd"); st=(d.get("st")==1)
    if c=="a":
        autoMode=st; forwardCmd=backCmd=leftCmd=rightCmd=False; autoState=0
        servoAngle=90; setServoAngle(90); clearFrontGuard()
        if autoMode:
            _next_auto_side=1   # bật AUTO lại thì vật cản đầu tiên né bên phải
            autoTimer=time.ticks_ms()
        else:
            driveStop()
    elif c=="speed":
        try: manualSpeed=max(30,min(40,int(d.get("val",manualSpeed))))
        except Exception: pass
    elif c=="trim":
        try: motorBalance=max(-120,min(120,int(d.get("val",0))))
        except Exception: pass
    elif c=="goto":
        # Gợi ý hướng khám phá từ web (đã tính từ bản đồ hiện có, biết chỗ nào chưa quét).
        # rel_deg: số độ cần quay TƯƠNG ĐỐI so với hướng hiện tại (âm=trái, dương=phải).
        # Chỉ lưu lại, robot sẽ tự quyết định lúc nào dùng nó (an toàn vẫn ưu tiên trên hết).
        global _goto_rel_deg,_goto_ts
        try:
            rel=float(d.get("rel_deg"))
            if rel>150: rel=150
            if rel<-150: rel=-150
            _goto_rel_deg=rel
            _goto_ts=time.ticks_ms()
            _drive_target_ms=GOTO_DRIVE_MS+(os.urandom(1)[0]%250)
        except Exception: pass
    elif c=="f":
        autoMode=False  # bấm lái tay là thoát AUTO thật sự, không chỉ tắt giao diện
        forwardCmd=st
        if forwardCmd: backCmd=leftCmd=rightCmd=False
    elif c=="b":
        autoMode=False
        backCmd=st
        if backCmd: forwardCmd=leftCmd=rightCmd=False
    elif c=="l":
        autoMode=False
        leftCmd=st
        if leftCmd: forwardCmd=backCmd=rightCmd=False
    elif c=="r":
        autoMode=False
        rightCmd=st
        if rightCmd: forwardCmd=backCmd=leftCmd=False
    elif c=="s":
        forwardCmd=backCmd=leftCmd=rightCmd=False; driveStop()

def runAuto():
    global autoState,autoTimer,servoAngle,servoDir,distLeft,distRight,turnDir,backupTime,_straight_active
    global _last_turn_dir,_same_turn_count,_drive_target_ms,_next_auto_side
    global _goto_rel_deg,_goto_ts,_align_accum_deg
    now=time.ticks_ms()
    if autoState==0:    # DRIVE: đi thẳng đoạn ngắn, servo nhìn thẳng canh trước
        if servoAngle!=90:
            servoAngle=90; setServoAngle(90)
        if USE_FRONT_IR_STOP and frontIrBlocked:
            driveStop(); _straight_active=False
            clearFrontGuard(); autoTimer=now; autoState=6
        elif (USE_CLIFF_GUARD and cliffAhead) or frontBlocked30():
            driveBrake(BRAKE_MS); _straight_active=False
            servoAngle=SERVO_MAX; setServoAngle(servoAngle); autoTimer=now; autoState=1
        elif time.ticks_diff(now,autoTimer)>=_drive_target_ms:   # đi đủ 1 đoạn -> dừng quét bồi map
            driveStop(); _straight_active=False; autoTimer=now; autoState=6
        else:
            # AUTO đi đều tới khi cách vật <=30cm mới phanh/lùi.
            # Không giảm tốc từ xa nữa vì sonar xa/nhiễu làm xe cứ tưởng có vật.
            spd=autoSpeed
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
            # BẢN ỔN ĐỊNH: vẫn dùng FSM AUTO cũ, nhưng ưu tiên zigzag luân phiên:
            # vật cản đầu tiên né PHẢI, vật cản kế tiếp né TRÁI, rồi lặp lại.
            # Nếu bên ưu tiên bị bí mà bên còn lại thoáng thì tự đổi bên để khỏi đâm tường.
            lOpen=sideClear(distLeft); rOpen=sideClear(distRight)
            if lOpen and rOpen:
                # Cả hai bên đều thoáng: đi theo bên rộng hơn, nếu ngang nhau thì luân phiên.
                if abs(normDist(distLeft)-normDist(distRight))<8:
                    turnDir=_next_auto_side
                else:
                    turnDir=-1 if normDist(distLeft)>normDist(distRight) else 1
                backupTime=0; autoState=5
            elif lOpen:
                turnDir=-1; backupTime=0; autoState=5
            elif rOpen:
                turnDir=1; backupTime=0; autoState=5
            else:
                # Trước, trái, phải đều bí. Nếu sau cũng bí thì đứng chờ/quét lại; nếu sau trống thì lùi ngắn rồi xoay về bên ít bí hơn.
                turnDir=-1 if normDist(distLeft)>=normDist(distRight) else 1
                if rearObstacle:
                    autoState=8
                else:
                    backupTime=BACKUP_LONG
                    autoState=4
            _last_turn_dir=turnDir
            _next_auto_side=-_next_auto_side
            autoTimer=now
    elif autoState==4:
        if rearObstacle: driveStop(); autoState=5; autoTimer=now
        else:
            driveBackward(BACK_SPEED)
            if time.ticks_diff(now,autoTimer)>=backupTime: autoState=5; autoTimer=now
    elif autoState==5:
        # rẽ CHO TỚI KHI phía trước thật sự thoáng (servo ~90 nên frontGuard = thẳng trước),
        # rẽ tối thiểu TURN_MIN để không thoát sớm do nhiễu, tối đa TURN_MAX để không quay mãi.
        driveLeft(autoSpeed) if turnDir<0 else driveRight(autoSpeed)
        el=time.ticks_diff(now,autoTimer)
        if (frontGuard>AUTO_CLEAR_DIST and el>=TURN_MIN) or el>=TURN_MAX:
            servoAngle=90; setServoAngle(90); servoDir=1
            _drive_target_ms=DRIVE_MS+(os.urandom(1)[0]%700)   # random hóa đoạn đi kế tiếp (~1.3-2.0s)
            clearFrontGuard(); autoState=0; autoTimer=now; _straight_active=False
    elif autoState==8:    # TRAPPED: trước/trái/phải/sau đều bí hoặc IR báo nguy hiểm -> đứng yên, quét lại
        driveStop(); _straight_active=False
        if time.ticks_diff(now,autoTimer)>=900:
            clearFrontGuard(); servoAngle=90; setServoAngle(90); servoDir=1
            autoState=6; autoTimer=now
    elif autoState==6:    # SCAN_MAP: đứng yên quét rộng (servo do main loop quét) để bồi map
        driveStop(); _straight_active=False
        if time.ticks_diff(now,autoTimer)>=SCAN_MS:
            servoAngle=90; setServoAngle(90); servoDir=1
            # Có gợi ý hướng khám phá từ web còn hiệu lực (chưa quá hạn) và đáng để quay (>15°)
            # -> xoay canh theo hướng đó trước khi đi thẳng, thay vì cứ đi thẳng theo hướng cũ.
            has_goto = _goto_rel_deg is not None and time.ticks_diff(now,_goto_ts) < _GOTO_VALID_MS
            if has_goto and abs(_goto_rel_deg) > 15:
                _drive_target_ms=GOTO_DRIVE_MS+(os.urandom(1)[0]%250)   # đi ngắn, quét lại thường xuyên để bám waypoint
                _align_accum_deg=0.0; turnDir=-1 if _goto_rel_deg<0 else 1
                autoState=7; autoTimer=now
            else:
                _drive_target_ms=DRIVE_MS+(os.urandom(1)[0]%700)   # không có waypoint thì đi khám phá tự do dài hơn
                clearFrontGuard(); autoState=0; autoTimer=now
    elif autoState==7:    # ALIGN: xoay canh theo hướng khám phá gợi ý từ web trước khi đi thẳng
        if USE_FRONT_IR_STOP and frontIrBlocked:
            driveStop(); _straight_active=False
            clearFrontGuard(); autoState=8; autoTimer=now
        elif (USE_CLIFF_GUARD and cliffAhead) or frontBlocked30():
            # An toàn trên hết: giữa chừng canh hướng mà phát hiện nguy hiểm -> bỏ canh hướng,
            # xử lý như gặp vật cản bình thường (ngó trái/phải rồi tự quyết định).
            driveBrake(BRAKE_MS); _straight_active=False
            servoAngle=SERVO_MAX; setServoAngle(servoAngle); autoTimer=now; autoState=1
        else:
            driveLeft(autoSpeed) if turnDir<0 else driveRight(autoSpeed)
            el=time.ticks_diff(now,autoTimer)
            done = abs(_align_accum_deg) >= abs(_goto_rel_deg) if _goto_rel_deg is not None else True
            if done or el>=ALIGN_TURN_MAX:
                driveStop(); _goto_rel_deg=None   # dùng xong, xóa để không lặp lại gợi ý cũ
                servoAngle=90; setServoAngle(90); servoDir=1
                clearFrontGuard(); autoState=0; autoTimer=now; _straight_active=False

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
    global frontIrBlocked,_ir_front_cnt,_ir_rear_cnt
    global servoAngle,servoDir,lastTicksL,lastTicksR,_straight_active,_dist_buf,_front_samples
    global _dist_buf_angle,_front_measure_ms,_heading_accum_deg,_last_gyro_ms,GYRO_OFFSET
    global _last_ticksL_loop,_last_ticksR_loop,_signed_dist_accum
    global _align_accum_deg,_goto_rel_deg,_goto_ts
    calibrateGyro()
    driveStop(); wifi_connect()
    ws=WSClient(SERVER_HOST,SERVER_PORT,SERVER_PATH)
    last_sonar=last_servo=last_send=time.ticks_ms()
    while True:
        if not ws.connected:
            try: print("Kết nối server..."); ws.connect(); print("OK")
            except Exception as e:
                print("Lỗi:",e); driveStop(); time.sleep_ms(2000); continue
        now=time.ticks_ms()
        # Tích phân góc quay LIÊN TỤC mỗi vòng lặp bằng dt THỰC ĐO ĐƯỢC (time.ticks_diff),
        # thay vì để phía web giả định mỗi gói tin cách nhau đúng 150ms cố định. Mạng có thể
        # trễ/dồn gói bất kỳ lúc nào -> nếu giả định sai dt, góc tích phân sai theo, làm map méo.
        dt_gyro_ms=time.ticks_diff(now,_last_gyro_ms)
        _last_gyro_ms=now
        if 0<dt_gyro_ms<200:   # bỏ qua nếu dt bất thường (vừa mất kết nối, lệnh block lâu, v.v.)
            raw_gz=readGyroZRaw()
            rate_dps=(raw_gz-GYRO_OFFSET)/131.0
            if motionDir!=0 and abs(rate_dps)<GYRO_STRAIGHT_DEADBAND_DPS:
                rate_dps=0.0
            if _bodyStationary and abs(rate_dps)<GYRO_STILL_DEADBAND_DPS:
                rate_dps=0.0
            _heading_accum_deg+=rate_dps*(dt_gyro_ms/1000.0)
            _align_accum_deg+=rate_dps*(dt_gyro_ms/1000.0)   # tích lũy riêng cho state ALIGN, reset lúc bắt đầu xoay canh hướng
            if _bodyStationary:
                # Robot đang THỰC SỰ đứng yên (không phải pivot-turn) -> nhân tiện tinh chỉnh lại
                # offset gyro rất nhẹ (EMA chậm). Bias của MPU6050 trôi theo nhiệt độ/thời gian
                # chạy; nếu không bù dần, phiên chạy càng dài (như test đi 1 vòng quay lại điểm
                # đầu) càng lệch. Hệ số 0.002 rất nhỏ nên không "đuổi theo" nhiễu tức thời.
                GYRO_OFFSET=GYRO_OFFSET*0.998+raw_gz*0.002
        # Tích lũy quãng đường CÓ DẤU mỗi vòng lặp, y hệt tinh thần của _heading_accum_deg ở trên:
        # cộng dồn theo motionDir TẠI ĐÚNG THỜI ĐIỂM tick phát sinh, không phải motionDir tại thời
        # điểm gửi gói. Trước đây dL được tính 1 lần lúc gửi (tổng tick cả 150ms) rồi NHÂN với "mv"
        # chụp nhanh -> nếu robot vừa dừng lại đúng lúc chuẩn bị gửi gói, cả đoạn tick vừa đi THẬT
        # trong 150ms đó bị nhân với mv=0 và mất trắng, khiến vị trí bị tính thiếu.
        dtkL=ticksL-_last_ticksL_loop; _last_ticksL_loop=ticksL
        dtkR=ticksR-_last_ticksR_loop; _last_ticksR_loop=ticksR
        _signed_dist_accum += ((dtkL+dtkR)/2)*motionDir
        # SERVO theo kiểu một-sonar: đang đi -> NHÌN THẲNG canh trước (không mù);
        # đứng yên -> quét rộng để dựng map. Auto đang ngó trái/phải -> FSM tự giữ servo.
        if time.ticks_diff(now,last_servo)>=55:
            fsm=autoMode and autoState!=6   # AUTO state 0 cũng giữ servo 90, không quét nhầm làm false obstacle
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
        # sonar (một con trên servo). AUTO chỉ lùi khi số đo THẲNG TRƯỚC <=30cm.
        # Không giữ số đo xa/chéo trong frontGuard nữa, vì như vậy xe dễ tưởng còn vật cũ rồi lùi liên tục.
        if time.ticks_diff(now,last_sonar)>=35:
            # Servo vừa đổi góc từ lần đo trước -> xóa buffer cũ. Nếu không, median filter sẽ
            # TRỘN các khoảng cách đo được ở NHIỀU góc khác nhau (do buffer giữ 5 mẫu gần nhất theo
            # thời gian, không theo góc) -> khi ghép vào map, khoảng cách bị gán sai lệch so với góc
            # thật -> chính là nguyên nhân làm góc tường bị bo tròn/nhòe thay vì sắc nét.
            if servoAngle!=_dist_buf_angle:
                _dist_buf=[]; _dist_buf_angle=servoAngle
            d=readDistanceOnce()
            if d>0:
                _dist_buf.append(d)
                if len(_dist_buf)>5: _dist_buf.pop(0)
            elif isServoFront():
                _dist_buf=[]
            frontDistance=sorted(_dist_buf)[len(_dist_buf)//2] if _dist_buf else 999.0
            if isDistBufFront() and d>0:
                _front_measure_ms=now
            # Chỉ ghi nhận vật chắn khi sonar đang nhìn gần đúng 90° VÀ khoảng cách <=30cm.
            # Nếu đo 31cm, 60cm, 100cm... thì coi là chưa tới vật chắn -> không lùi.
            if isDistBufFront():
                if frontMeasureFresh() and 0<frontDistance<=AUTO_STOP_DIST:
                    _front_samples.append((now,frontDistance))
                else:
                    _front_samples=[]
            _front_samples=[(t,v) for (t,v) in _front_samples if time.ticks_diff(now,t)<320]
            frontTrust=min(v for (t,v) in _front_samples) if _front_samples else 999.0
            frontGuard=frontTrust
            updateIrSensors()
            last_sonar=now
        # điều khiển
        if autoMode:
            runAuto()
        else:
            spd=manualSpeed
            # Lái tay giữ đúng slider 30-40, không tự nhảy tốc làm xe giật.
            # Lái tay phải chạy được cả TIẾN và LÙI. Không chặn tiến bằng sonar nữa,
            # vì sonar/servo đang quét có thể đọc nhầm vật cản làm nút ▲ không chạy.
            # Khi cần dừng khẩn thì bấm nút ■.
            if forwardCmd:
                if not _straight_active: _straight_reset()
                driveForwardStraight(spd)
            elif backCmd:
                _straight_active=False; driveBackward(manualSpeed)
            elif leftCmd: _straight_active=False; driveLeft(manualSpeed)
            elif rightCmd: _straight_active=False; driveRight(manualSpeed)
            else: _straight_active=False; driveStop()
        # nhận lệnh
        msg=ws.recv()
        if msg is not None: handle_command(msg)
        # gửi telemetry
        if time.ticks_diff(now,last_send)>=150:
            dz=_heading_accum_deg; _heading_accum_deg=0.0   # góc quay dồn tích, reset sau khi gửi
            dL=_signed_dist_accum; _signed_dist_accum=0.0   # quãng đường CÓ DẤU dồn tích, reset sau khi gửi
            ang_for_dist=_dist_buf_angle if _dist_buf_angle is not None else servoAngle
            dnL=ticksL-lastTicksL; lastTicksL=ticksL
            dnR=ticksR-lastTicksR; lastTicksR=ticksR
            if autoMode: safe=0 if ((USE_CLIFF_GUARD and cliffAhead) or frontBlocked30()) else 1
            else: safe=1 if canMoveForward() else 0
            pkt={"ticks":ticksL+ticksR,"dL":round(dL,2),"dnL":dnL,"dnR":dnR,"dz":round(dz,3),
                 "dist":round(frontDistance,1),"ang":ang_for_dist,"mv":motionDir,
                 "ast":autoState,"spd":manualSpeed,"aspd":autoSpeed,"bs":BACK_SPEED,"safe":safe,
                 "irf":1 if frontIrBlocked else 0,"rear":1 if rearObstacle else 0,
                 "auto":1 if autoMode else 0}
            try: ws.send(json.dumps(pkt))
            except Exception: ws.connected=False
            last_send=now
        time.sleep_ms(2)

if __name__=="__main__":
    try: main()
    except KeyboardInterrupt: driveStop(); print("Dừng.")
