"""
SLAM ROBOT - SERVER TRUNG TÂM (FastAPI)
=======================================
Tách web khỏi robot để có thể:
  - Public ra internet (qua tunnel hoặc cloud) -> điện thoại dùng từ bất cứ đâu
  - Dùng được KHI KHÔNG có robot: xem lại phiên chạy cũ, nạp/lưu map

Endpoint:
  WS  /ws/robot       <- robot ESP32 (STA client) gửi telemetry / nhận lệnh
  WS  /ws/dashboard   <- trình duyệt: nhận telemetry trực tiếp / gửi lệnh
  GET /api/sessions             danh sách phiên đã ghi
  GET /api/sessions/{id}        dữ liệu 1 phiên (để xem lại)
  DEL /api/sessions/{id}        xoá phiên
  GET /api/maps                 danh sách map đã lưu
  POST /api/maps                lưu 1 map (JSON tuỳ frontend)
  GET /api/maps/{id}            nạp 1 map
  DEL /api/maps/{id}            xoá map
  /                             phục vụ frontend

Chạy:  uvicorn server:app --host 0.0.0.0 --port 8000
"""
from __future__ import annotations
import json, os, time, uuid, asyncio
from typing import Optional, Set, Dict, List
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

BASE = os.path.dirname(__file__)
DATA = os.path.join(BASE, "data")
SES_DIR = os.path.join(DATA, "sessions")
MAP_DIR = os.path.join(DATA, "maps")
os.makedirs(SES_DIR, exist_ok=True)
os.makedirs(MAP_DIR, exist_ok=True)

app = FastAPI(title="SLAM Robot Server")


class Hub:
    def __init__(self):
        self.robot: Optional[WebSocket] = None
        self.dashboards: Set[WebSocket] = set()
        # ghi phiên
        self.recording: List[dict] = []
        self.session_start: float = 0.0
        self.last_frame_t: float = 0.0

    def robot_online(self) -> bool:
        return self.robot is not None

HUB = Hub()
MAX_FRAMES = 20000          # giới hạn 1 phiên (~ vài chục phút ở 7 khung/s)
MIN_FRAMES_TO_SAVE = 15     # phiên quá ngắn thì bỏ


def _save_session():
    """Lưu phiên đang ghi xuống file rồi xoá bộ đệm."""
    if len(HUB.recording) >= MIN_FRAMES_TO_SAVE:
        sid = time.strftime("%Y%m%d-%H%M%S", time.localtime(HUB.session_start))
        meta = {
            "id": sid,
            "start": HUB.session_start,
            "end": time.time(),
            "duration_s": round(time.time() - HUB.session_start, 1),
            "frames": HUB.recording,
        }
        try:
            with open(os.path.join(SES_DIR, sid + ".json"), "w") as f:
                json.dump(meta, f)
            print("[session] đã lưu", sid, "(", len(HUB.recording), "khung )")
        except Exception as e:
            print("[session] lỗi lưu:", e)
    HUB.recording = []
    HUB.session_start = 0.0


async def broadcast(payload: dict):
    if not HUB.dashboards:
        return
    text = json.dumps(payload)
    dead = []
    for ws in HUB.dashboards:
        try:
            await ws.send_text(text)
        except Exception:
            dead.append(ws)
    for ws in dead:
        HUB.dashboards.discard(ws)


# ---------------- WebSocket: ROBOT ----------------
@app.websocket("/ws/robot")
async def ws_robot(ws: WebSocket):
    await ws.accept()
    HUB.robot = ws
    HUB.session_start = time.time()
    HUB.recording = []
    print("[robot] online — bắt đầu ghi phiên")
    await broadcast({"type": "robot_status", "online": True})
    try:
        while True:
            raw = await ws.receive_text()
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                continue
            # ghi khung (kèm timestamp tương đối ms)
            data["_t"] = int((time.time() - HUB.session_start) * 1000)
            if len(HUB.recording) < MAX_FRAMES:
                HUB.recording.append(data)
            # chuyển tiếp cho dashboard (gắn nhãn live)
            await broadcast({"type": "telemetry", "data": data})
    except WebSocketDisconnect:
        pass
    finally:
        HUB.robot = None
        _save_session()
        print("[robot] offline — đã lưu phiên")
        await broadcast({"type": "robot_status", "online": False})


# ---------------- WebSocket: DASHBOARD ----------------
@app.websocket("/ws/dashboard")
async def ws_dashboard(ws: WebSocket):
    await ws.accept()
    HUB.dashboards.add(ws)
    await ws.send_text(json.dumps({"type": "robot_status", "online": HUB.robot_online()}))
    try:
        while True:
            raw = await ws.receive_text()
            # lệnh điều khiển -> chuyển nguyên văn xuống robot (cùng định dạng cũ)
            if HUB.robot is not None:
                try:
                    await HUB.robot.send_text(raw)
                except Exception:
                    pass
    except WebSocketDisconnect:
        pass
    finally:
        HUB.dashboards.discard(ws)


# ---------------- REST: SESSIONS ----------------
@app.get("/api/sessions")
async def list_sessions():
    out = []
    for fn in sorted(os.listdir(SES_DIR), reverse=True):
        if not fn.endswith(".json"):
            continue
        try:
            with open(os.path.join(SES_DIR, fn)) as f:
                m = json.load(f)
            out.append({"id": m["id"], "start": m["start"],
                        "duration_s": m.get("duration_s", 0),
                        "frames": len(m.get("frames", []))})
        except Exception:
            continue
    return out


@app.get("/api/sessions/{sid}")
async def get_session(sid: str):
    path = os.path.join(SES_DIR, sid + ".json")
    if not os.path.isfile(path):
        raise HTTPException(404, "Không tìm thấy phiên")
    with open(path) as f:
        return json.load(f)


@app.delete("/api/sessions/{sid}")
async def del_session(sid: str):
    path = os.path.join(SES_DIR, sid + ".json")
    if os.path.isfile(path):
        os.remove(path)
    return {"ok": True}


# ---------------- REST: MAPS ----------------
@app.get("/api/maps")
async def list_maps():
    out = []
    for fn in sorted(os.listdir(MAP_DIR), reverse=True):
        if not fn.endswith(".json"):
            continue
        try:
            with open(os.path.join(MAP_DIR, fn)) as f:
                m = json.load(f)
            out.append({"id": m["id"], "name": m.get("name", m["id"]),
                        "created": m.get("created", 0)})
        except Exception:
            continue
    return out


@app.post("/api/maps")
async def save_map(payload: dict):
    mid = uuid.uuid4().hex[:10]
    rec = {"id": mid, "name": payload.get("name", "map"),
           "created": time.time(), "data": payload.get("data", {})}
    with open(os.path.join(MAP_DIR, mid + ".json"), "w") as f:
        json.dump(rec, f)
    return {"ok": True, "id": mid}


@app.get("/api/maps/{mid}")
async def get_map(mid: str):
    path = os.path.join(MAP_DIR, mid + ".json")
    if not os.path.isfile(path):
        raise HTTPException(404, "Không tìm thấy map")
    with open(path) as f:
        return json.load(f)


@app.delete("/api/maps/{mid}")
async def del_map(mid: str):
    path = os.path.join(MAP_DIR, mid + ".json")
    if os.path.isfile(path):
        os.remove(path)
    return {"ok": True}


@app.get("/api/status")
async def status():
    return {"robot_online": HUB.robot_online(),
            "dashboards": len(HUB.dashboards),
            "recording_frames": len(HUB.recording)}


# ---------------- Phục vụ frontend ----------------
FE = os.path.join(BASE, "..", "frontend")
if os.path.isdir(FE):
    @app.get("/")
    async def index():
        return FileResponse(os.path.join(FE, "index.html"))
    app.mount("/static", StaticFiles(directory=FE), name="static")
