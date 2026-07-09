from __future__ import annotations

import json
import os
import re
import secrets
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

from fastapi import Body, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse


ROOT = Path(__file__).resolve().parent
PROJECT_DIR = ROOT.parent
FRONTEND_INDEX = PROJECT_DIR / "frontend" / "index.html"

DATA_DIR = ROOT / "data"
SESSIONS_DIR = DATA_DIR / "sessions"
MAPS_DIR = DATA_DIR / "maps"

for d in (SESSIONS_DIR, MAPS_DIR):
    d.mkdir(parents=True, exist_ok=True)


app = FastAPI(title="ESP32 SLAM Robot Backend")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


dashboard_clients: set[WebSocket] = set()
robot_client: Optional[WebSocket] = None
ID_RE = re.compile(r"^[A-Za-z0-9_-]+$")


def dumps(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, separators=(",", ":"))


def read_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
    os.replace(tmp, path)


def safe_id(value: str) -> str:
    if not value or not ID_RE.match(value):
        raise HTTPException(status_code=404, detail="Not found")
    return value


def new_session_id() -> str:
    base = datetime.now().strftime("%Y%m%d-%H%M%S")
    sid = base
    i = 1
    while (SESSIONS_DIR / f"{sid}.json").exists():
        sid = f"{base}-{i}"
        i += 1
    return sid


def new_map_id() -> str:
    while True:
        mid = secrets.token_hex(5)
        if not (MAPS_DIR / f"{mid}.json").exists():
            return mid


async def ws_send_json(ws: WebSocket, data: Any) -> None:
    await ws.send_text(dumps(data))


async def broadcast(data: Any) -> None:
    msg = dumps(data)
    dead = []
    for ws in list(dashboard_clients):
        try:
            await ws.send_text(msg)
        except Exception:
            dead.append(ws)
    for ws in dead:
        dashboard_clients.discard(ws)


async def broadcast_robot_status() -> None:
    await broadcast({"type": "robot_status", "online": robot_client is not None})


def save_session(
    sid: str,
    start_epoch: float,
    start_mono: float,
    frames: list[dict[str, Any]],
) -> None:
    if not frames:
        return
    data = {
        "id": sid,
        "start": start_epoch,
        "end": time.time(),
        "duration_s": round(time.monotonic() - start_mono, 1),
        "frames": frames,
    }
    write_json(SESSIONS_DIR / f"{sid}.json", data)


@app.get("/")
async def index():
    if not FRONTEND_INDEX.exists():
        raise HTTPException(status_code=404, detail="frontend/index.html not found")
    return FileResponse(FRONTEND_INDEX)


@app.get("/index.html")
async def index_html():
    return await index()


@app.websocket("/ws/dashboard")
async def dashboard_ws(ws: WebSocket):
    global robot_client

    await ws.accept()
    dashboard_clients.add(ws)
    await ws_send_json(ws, {"type": "robot_status", "online": robot_client is not None})

    try:
        while True:
            command_text = await ws.receive_text()
            target = robot_client

            if target is None:
                await ws_send_json(ws, {"type": "robot_status", "online": False})
                continue

            try:
                await target.send_text(command_text)
            except Exception:
                if robot_client is target:
                    robot_client = None
                await broadcast_robot_status()

    except WebSocketDisconnect:
        pass
    finally:
        dashboard_clients.discard(ws)


@app.websocket("/ws/robot")
async def robot_ws(ws: WebSocket):
    global robot_client

    await ws.accept()

    old_robot = robot_client
    if old_robot is not None and old_robot is not ws:
        try:
            await old_robot.close(code=1012)
        except Exception:
            pass

    robot_client = ws
    await broadcast_robot_status()

    sid = new_session_id()
    start_epoch = time.time()
    start_mono = time.monotonic()
    frames: list[dict[str, Any]] = []

    try:
        while True:
            text = await ws.receive_text()

            try:
                frame = json.loads(text)
            except json.JSONDecodeError:
                continue

            if not isinstance(frame, dict):
                continue

            frame["_t"] = int((time.monotonic() - start_mono) * 1000)
            frames.append(frame)

            await broadcast({"type": "telemetry", "data": frame})

            if len(frames) % 25 == 0:
                save_session(sid, start_epoch, start_mono, frames)

    except WebSocketDisconnect:
        pass
    finally:
        if robot_client is ws:
            robot_client = None
        save_session(sid, start_epoch, start_mono, frames)
        await broadcast_robot_status()


@app.get("/api/sessions")
async def list_sessions():
    rows = []

    for path in sorted(SESSIONS_DIR.glob("*.json"), reverse=True):
        data = read_json(path)
        if not isinstance(data, dict):
            continue

        frames = data.get("frames") or []
        rows.append(
            {
                "id": data.get("id") or path.stem,
                "start": data.get("start"),
                "end": data.get("end"),
                "duration_s": data.get("duration_s", 0),
                "frames": len(frames) if isinstance(frames, list) else 0,
            }
        )

    return JSONResponse(rows)


@app.get("/api/sessions/{session_id}")
async def get_session(session_id: str):
    sid = safe_id(session_id)
    path = SESSIONS_DIR / f"{sid}.json"

    data = read_json(path)
    if not isinstance(data, dict):
        raise HTTPException(status_code=404, detail="Session not found")

    return JSONResponse(data)


@app.delete("/api/sessions/{session_id}")
async def delete_session(session_id: str):
    sid = safe_id(session_id)
    path = SESSIONS_DIR / f"{sid}.json"

    if path.exists():
        path.unlink()

    return {"ok": True}


@app.get("/api/maps")
async def list_maps():
    rows = []

    for path in MAPS_DIR.glob("*.json"):
        data = read_json(path)
        if not isinstance(data, dict):
            continue

        rows.append(
            {
                "id": data.get("id") or path.stem,
                "name": data.get("name") or path.stem,
                "created": data.get("created", 0),
            }
        )

    rows.sort(key=lambda row: row.get("created") or 0, reverse=True)
    return JSONResponse(rows)


@app.post("/api/maps")
async def create_map(payload: dict[str, Any] = Body(...)):
    name = str(payload.get("name") or "Map").strip() or "Map"
    data = payload.get("data") or {}

    if not isinstance(data, dict):
        raise HTTPException(status_code=400, detail="data must be an object")

    mid = new_map_id()
    item = {
        "id": mid,
        "name": name,
        "created": time.time(),
        "data": data,
    }

    write_json(MAPS_DIR / f"{mid}.json", item)
    return JSONResponse(item)


@app.get("/api/maps/{map_id}")
async def get_map(map_id: str):
    mid = safe_id(map_id)
    path = MAPS_DIR / f"{mid}.json"

    data = read_json(path)
    if not isinstance(data, dict):
        raise HTTPException(status_code=404, detail="Map not found")

    return JSONResponse(data)


@app.delete("/api/maps/{map_id}")
async def delete_map(map_id: str):
    mid = safe_id(map_id)
    path = MAPS_DIR / f"{mid}.json"

    if path.exists():
        path.unlink()

    return {"ok": True}


@app.get("/health")
async def health():
    return {
        "ok": True,
        "robot_online": robot_client is not None,
        "dashboards": len(dashboard_clients),
    }