"""pc_server エントリポイント(FastAPI)。

このファイルだけが Web フレームワークに依存する薄い層:
- static/ の配信(ビルド不要 vanilla JS UI)
- REST: /api/ports, /api/airframes, /api/config
- WebSocket /ws: UI コマンド受付 + 20Hz 状態配信 + 即時 event/log 配信

ブロッキングする SessionManager 呼び出しは asyncio.to_thread で executor に
逃がし、スレッド→async の橋渡しは queue.Queue を 20Hz でポーリングして行う
(コーディング規約: ブロッキング I/O を async ループに持ち込まない)。

起動: cd pc_server && python3 -m uvicorn app:app --host 127.0.0.1 --port 8000
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import queue
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

import core  # noqa: F401  (protocol/ と vendor/ の sys.path シム)
from core.session import SessionManager

PC_SERVER_DIR = Path(__file__).resolve().parent
STATIC_DIR = PC_SERVER_DIR / "static"

_log = logging.getLogger(__name__)

session = SessionManager()

# 1クライアントへの WebSocket 送信タイムアウト。受信ウィンドウが詰まった
# クライアント1つが全クライアント向け配信を止めるのを防ぐ(超過=切断扱い)
_WS_SEND_TIMEOUT_S: float = session.server_config["websocket"]["send_timeout_s"]

# 接続中の WebSocket クライアント(asyncio ループ内でのみ操作する)
_clients: set[WebSocket] = set()


def _list_serial_ports() -> list[dict]:
    """接続可能なシリアルポートの列挙(pyserial)。"""
    try:
        from serial.tools import list_ports  # 遅延 import
    except ImportError:
        return []
    return [{"device": p.device, "description": p.description}
            for p in list_ports.comports()]


async def _send_safely(websocket: WebSocket, message: dict) -> bool:
    """1クライアントへの送信。失敗/タイムアウトで False(呼び出し元が除去)。"""
    try:
        await asyncio.wait_for(websocket.send_json(message),
                               timeout=_WS_SEND_TIMEOUT_S)
        return True
    except Exception:
        # タイムアウトも「死んだクライアント」として扱い、配信ループ全体を
        # 1接続の停滞に巻き込ませない
        return False


async def _broadcast(message: dict) -> None:
    dead = [ws for ws in list(_clients)
            if not await _send_safely(ws, message)]
    for ws in dead:
        _clients.discard(ws)


async def _broadcaster_loop() -> None:
    """20Hz: 即時イベントの drain + 状態スナップショット配信。

    このタスクは session.events(無制限キュー)の唯一の消費者なので、
    1周期の例外で死なせてはならない(死ぬと UI 配信が全停止し、イベントが
    無限に溜まる)。例外はログして次周期へ進む。
    """
    period_s = 1.0 / session.server_config["rates"]["ws_state_hz"]
    while True:
        try:
            # 即時メッセージ(TLM_EVENT / LOG_TEXT / サーバ警告)を順に配信
            while True:
                try:
                    event = session.events.get_nowait()
                except queue.Empty:
                    break
                await _broadcast(event)
            if _clients:
                state = await asyncio.to_thread(session.get_state_snapshot)
                await _broadcast(state)
        except Exception:
            # CancelledError は BaseException なのでここを素通りし、
            # lifespan の task.cancel() による停止は妨げない
            _log.exception("broadcaster loop iteration failed")
        await asyncio.sleep(period_s)


@contextlib.asynccontextmanager
async def _lifespan(app: FastAPI):
    task = asyncio.create_task(_broadcaster_loop())
    try:
        yield
    finally:
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task
        await asyncio.to_thread(session.shutdown)


app = FastAPI(title="StampFly Integrated Control", lifespan=_lifespan)


# ----------------------------------------------------------------------
# REST
# ----------------------------------------------------------------------

@app.get("/api/ports")
async def api_ports() -> list[dict]:
    return await asyncio.to_thread(_list_serial_ports)


@app.get("/api/airframes")
async def api_airframes() -> dict:
    return {"airframes": session.airframes}


@app.put("/api/airframes")
async def api_airframes_update(body: dict) -> dict:
    """機体プロファイル一覧の更新(UI の編集画面から)。

    body: {"airframes":[...]}。検証・原子的保存・セッション反映は
    session.update_airframes が行う(core/ は fastapi 非依存のまま)。
    失敗時も HTTP 200 で {"ok":false,"error":...} を返し、UI が表示する。
    """
    ok, error = await asyncio.to_thread(
        session.update_airframes, body.get("airframes"))
    return {"ok": ok, "error": error, "airframes": session.airframes}


@app.get("/api/config")
async def api_config() -> dict:
    return {"server": session.server_config, "control": session.control_config}


# ----------------------------------------------------------------------
# WebSocket
# ----------------------------------------------------------------------

async def _handle_command(message: dict) -> None:
    """{"type":"command", ...} の処理(ブロッキングし得るので to_thread)。"""
    action = message.get("action")
    if action == "connect":
        await asyncio.to_thread(session.connect, str(message.get("port", "")))
    elif action == "disconnect":
        await asyncio.to_thread(session.disconnect)
    elif action == "select_airframe":
        await asyncio.to_thread(session.select_airframe, str(message.get("name", "")))
    elif action == "set_mode":
        await asyncio.to_thread(session.set_mode, str(message.get("mode", "")))
    elif action == "start":
        await asyncio.to_thread(session.start)
    elif action == "stop":
        await asyncio.to_thread(session.stop)
    elif action == "reset":
        await asyncio.to_thread(session.reset)
    elif action == "set_logging":
        await asyncio.to_thread(session.set_logging, bool(message.get("enabled")))
    else:
        session.warn(f"不明なコマンド: {action}")


async def _handle_message(message: dict) -> None:
    msg_type = message.get("type")
    try:
        if msg_type == "command":
            await _handle_command(message)
        elif msg_type == "setpoint":
            # Posture モード: deg/m → session 層で rad へ変換(非ブロッキング)
            session.set_setpoint_deg(float(message["roll_deg"]),
                                     float(message["pitch_deg"]),
                                     float(message["alt_m"]))
        elif msg_type == "target":
            # Position モード: 制御座標系 m(非ブロッキング)
            session.set_target(float(message["x"]),
                               float(message["y"]),
                               float(message["z"]))
        else:
            session.warn(f"不明なメッセージ型: {msg_type}")
    except (KeyError, TypeError, ValueError) as exc:
        session.warn(f"不正なメッセージ ({msg_type}): {exc}")


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    await websocket.accept()
    _clients.add(websocket)
    # 接続直後に現在状態を1回送る(UI 初期表示用)
    await _send_safely(websocket,
                       await asyncio.to_thread(session.get_state_snapshot))
    try:
        while True:
            message = await websocket.receive_json()
            if isinstance(message, dict):
                await _handle_message(message)
    except WebSocketDisconnect:
        pass
    finally:
        _clients.discard(websocket)


# ----------------------------------------------------------------------
# 静的 UI(API ルートより後にマウントする)
# ----------------------------------------------------------------------

STATIC_DIR.mkdir(parents=True, exist_ok=True)
app.mount("/", StaticFiles(directory=str(STATIC_DIR), html=True), name="static")
