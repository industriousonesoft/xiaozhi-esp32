from __future__ import annotations

import argparse
import asyncio
import json
import random
import socket
import time
from collections import defaultdict
from pathlib import Path

from aiohttp import WSMsgType, web

from .protocol import Packet, StreamType, parse_packet
from .recorder import SessionRecorder


class DashboardProtocol(asyncio.DatagramProtocol):
    def __init__(self, state: "DashboardState"):
        self.state = state

    def connection_made(self, transport):
        self.state.transport = transport

    def datagram_received(self, data: bytes, address):
        if data.startswith(b"{"):
            self.state.handle_control_response(data, address)
            return
        try:
            packet = parse_packet(data)
        except ValueError:
            self.state.invalid_packets += 1
            return
        self.state.handle_packet(packet, data)


class DashboardState:
    def __init__(self, recordings: Path, control_port: int):
        self.recordings = recordings
        self.control_port = control_port
        self.transport = None
        self.clients: set[web.WebSocketResponse] = set()
        self.controller: web.WebSocketResponse | None = None
        self.device_ip = ""
        self.session_id = 0
        self.connected = False
        self.invalid_packets = 0
        self.received = defaultdict(int)
        self.network_missing = defaultdict(int)
        self.last_sequence: dict[StreamType, int] = {}
        self.recorder: SessionRecorder | None = None
        self.last_device_status: dict = {}
        self.heartbeat_task: asyncio.Task | None = None

    def send_control(self, message: dict) -> None:
        if not self.device_ip or self.transport is None:
            return
        self.transport.sendto(
            json.dumps(message, separators=(",", ":")).encode(),
            (self.device_ip, self.control_port),
        )

    async def connect(self, device_ip: str, data_port: int) -> None:
        socket.inet_aton(device_ip)
        if self.connected:
            self.disconnect()
        self.device_ip = device_ip
        self.session_id = random.SystemRandom().randrange(1, 2**32)
        self.received.clear()
        self.network_missing.clear()
        self.last_sequence.clear()
        self.send_control(
            {
                "type": "start",
                "version": 1,
                "session_id": self.session_id,
                "data_port": data_port,
            }
        )
        if self.heartbeat_task is not None:
            self.heartbeat_task.cancel()
        self.heartbeat_task = asyncio.create_task(self._heartbeat_loop())

    def disconnect(self) -> None:
        if self.device_ip and self.session_id:
            self.send_control({"type": "stop", "session_id": self.session_id})
        self.connected = False
        if self.recorder is not None:
            self.recorder.close()
            self.recorder = None
        if self.heartbeat_task is not None:
            self.heartbeat_task.cancel()
            self.heartbeat_task = None

    async def _heartbeat_loop(self) -> None:
        while True:
            await asyncio.sleep(1)
            self.send_control({"type": "heartbeat", "session_id": self.session_id})
            await self.broadcast_status()

    def handle_control_response(self, data: bytes, _address) -> None:
        try:
            response = json.loads(data)
        except json.JSONDecodeError:
            self.invalid_packets += 1
            return
        self.last_device_status = response
        if (
            response.get("type") == "start"
            and response.get("status") == "ok"
            and response.get("session_id") == self.session_id
        ):
            self.connected = True
        asyncio.create_task(self.broadcast_json({"kind": "device", **response}))

    def handle_packet(self, packet: Packet, raw: bytes) -> None:
        if packet.session_id != self.session_id:
            return
        self.received[packet.stream] += 1
        previous = self.last_sequence.get(packet.stream)
        if previous is not None and packet.sequence > previous + 1:
            self.network_missing[packet.stream] += packet.sequence - previous - 1
        if previous is None or packet.sequence > previous:
            self.last_sequence[packet.stream] = packet.sequence
        if self.recorder is not None:
            self.recorder.write(packet)
        for client in tuple(self.clients):
            if not client.closed:
                asyncio.create_task(self._send_binary(client, raw))

    async def _send_binary(self, client: web.WebSocketResponse, data: bytes) -> None:
        try:
            await asyncio.wait_for(client.send_bytes(data), timeout=0.1)
        except (asyncio.TimeoutError, ConnectionError, RuntimeError):
            pass

    async def broadcast_json(self, payload: dict) -> None:
        for client in tuple(self.clients):
            if not client.closed:
                try:
                    await client.send_json(payload)
                except (ConnectionError, RuntimeError):
                    pass

    async def broadcast_status(self) -> None:
        streams = {}
        for stream in StreamType:
            name = stream.name.lower()
            streams[name] = {
                "received": self.received[stream],
                "missing": self.network_missing[stream],
            }
        await self.broadcast_json(
            {
                "kind": "status",
                "connected": self.connected,
                "device_ip": self.device_ip,
                "session_id": self.session_id,
                "recording": self.recorder is not None,
                "invalid_packets": self.invalid_packets,
                "streams": streams,
                "device": self.last_device_status,
            }
        )

    def start_recording(self) -> Path:
        if not self.connected:
            raise RuntimeError("device is not connected")
        if self.recorder is None:
            self.recorder = SessionRecorder(
                self.recordings, self.session_id, device_ip=self.device_ip
            )
        return self.recorder.directory

    def stop_recording(self) -> Path | None:
        if self.recorder is None:
            return None
        path = self.recorder.close()
        self.recorder = None
        return path


async def websocket_handler(request: web.Request) -> web.WebSocketResponse:
    state: DashboardState = request.app["state"]
    ws = web.WebSocketResponse(heartbeat=20, max_msg_size=1 << 20)
    await ws.prepare(request)
    state.clients.add(ws)
    if state.controller is None:
        state.controller = ws
    await ws.send_json({"kind": "role", "controller": state.controller is ws})
    await state.broadcast_status()

    try:
        async for message in ws:
            if message.type is not WSMsgType.TEXT:
                continue
            try:
                command = json.loads(message.data)
            except json.JSONDecodeError:
                await ws.send_json({"kind": "error", "error": "invalid_json"})
                continue
            if state.controller is not ws:
                await ws.send_json({"kind": "error", "error": "read_only_client"})
                continue
            action = command.get("action")
            try:
                if action == "connect":
                    await state.connect(command["device_ip"], request.app["data_port"])
                elif action == "disconnect":
                    state.disconnect()
                elif action == "record_start":
                    path = state.start_recording()
                    await ws.send_json({"kind": "recording", "path": str(path)})
                elif action == "record_stop":
                    path = state.stop_recording()
                    await ws.send_json(
                        {"kind": "recording", "path": str(path) if path else ""}
                    )
                elif action == "set_params":
                    state.send_control(
                        {
                            "type": "set_params",
                            "session_id": state.session_id,
                            "params": command["params"],
                        }
                    )
                elif action == "reset_params":
                    state.send_control(
                        {"type": "reset_params", "session_id": state.session_id}
                    )
                elif action == "status":
                    state.send_control(
                        {"type": "status", "session_id": state.session_id}
                    )
                    await state.broadcast_status()
                else:
                    raise ValueError("unknown_action")
            except (KeyError, OSError, RuntimeError, ValueError) as exc:
                await ws.send_json({"kind": "error", "error": str(exc)})
    finally:
        state.clients.discard(ws)
        if state.controller is ws:
            state.controller = next(iter(state.clients), None)
            if state.controller is not None:
                await state.controller.send_json({"kind": "role", "controller": True})
    return ws


async def create_app(host: str, http_port: int, data_port: int, control_port: int):
    static_dir = Path(__file__).with_name("static")
    state = DashboardState(Path.cwd() / "recordings", control_port)
    app = web.Application()
    app["state"] = state
    app["data_port"] = data_port
    app.router.add_get("/ws", websocket_handler)
    app.router.add_get("/", lambda _: web.FileResponse(static_dir / "index.html"))
    app.router.add_static("/static", static_dir)

    loop = asyncio.get_running_loop()
    await loop.create_datagram_endpoint(
        lambda: DashboardProtocol(state), local_addr=("0.0.0.0", data_port)
    )
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, host, http_port)
    await site.start()
    print(f"Audio debug dashboard: http://{host}:{http_port}")
    return app, runner


async def async_main(args) -> None:
    _app, runner = await create_app(
        args.host, args.http_port, args.data_port, args.control_port
    )
    try:
        await asyncio.Event().wait()
    finally:
        await runner.cleanup()


def main() -> None:
    parser = argparse.ArgumentParser(description="ESP32 audio debug dashboard")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8000)
    parser.add_argument("--data-port", type=int, default=8002)
    parser.add_argument("--control-port", type=int, default=8001)
    args = parser.parse_args()
    asyncio.run(async_main(args))


if __name__ == "__main__":
    main()
