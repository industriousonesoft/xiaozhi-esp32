import asyncio
import json

from audio_debug_dashboard.protocol import StreamType, build_packet
from audio_debug_dashboard.server import DashboardState


class FakeTransport:
    def __init__(self):
        self.sent = []

    def sendto(self, data, address):
        self.sent.append((json.loads(data), address))


class FakeWebSocket:
    closed = False

    def __init__(self):
        self.binary = []
        self.json = []

    async def send_bytes(self, data):
        self.binary.append(data)

    async def send_json(self, data):
        self.json.append(data)


def test_connect_forward_and_disconnect(tmp_path):
    async def scenario():
        state = DashboardState(tmp_path, control_port=8001)
        state.transport = FakeTransport()
        client = FakeWebSocket()
        state.clients.add(client)

        await state.connect("192.168.1.20", data_port=8002)
        start, address = state.transport.sent[-1]
        assert address == ("192.168.1.20", 8001)
        assert start["type"] == "start"
        assert start["data_port"] == 8002
        assert start["session_id"] == state.session_id

        state.handle_control_response(
            json.dumps(
                {
                    "type": "start",
                    "status": "ok",
                    "session_id": state.session_id,
                    "active": True,
                }
            ).encode(),
            ("192.168.1.20", 8001),
        )
        assert state.connected

        first = build_packet(
            StreamType.AFE, state.session_id, 3, 0, 16_000, 2, 1, b"\0\0\0\0"
        )
        second = build_packet(
            StreamType.AFE, state.session_id, 5, 125, 16_000, 2, 1, b"\0\0\0\0"
        )
        from audio_debug_dashboard.protocol import parse_packet

        state.handle_packet(parse_packet(first), first)
        state.handle_packet(parse_packet(second), second)
        await asyncio.sleep(0.01)
        assert client.binary == [first, second]
        assert state.network_missing[StreamType.AFE] == 1

        session_id = state.session_id
        state.disconnect()
        stop, address = state.transport.sent[-1]
        assert stop == {"type": "stop", "session_id": session_id}
        assert address == ("192.168.1.20", 8001)

    asyncio.run(scenario())
