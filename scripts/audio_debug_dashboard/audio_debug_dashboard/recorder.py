from __future__ import annotations

import json
import struct
import time
import wave
from dataclasses import dataclass, field
from pathlib import Path

from .protocol import Packet, StreamType

STREAM_NAMES = {
    StreamType.RAW_MICS: "raw_mics",
    StreamType.BEAM: "beam",
    StreamType.AFE: "afe",
    StreamType.DIRECTION: "direction",
}


@dataclass
class StreamState:
    expected_sequence: int | None = None
    pending: dict[int, Packet] = field(default_factory=dict)
    wav_file: wave.Wave_write | None = None
    last_end_timestamp_us: int | None = None
    received_packets: int = 0
    missing_packets: int = 0
    inserted_silence_frames: int = 0
    late_packets: int = 0


class SessionRecorder:
    def __init__(
        self,
        root: Path,
        session_id: int,
        reorder_window: int = 8,
        device_ip: str = "",
    ):
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        self.directory = Path(root) / f"{timestamp}-{session_id:08x}"
        self.directory.mkdir(parents=True, exist_ok=False)
        self.session_id = session_id
        self.device_ip = device_ip
        self.reorder_window = max(0, reorder_window)
        self.states = {stream: StreamState() for stream in StreamType}
        self.direction_file = (self.directory / "direction.jsonl").open("w", encoding="utf-8")
        self.closed = False

    def write(self, packet: Packet) -> None:
        if self.closed or packet.session_id != self.session_id:
            return
        state = self.states[packet.stream]
        state.received_packets += 1
        if state.expected_sequence is None:
            state.expected_sequence = packet.sequence
        if packet.sequence < state.expected_sequence:
            state.late_packets += 1
            return
        state.pending[packet.sequence] = packet
        self._drain(packet.stream, force=False)

    def _drain(self, stream: StreamType, force: bool) -> None:
        state = self.states[stream]
        while state.pending and state.expected_sequence is not None:
            packet = state.pending.pop(state.expected_sequence, None)
            if packet is not None:
                self._write_packet(stream, state, packet)
                state.expected_sequence += 1
                continue

            if not force and len(state.pending) <= self.reorder_window:
                break

            next_sequence = min(state.pending)
            next_packet = state.pending[next_sequence]
            state.missing_packets += next_sequence - state.expected_sequence
            self._insert_timestamp_gap(state, next_packet)
            state.expected_sequence = next_sequence

    def _open_wav(self, stream: StreamType, packet: Packet, state: StreamState) -> None:
        path = self.directory / f"{STREAM_NAMES[stream]}.wav"
        state.wav_file = wave.open(str(path), "wb")
        state.wav_file.setnchannels(packet.channels)
        state.wav_file.setsampwidth(2)
        state.wav_file.setframerate(packet.sample_rate)

    def _write_packet(self, stream: StreamType, state: StreamState, packet: Packet) -> None:
        if stream is StreamType.DIRECTION:
            if len(packet.payload) != 12:
                return
            angle, confidence, active = struct.unpack("<ffB3x", packet.payload)
            self.direction_file.write(
                json.dumps(
                    {
                        "sequence": packet.sequence,
                        "timestamp_us": packet.timestamp_us,
                        "angle_deg": angle,
                        "confidence": confidence,
                        "active": bool(active),
                    },
                    separators=(",", ":"),
                )
                + "\n"
            )
            return

        if state.wav_file is None:
            self._open_wav(stream, packet, state)
        state.wav_file.writeframesraw(packet.payload)
        duration_us = round(packet.frame_count * 1_000_000 / packet.sample_rate)
        state.last_end_timestamp_us = packet.timestamp_us + duration_us

    def _insert_timestamp_gap(self, state: StreamState, next_packet: Packet) -> None:
        if state.wav_file is None or state.last_end_timestamp_us is None:
            return
        gap_us = max(0, next_packet.timestamp_us - state.last_end_timestamp_us)
        gap_frames = round(gap_us * next_packet.sample_rate / 1_000_000)
        if gap_frames:
            state.wav_file.writeframesraw(b"\0" * gap_frames * next_packet.channels * 2)
            state.inserted_silence_frames += gap_frames
            state.last_end_timestamp_us += round(gap_frames * 1_000_000 / next_packet.sample_rate)

    def close(self) -> Path:
        if self.closed:
            return self.directory
        for stream in StreamType:
            self._drain(stream, force=True)
        for state in self.states.values():
            if state.wav_file is not None:
                state.wav_file.close()
        self.direction_file.close()
        metadata = {
            "protocol_version": 1,
            "session_id": self.session_id,
            "device_ip": self.device_ip,
            "streams": {
                STREAM_NAMES[stream]: {
                    "received_packets": state.received_packets,
                    "missing_packets": state.missing_packets,
                    "inserted_silence_frames": state.inserted_silence_frames,
                    "late_packets": state.late_packets,
                }
                for stream, state in self.states.items()
            },
        }
        (self.directory / "session.json").write_text(
            json.dumps(metadata, indent=2), encoding="utf-8"
        )
        self.closed = True
        return self.directory
