from __future__ import annotations

import enum
import struct
from dataclasses import dataclass

MAGIC = 0x47424441
VERSION = 1
HEADER_SIZE = 40
MAX_DATAGRAM_SIZE = 1472
SAMPLE_FORMAT_S16_LE = 1
HEADER = struct.Struct("<IBBHIIQI HBBHHI".replace(" ", ""))


class StreamType(enum.IntEnum):
    RAW_MICS = 1
    BEAM = 2
    AFE = 3
    DIRECTION = 4


@dataclass(frozen=True)
class Packet:
    stream: StreamType
    session_id: int
    sequence: int
    timestamp_us: int
    sample_rate: int
    frame_count: int
    channels: int
    sample_format: int
    flags: int
    payload: bytes

    def encode(self) -> bytes:
        return build_packet(
            self.stream,
            self.session_id,
            self.sequence,
            self.timestamp_us,
            self.sample_rate,
            self.frame_count,
            self.channels,
            self.payload,
            self.sample_format,
            self.flags,
        )


def build_packet(
    stream: StreamType,
    session_id: int,
    sequence: int,
    timestamp_us: int,
    sample_rate: int,
    frame_count: int,
    channels: int,
    payload: bytes,
    sample_format: int = SAMPLE_FORMAT_S16_LE,
    flags: int = 0,
) -> bytes:
    if HEADER_SIZE + len(payload) > MAX_DATAGRAM_SIZE:
        raise ValueError("packet exceeds UDP datagram limit")
    header = HEADER.pack(
        MAGIC,
        VERSION,
        int(stream),
        HEADER_SIZE,
        session_id,
        sequence,
        timestamp_us,
        sample_rate,
        frame_count,
        channels,
        sample_format,
        len(payload),
        flags,
        0,
    )
    return header + payload


def parse_packet(data: bytes) -> Packet:
    if len(data) < HEADER_SIZE:
        raise ValueError("truncated header")
    (
        magic,
        version,
        stream,
        header_size,
        session_id,
        sequence,
        timestamp_us,
        sample_rate,
        frame_count,
        channels,
        sample_format,
        payload_size,
        flags,
        _reserved,
    ) = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise ValueError("invalid magic")
    if version != VERSION:
        raise ValueError(f"unsupported protocol version {version}")
    if header_size != HEADER_SIZE:
        raise ValueError("invalid header size")
    if len(data) != HEADER_SIZE + payload_size:
        raise ValueError("invalid payload size")
    if sample_format != SAMPLE_FORMAT_S16_LE and stream != StreamType.DIRECTION:
        raise ValueError("unsupported sample format")
    try:
        stream_type = StreamType(stream)
    except ValueError as exc:
        raise ValueError(f"unknown stream {stream}") from exc
    return Packet(
        stream_type,
        session_id,
        sequence,
        timestamp_us,
        sample_rate,
        frame_count,
        channels,
        sample_format,
        flags,
        data[HEADER_SIZE:],
    )
