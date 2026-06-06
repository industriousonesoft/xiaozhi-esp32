import struct

import pytest

from audio_debug_dashboard.protocol import (
    HEADER_SIZE,
    MAGIC,
    Packet,
    StreamType,
    build_packet,
    parse_packet,
)


def test_round_trip_audio_packet():
    payload = struct.pack("<6h", 1, -2, 3, -4, 5, -6)
    encoded = build_packet(
        stream=StreamType.RAW_MICS,
        session_id=0x12345678,
        sequence=9,
        timestamp_us=123_000,
        sample_rate=48_000,
        frame_count=2,
        channels=3,
        payload=payload,
    )

    packet = parse_packet(encoded)

    assert packet.stream is StreamType.RAW_MICS
    assert packet.session_id == 0x12345678
    assert packet.sequence == 9
    assert packet.timestamp_us == 123_000
    assert packet.sample_rate == 48_000
    assert packet.frame_count == 2
    assert packet.channels == 3
    assert packet.payload == payload


def test_rejects_bad_magic_and_truncated_payload():
    encoded = bytearray(
        build_packet(StreamType.AFE, 1, 0, 0, 16_000, 2, 1, b"\0\0\0\0")
    )
    encoded[0:4] = struct.pack("<I", MAGIC + 1)
    with pytest.raises(ValueError, match="magic"):
        parse_packet(bytes(encoded))

    valid = build_packet(StreamType.AFE, 1, 0, 0, 16_000, 2, 1, b"\0\0\0\0")
    with pytest.raises(ValueError, match="payload"):
        parse_packet(valid[:-1])


def test_header_size_matches_firmware_contract():
    packet = Packet(StreamType.BEAM, 1, 2, 3, 48_000, 1, 1, 1, 0, b"\0\0")
    assert len(packet.encode()) == HEADER_SIZE + 2
