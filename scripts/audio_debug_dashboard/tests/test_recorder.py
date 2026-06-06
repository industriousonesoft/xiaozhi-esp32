import json
import struct
import wave

from audio_debug_dashboard.protocol import StreamType, build_packet, parse_packet
from audio_debug_dashboard.recorder import SessionRecorder


def pcm_packet(stream, sequence, timestamp_us, values, sample_rate=16_000, channels=1):
    payload = struct.pack(f"<{len(values)}h", *values)
    return parse_packet(
        build_packet(
            stream,
            session_id=7,
            sequence=sequence,
            timestamp_us=timestamp_us,
            sample_rate=sample_rate,
            frame_count=len(values) // channels,
            channels=channels,
            payload=payload,
        )
    )


def test_reorders_short_out_of_order_burst(tmp_path):
    recorder = SessionRecorder(tmp_path, session_id=7, reorder_window=2)
    recorder.write(pcm_packet(StreamType.AFE, 0, 0, [1, 2]))
    recorder.write(pcm_packet(StreamType.AFE, 2, 250, [5, 6]))
    recorder.write(pcm_packet(StreamType.AFE, 1, 125, [3, 4]))
    session_dir = recorder.close()

    with wave.open(str(session_dir / "afe.wav"), "rb") as wav_file:
        values = struct.unpack("<6h", wav_file.readframes(6))
    assert values == (1, 2, 3, 4, 5, 6)


def test_fills_confirmed_timestamp_gap_with_silence(tmp_path):
    recorder = SessionRecorder(tmp_path, session_id=7, reorder_window=1)
    recorder.write(pcm_packet(StreamType.AFE, 0, 0, [10, 11]))
    recorder.write(pcm_packet(StreamType.AFE, 2, 250, [20, 21]))
    recorder.write(pcm_packet(StreamType.AFE, 3, 375, [30, 31]))
    session_dir = recorder.close()

    with wave.open(str(session_dir / "afe.wav"), "rb") as wav_file:
        values = struct.unpack("<8h", wav_file.readframes(8))
    assert values == (10, 11, 0, 0, 20, 21, 30, 31)

    metadata = json.loads((session_dir / "session.json").read_text())
    assert metadata["streams"]["afe"]["missing_packets"] == 1
    assert metadata["streams"]["afe"]["inserted_silence_frames"] == 2
