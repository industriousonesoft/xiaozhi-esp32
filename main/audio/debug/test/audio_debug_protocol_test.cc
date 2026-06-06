#include "../audio_debug_protocol.h"

#include <cassert>
#include <cstdint>
#include <vector>

int main() {
    AudioDebugPacketHeader header;
    header.stream = AudioDebugStream::kRawMics;
    header.session_id = 0x12345678;
    header.sequence = 42;
    header.timestamp_us = 123456789;
    header.sample_rate = 48000;
    header.frame_count = 238;
    header.channels = 3;
    header.payload_size = 1428;

    uint8_t encoded[kAudioDebugHeaderSize] = {};
    assert(EncodeAudioDebugHeader(header, encoded, sizeof(encoded)));

    AudioDebugPacketHeader decoded;
    assert(DecodeAudioDebugHeader(encoded, sizeof(encoded), decoded));
    assert(decoded.stream == header.stream);
    assert(decoded.session_id == header.session_id);
    assert(decoded.sequence == header.sequence);
    assert(decoded.timestamp_us == header.timestamp_us);
    assert(decoded.sample_rate == header.sample_rate);
    assert(decoded.frame_count == header.frame_count);
    assert(decoded.channels == header.channels);
    assert(decoded.payload_size == header.payload_size);

    encoded[0] = 0;
    assert(!DecodeAudioDebugHeader(encoded, sizeof(encoded), decoded));
    assert(!DecodeAudioDebugHeader(encoded, kAudioDebugHeaderSize - 1, decoded));

    assert(AudioDebugMaxFramesPerPacket(3) == 238);
    assert(AudioDebugMaxFramesPerPacket(1) == 716);
    assert(AudioDebugMaxFramesPerPacket(0) == 0);

    std::vector<int16_t> raw(480 * 3);
    auto chunks = SplitAudioDebugPcm(raw.data(), 480, 3);
    assert(chunks.size() == 3);
    assert(chunks[0].frame_offset == 0);
    assert(chunks[0].frame_count == 238);
    assert(chunks[1].frame_offset == 238);
    assert(chunks[1].frame_count == 238);
    assert(chunks[2].frame_offset == 476);
    assert(chunks[2].frame_count == 4);

    std::vector<int16_t> mono(960);
    chunks = SplitAudioDebugPcm(mono.data(), 960, 1);
    assert(chunks.size() == 2);
    assert(chunks[0].frame_count == 716);
    assert(chunks[1].frame_count == 244);

    AudioDebugPacketBuilder builder(77);
    auto packets = builder.BuildPcmPackets(
        AudioDebugStream::kRawMics, raw.data(), 480, 3, 48000, 1'000'000);
    assert(packets.size() == 3);
    assert(packets[0].size() == kAudioDebugHeaderSize + 238 * 3 * sizeof(int16_t));
    assert(packets[2].size() == kAudioDebugHeaderSize + 4 * 3 * sizeof(int16_t));

    AudioDebugPacketHeader packet_header;
    assert(DecodeAudioDebugHeader(packets[0].data(), packets[0].size(), packet_header));
    assert(packet_header.session_id == 77);
    assert(packet_header.sequence == 0);
    assert(packet_header.timestamp_us == 1'000'000);
    assert(packet_header.frame_count == 238);
    assert(DecodeAudioDebugHeader(packets[1].data(), packets[1].size(), packet_header));
    assert(packet_header.sequence == 1);
    assert(packet_header.timestamp_us == 1'004'958);
    assert(DecodeAudioDebugHeader(packets[2].data(), packets[2].size(), packet_header));
    assert(packet_header.sequence == 2);

    auto next_packets = builder.BuildPcmPackets(
        AudioDebugStream::kRawMics, raw.data(), 1, 3, 48000, 2'000'000);
    assert(DecodeAudioDebugHeader(next_packets[0].data(), next_packets[0].size(), packet_header));
    assert(packet_header.sequence == 3);

    auto direction_packet = builder.BuildDirectionPacket(12.5f, 0.75f, true, 3'000'000);
    assert(DecodeAudioDebugHeader(direction_packet.data(), direction_packet.size(), packet_header));
    assert(packet_header.stream == AudioDebugStream::kDirection);
    assert(packet_header.sequence == 0);
    assert(packet_header.payload_size == 12);

    return 0;
}
