#include "audio_debug_protocol.h"

#include <algorithm>
#include <cstring>

namespace {
void WriteU16(uint8_t* output, uint16_t value) {
    output[0] = static_cast<uint8_t>(value);
    output[1] = static_cast<uint8_t>(value >> 8);
}

void WriteU32(uint8_t* output, uint32_t value) {
    for (int i = 0; i < 4; ++i) {
        output[i] = static_cast<uint8_t>(value >> (i * 8));
    }
}

void WriteU64(uint8_t* output, uint64_t value) {
    for (int i = 0; i < 8; ++i) {
        output[i] = static_cast<uint8_t>(value >> (i * 8));
    }
}

uint16_t ReadU16(const uint8_t* input) {
    return static_cast<uint16_t>(input[0]) |
        (static_cast<uint16_t>(input[1]) << 8);
}

uint32_t ReadU32(const uint8_t* input) {
    uint32_t value = 0;
    for (int i = 0; i < 4; ++i) {
        value |= static_cast<uint32_t>(input[i]) << (i * 8);
    }
    return value;
}

uint64_t ReadU64(const uint8_t* input) {
    uint64_t value = 0;
    for (int i = 0; i < 8; ++i) {
        value |= static_cast<uint64_t>(input[i]) << (i * 8);
    }
    return value;
}
} // namespace

bool EncodeAudioDebugHeader(const AudioDebugPacketHeader& header, uint8_t* output, size_t output_size) {
    if (output == nullptr || output_size < kAudioDebugHeaderSize ||
        header.payload_size + kAudioDebugHeaderSize > kAudioDebugMaxDatagramSize) {
        return false;
    }

    WriteU32(output, kAudioDebugMagic);
    output[4] = kAudioDebugProtocolVersion;
    output[5] = static_cast<uint8_t>(header.stream);
    WriteU16(output + 6, kAudioDebugHeaderSize);
    WriteU32(output + 8, header.session_id);
    WriteU32(output + 12, header.sequence);
    WriteU64(output + 16, header.timestamp_us);
    WriteU32(output + 24, header.sample_rate);
    WriteU16(output + 28, header.frame_count);
    output[30] = header.channels;
    output[31] = header.sample_format;
    WriteU16(output + 32, header.payload_size);
    WriteU16(output + 34, header.flags);
    WriteU32(output + 36, 0);
    return true;
}

bool DecodeAudioDebugHeader(const uint8_t* data, size_t size, AudioDebugPacketHeader& header) {
    if (data == nullptr || size < kAudioDebugHeaderSize ||
        ReadU32(data) != kAudioDebugMagic ||
        data[4] != kAudioDebugProtocolVersion ||
        ReadU16(data + 6) != kAudioDebugHeaderSize) {
        return false;
    }

    const uint16_t payload_size = ReadU16(data + 32);
    if (payload_size + kAudioDebugHeaderSize > kAudioDebugMaxDatagramSize) {
        return false;
    }

    header.stream = static_cast<AudioDebugStream>(data[5]);
    header.session_id = ReadU32(data + 8);
    header.sequence = ReadU32(data + 12);
    header.timestamp_us = ReadU64(data + 16);
    header.sample_rate = ReadU32(data + 24);
    header.frame_count = ReadU16(data + 28);
    header.channels = data[30];
    header.sample_format = data[31];
    header.payload_size = payload_size;
    header.flags = ReadU16(data + 34);
    return true;
}

size_t AudioDebugMaxFramesPerPacket(uint8_t channels) {
    if (channels == 0) {
        return 0;
    }
    return (kAudioDebugMaxDatagramSize - kAudioDebugHeaderSize) /
        (sizeof(int16_t) * channels);
}

std::vector<AudioDebugPcmChunk> SplitAudioDebugPcm(const int16_t* samples, size_t frames, uint8_t channels) {
    std::vector<AudioDebugPcmChunk> chunks;
    const size_t max_frames = AudioDebugMaxFramesPerPacket(channels);
    if (samples == nullptr || frames == 0 || max_frames == 0) {
        return chunks;
    }

    for (size_t offset = 0; offset < frames; offset += max_frames) {
        chunks.push_back({offset, std::min(max_frames, frames - offset)});
    }
    return chunks;
}

AudioDebugPacketBuilder::AudioDebugPacketBuilder(uint32_t session_id)
    : session_id_(session_id) {
}

std::vector<std::vector<uint8_t>> AudioDebugPacketBuilder::BuildPcmPackets(
    AudioDebugStream stream, const int16_t* samples, size_t frames, uint8_t channels,
    uint32_t sample_rate, uint64_t timestamp_us) {
    std::vector<std::vector<uint8_t>> packets;
    if (sample_rate == 0) {
        return packets;
    }
    for (const auto& chunk : SplitAudioDebugPcm(samples, frames, channels)) {
        const size_t payload_size = chunk.frame_count * channels * sizeof(int16_t);
        std::vector<uint8_t> packet(kAudioDebugHeaderSize + payload_size);
        AudioDebugPacketHeader header;
        header.stream = stream;
        header.session_id = session_id_;
        header.sequence = NextSequence(stream);
        header.timestamp_us = timestamp_us + chunk.frame_offset * 1000000ULL / sample_rate;
        header.sample_rate = sample_rate;
        header.frame_count = static_cast<uint16_t>(chunk.frame_count);
        header.channels = channels;
        header.payload_size = static_cast<uint16_t>(payload_size);
        if (!EncodeAudioDebugHeader(header, packet.data(), packet.size())) {
            continue;
        }
        std::memcpy(packet.data() + kAudioDebugHeaderSize,
                    samples + chunk.frame_offset * channels, payload_size);
        packets.push_back(std::move(packet));
    }
    return packets;
}

std::vector<uint8_t> AudioDebugPacketBuilder::BuildDirectionPacket(
    float angle_deg, float confidence, bool active, uint64_t timestamp_us) {
    constexpr size_t kPayloadSize = 12;
    std::vector<uint8_t> packet(kAudioDebugHeaderSize + kPayloadSize, 0);
    AudioDebugPacketHeader header;
    header.stream = AudioDebugStream::kDirection;
    header.session_id = session_id_;
    header.sequence = NextSequence(header.stream);
    header.timestamp_us = timestamp_us;
    header.payload_size = kPayloadSize;
    header.sample_format = 0;
    if (!EncodeAudioDebugHeader(header, packet.data(), packet.size())) {
        return {};
    }
    std::memcpy(packet.data() + kAudioDebugHeaderSize, &angle_deg, sizeof(angle_deg));
    std::memcpy(packet.data() + kAudioDebugHeaderSize + 4, &confidence, sizeof(confidence));
    packet[kAudioDebugHeaderSize + 8] = active ? 1 : 0;
    return packet;
}

uint32_t AudioDebugPacketBuilder::NextSequence(AudioDebugStream stream) {
    const auto index = static_cast<uint8_t>(stream) - 1;
    if (index >= 4) {
        return 0;
    }
    return sequences_[index]++;
}
