#include "logging/wav_writer.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cstring>

namespace openCREST::logging {

// ---------------------------------------------------------------------------
// WAV header layout (44 bytes, all fields little-endian):
//   Offset  0: "RIFF"
//   Offset  4: file_size - 8  (patched at finalize)
//   Offset  8: "WAVE"
//   Offset 12: "fmt "
//   Offset 16: 16             (PCM chunk size)
//   Offset 18: 1              (PCM format)
//   Offset 20: 1              (channels: mono)
//   Offset 22: sample_rate
//   Offset 26: byte_rate = sample_rate * 2
//   Offset 30: 2              (block align: 1 channel × 2 bytes)
//   Offset 32: 16             (bits per sample)
//   Offset 36: "data"
//   Offset 40: data_bytes     (patched at finalize)
//   Offset 44: <samples>
// ---------------------------------------------------------------------------

WavWriter::WavWriter(const std::string& path, uint32_t sample_rate,
                     uint8_t source_bits)
    : path_(path)
    , sample_rate_(sample_rate)
    , source_shift_(source_bits >= 16 ? 0
                                      : static_cast<uint8_t>(16 - source_bits))
{
    file_.open(path, std::ios::in | std::ios::out | std::ios::binary | std::ios::trunc);
    if (!file_.is_open()) {
        // Try without in flag (file may not exist yet)
        file_.clear();
        file_.open(path, std::ios::out | std::ios::binary | std::ios::trunc);
        if (!file_.is_open()) {
            spdlog::error("WavWriter: cannot open '{}'", path);
            return;
        }
        // Reopen for read+write so we can seek back to patch header
        file_.close();
        file_.open(path, std::ios::in | std::ios::out | std::ios::binary);
    }

    if (!file_.is_open()) {
        spdlog::error("WavWriter: cannot reopen '{}' for read+write", path);
        return;
    }

    write_placeholder_header();
    spdlog::debug("WavWriter: opened '{}'", path);
}

WavWriter::~WavWriter() {
    finalize();
}

size_t WavWriter::write(const uint16_t* samples, size_t count) {
    if (!is_open() || count == 0) return 0;

    // Samples arrive as unsigned N-bit codes (N = source_bits) stored in the
    // low bits of uint16_t. Left-shift to fill the top of the 16-bit field,
    // then XOR 0x8000 to convert unsigned-midpoint to signed-zero PCM16.
    constexpr size_t CHUNK = 512;
    uint16_t buf[CHUNK];
    const unsigned shift = source_shift_;
    size_t written = 0;
    while (written < count) {
        const size_t n = std::min(CHUNK, count - written);
        for (size_t i = 0; i < n; ++i) {
            const uint16_t s = static_cast<uint16_t>(samples[written + i] << shift);
            buf[i] = static_cast<uint16_t>(s ^ 0x8000u);
        }
        file_.write(reinterpret_cast<const char*>(buf), n * sizeof(uint16_t));
        if (!file_.good()) {
            spdlog::warn("WavWriter: write error on '{}'", path_);
            return written;
        }
        written += n;
    }
    sample_count_ += written;
    return written;
}

void WavWriter::finalize() {
    if (finalized_ || !is_open()) return;
    finalized_ = true;
    patch_header();
    file_.flush();
    file_.close();
    spdlog::debug("WavWriter: finalized '{}' ({} samples)", path_, sample_count_);
}

bool WavWriter::is_open() const {
    return file_.is_open();
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void WavWriter::write_le16(uint16_t v) {
    const char bytes[2] = {
        static_cast<char>(v & 0xFF),
        static_cast<char>((v >> 8) & 0xFF)
    };
    file_.write(bytes, 2);
}

void WavWriter::write_le32(uint32_t v) {
    const char bytes[4] = {
        static_cast<char>(v & 0xFF),
        static_cast<char>((v >> 8)  & 0xFF),
        static_cast<char>((v >> 16) & 0xFF),
        static_cast<char>((v >> 24) & 0xFF)
    };
    file_.write(bytes, 4);
}

void WavWriter::write_placeholder_header() {
    file_.seekp(0);

    // RIFF chunk
    file_.write("RIFF", 4);
    write_le32(0);            // placeholder: file_size - 8
    file_.write("WAVE", 4);

    // fmt sub-chunk
    file_.write("fmt ", 4);
    write_le32(16);           // PCM chunk size
    write_le16(1);            // PCM format
    write_le16(1);            // mono
    write_le32(sample_rate_);
    write_le32(sample_rate_ * 2u); // byte rate = sample_rate × channels × bytes_per_sample
    write_le16(2);            // block align
    write_le16(16);           // bits per sample

    // data sub-chunk header
    file_.write("data", 4);
    write_le32(0);            // placeholder: data size in bytes
}

void WavWriter::patch_header() {
    if (!file_.is_open()) return;

    const uint32_t data_bytes = static_cast<uint32_t>(sample_count_ * sizeof(uint16_t));
    const uint32_t riff_size  = static_cast<uint32_t>(HEADER_BYTES - 8 + data_bytes);

    file_.seekp(RIFF_SIZE_OFFSET);
    write_le32(riff_size);

    file_.seekp(DATA_SIZE_OFFSET);
    write_le32(data_bytes);
}

} // namespace openCREST::logging
