#pragma once
#include <fstream>
#include <string>
#include <cstdint>
#include <cstddef>

namespace openCREST::logging {

// Minimal 16-bit mono PCM WAV writer.
//
// Writes the 44-byte header at construction with a zero data-size placeholder.
// finalize() patches the header's RIFF and data chunk sizes with the actual
// byte count. finalize() is also called automatically in the destructor.
//
// Thread-safety: not thread-safe. Intended to be used from one I/O thread.
class WavWriter {
public:
    // `source_bits` is the effective precision of incoming uint16_t samples
    // (ADC = 16, DAC = 12, etc.). WAV always stores 16-bit signed PCM;
    // samples are left-shifted to fill the top of the 16-bit field so
    // playback amplitude matches the acoustic signal instead of bunching
    // at one rail.
    WavWriter(const std::string& path, uint32_t sample_rate,
              uint8_t source_bits = 16);
    ~WavWriter();

    WavWriter(const WavWriter&)            = delete;
    WavWriter& operator=(const WavWriter&) = delete;

    // Write uint16_t PCM samples. Returns number of samples written.
    size_t write(const uint16_t* samples, size_t count);

    // Patch the WAV header with the actual data size. Idempotent.
    void finalize();

    bool   is_open()     const;
    size_t sample_count() const { return sample_count_; }
    const std::string& path() const { return path_; }

private:
    void write_placeholder_header();
    void patch_header();
    void write_le16(uint16_t v);
    void write_le32(uint32_t v);

    std::string   path_;
    std::fstream  file_;
    uint32_t      sample_rate_;
    uint8_t       source_shift_ = 0;    // left-shift applied to each sample
    size_t        sample_count_ = 0;
    bool          finalized_    = false;

    // Byte offsets within the WAV header that need patching
    static constexpr long RIFF_SIZE_OFFSET = 4;
    static constexpr long DATA_SIZE_OFFSET = 40;
    static constexpr size_t HEADER_BYTES   = 44;
};

} // namespace openCREST::logging
