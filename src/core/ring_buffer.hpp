#pragma once
#include <atomic>
#include <cstddef>
#include <memory>
#include <algorithm>

namespace openCREST {

// Single-producer, single-consumer lock-free ring buffer.
//
// Thread-safety contract:
//   * write() / available_write() — called ONLY from the producer thread.
//   * read()  / available_read()  — called ONLY from the consumer thread.
//   * capacity() / reset()        — call only when no concurrent access.
//
// Capacity is rounded up to the next power of two so that index masking
// replaces modulo division on the hot path.
template<typename T>
class SPSCRingBuffer {
public:
    explicit SPSCRingBuffer(size_t capacity) {
        // Round up to power of two
        size_t cap = 1;
        while (cap < capacity) cap <<= 1;
        capacity_ = cap;
        mask_     = cap - 1;
        buffer_   = std::make_unique<T[]>(cap);
    }

    // Not copyable; ring buffers are owned resources tied to specific threads.
    SPSCRingBuffer(const SPSCRingBuffer&)            = delete;
    SPSCRingBuffer& operator=(const SPSCRingBuffer&) = delete;

    // Movable (before any threads touch it).
    // std::atomic is not movable, so we manually transfer the values.
    SPSCRingBuffer(SPSCRingBuffer&& other) noexcept
        : write_pos_(other.write_pos_.load(std::memory_order_relaxed))
        , read_pos_(other.read_pos_.load(std::memory_order_relaxed))
        , capacity_(other.capacity_)
        , mask_(other.mask_)
        , buffer_(std::move(other.buffer_))
    {
        other.capacity_ = 0;
        other.mask_ = 0;
    }

    SPSCRingBuffer& operator=(SPSCRingBuffer&& other) noexcept {
        if (this != &other) {
            write_pos_.store(other.write_pos_.load(std::memory_order_relaxed),
                             std::memory_order_relaxed);
            read_pos_.store(other.read_pos_.load(std::memory_order_relaxed),
                            std::memory_order_relaxed);
            capacity_ = other.capacity_;
            mask_ = other.mask_;
            buffer_ = std::move(other.buffer_);
            other.capacity_ = 0;
            other.mask_ = 0;
        }
        return *this;
    }

    // ---------------------------------------------------------------------------
    // Producer interface — call only from the single producer thread
    // ---------------------------------------------------------------------------

    // Slots available for writing without blocking.
    size_t available_write() const {
        const size_t wp = write_pos_.load(std::memory_order_relaxed);
        const size_t rp = read_pos_.load(std::memory_order_acquire);
        return capacity_ - (wp - rp);
    }

    // Write up to `count` items from `data`. Returns the number written.
    // Returns less than `count` only when the buffer is full (never blocks).
    size_t write(const T* data, size_t count) {
        const size_t wp        = write_pos_.load(std::memory_order_relaxed);
        const size_t rp        = read_pos_.load(std::memory_order_acquire);
        const size_t available = capacity_ - (wp - rp);
        const size_t to_write  = std::min(count, available);

        for (size_t i = 0; i < to_write; ++i) {
            buffer_[(wp + i) & mask_] = data[i];
        }

        write_pos_.store(wp + to_write, std::memory_order_release);
        return to_write;
    }

    // ---------------------------------------------------------------------------
    // Consumer interface — call only from the single consumer thread
    // ---------------------------------------------------------------------------

    // Items available for reading.
    size_t available_read() const {
        const size_t rp = read_pos_.load(std::memory_order_relaxed);
        const size_t wp = write_pos_.load(std::memory_order_acquire);
        return wp - rp;
    }

    // Read up to `count` items into `data`. Returns the number read.
    // Returns less than `count` only when the buffer has fewer items available.
    size_t read(T* data, size_t count) {
        const size_t rp        = read_pos_.load(std::memory_order_relaxed);
        const size_t wp        = write_pos_.load(std::memory_order_acquire);
        const size_t available = wp - rp;
        const size_t to_read   = std::min(count, available);

        for (size_t i = 0; i < to_read; ++i) {
            data[i] = buffer_[(rp + i) & mask_];
        }

        read_pos_.store(rp + to_read, std::memory_order_release);
        return to_read;
    }

    // ---------------------------------------------------------------------------
    // Accessors — safe to call from either thread when there is no concurrent
    // write/read in progress
    // ---------------------------------------------------------------------------

    size_t capacity() const { return capacity_; }

    // Reset to empty state. Only call when no concurrent access is happening.
    void reset() {
        write_pos_.store(0, std::memory_order_relaxed);
        read_pos_.store(0, std::memory_order_relaxed);
    }

private:
    // Producer-owned cache line
    alignas(64) std::atomic<size_t> write_pos_{0};
    // Consumer-owned cache line
    alignas(64) std::atomic<size_t> read_pos_{0};

    size_t              capacity_ = 0;
    size_t              mask_     = 0;
    std::unique_ptr<T[]> buffer_;
};

} // namespace openCREST
