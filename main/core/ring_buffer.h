// =============================================================================
//  ring_buffer.h - PSRAM-backed SPSC ring of fixed-size CAN frame slots.
// -----------------------------------------------------------------------------
//  Why not a FreeRTOS Queue? At full STMA monitor load we can see >5k frames/s.
//  A copy-by-value FreeRTOS queue with per-item mutexing adds avoidable
//  overhead. This is a single-producer (OBD task) / multi-consumer-friendly
//  ring with atomic head/tail indices, lock-free on the hot producer path.
//
//  Capacity is rounded up to a power of two so index wrap is a cheap mask.
//  The backing store is allocated from PSRAM (frames are bulky and cold for
//  the CPU cache once written).
//
//  Concurrency model:
//    * Exactly one producer thread calls push().
//    * One or more consumer threads call pop(); consumers must serialize
//      among themselves (the sniffer UI drains it; the SD logger uses its own
//      ring). For the two-consumer fan-out we instantiate two rings and the
//      producer pushes to both.
// =============================================================================
#pragma once

#include <atomic>
#include <cstdint>
#include "core/can_types.h"

class FrameRing {
public:
    FrameRing() = default;
    ~FrameRing();

    // Allocate the backing store (power-of-two >= requested_slots) in PSRAM.
    // Returns false if allocation fails; the object stays unusable but safe.
    bool init(size_t requested_slots);

    // Producer: copy `frame` into the ring. Returns false if full (caller
    // should bump the dropped-frame counter). Never blocks.
    bool push(const can_frame_t& frame);

    // Consumer: pop oldest frame into `out`. Returns false if empty.
    bool pop(can_frame_t& out);

    // Approximate number of queued frames (safe to call from any thread).
    size_t size() const;
    bool   empty() const { return size() == 0; }
    size_t capacity() const { return capacity_; }

    // Discard everything (used by the sniffer "clear" action). Producer must
    // be quiescent or tolerate the racy reset; we only move the tail.
    void clear() { tail_.store(head_.load(std::memory_order_acquire),
                               std::memory_order_release); }

private:
    can_frame_t*               buf_     = nullptr;
    size_t                     capacity_ = 0;   // power of two
    size_t                     mask_     = 0;   // capacity_ - 1
    std::atomic<size_t>        head_{0};        // next write index
    std::atomic<size_t>        tail_{0};        // next read index
};
