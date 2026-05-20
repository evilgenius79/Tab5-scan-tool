// =============================================================================
//  ring_buffer.cpp - implementation of the PSRAM SPSC frame ring.
// =============================================================================
#include "core/ring_buffer.h"

#include <cstring>
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char* TAG = "FrameRing";

// Round up to the next power of two (>= 2).
static size_t next_pow2(size_t v) {
    size_t p = 2;
    while (p < v) p <<= 1;
    return p;
}

FrameRing::~FrameRing() {
    if (buf_) {
        heap_caps_free(buf_);
        buf_ = nullptr;
    }
}

bool FrameRing::init(size_t requested_slots) {
    capacity_ = next_pow2(requested_slots);
    mask_     = capacity_ - 1;

    // Prefer PSRAM; these buffers are large and not latency-critical to fetch.
    buf_ = static_cast<can_frame_t*>(
        heap_caps_calloc(capacity_, sizeof(can_frame_t), MALLOC_CAP_SPIRAM));
    if (!buf_) {
        ESP_LOGE(TAG, "PSRAM alloc failed for %u slots, retrying internal",
                 (unsigned)capacity_);
        buf_ = static_cast<can_frame_t*>(
            heap_caps_calloc(capacity_, sizeof(can_frame_t), MALLOC_CAP_8BIT));
    }
    if (!buf_) {
        ESP_LOGE(TAG, "ring allocation failed");
        capacity_ = mask_ = 0;
        return false;
    }
    head_.store(0, std::memory_order_relaxed);
    tail_.store(0, std::memory_order_relaxed);
    ESP_LOGI(TAG, "ring ready: %u slots (%u bytes)",
             (unsigned)capacity_, (unsigned)(capacity_ * sizeof(can_frame_t)));
    return true;
}

bool FrameRing::push(const can_frame_t& frame) {
    if (!buf_) return false;
    const size_t head = head_.load(std::memory_order_relaxed);
    const size_t next = (head + 1) & mask_;
    // Full when advancing head would collide with the consumer's tail.
    if (next == tail_.load(std::memory_order_acquire)) {
        return false;
    }
    buf_[head] = frame;                                  // POD copy
    head_.store(next, std::memory_order_release);        // publish
    return true;
}

bool FrameRing::pop(can_frame_t& out) {
    if (!buf_) return false;
    const size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {
        return false;                                    // empty
    }
    out = buf_[tail];
    tail_.store((tail + 1) & mask_, std::memory_order_release);
    return true;
}

size_t FrameRing::size() const {
    if (!buf_) return 0;
    const size_t head = head_.load(std::memory_order_acquire);
    const size_t tail = tail_.load(std::memory_order_acquire);
    return (head - tail) & mask_;
}
