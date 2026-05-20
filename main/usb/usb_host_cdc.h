// =============================================================================
//  usb_host_cdc.h - USB Host CDC/VCP transport for the OBDLink EX.
// -----------------------------------------------------------------------------
//  Thin C++ wrapper over Espressif's `usb_host_vcp` stack. The VCP layer
//  auto-selects a line driver (FTDI for the genuine OBDLink EX, CP210x/CH34x
//  for clones) once the corresponding driver components are registered.
//
//  Responsibilities:
//    * Install the USB Host library and pump its event loop on a daemon task.
//    * Handle hot-plug: open the device when attached, tear down on removal.
//    * Funnel all received bytes into a FreeRTOS StreamBuffer (the RX byte
//      ring) for the OBD parser to consume.
//    * Provide a blocking-with-timeout write() for command transmission.
//    * Allow runtime line-coding (baud) changes for the Settings screen.
//
//  Threading: the USB stack's RX callback runs in the cdc_acm host task
//  context. We do nothing heavy there - just an xStreamBufferSendFromISR-style
//  copy into the RX ring - so the host task stays responsive.
// =============================================================================
#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>

#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"

class UsbHostCdc {
public:
    // Invoked (from the USB daemon context) on attach/detach transitions so
    // the owner can advance its connection state machine.
    using ConnChangeCb = std::function<void(bool connected)>;

    UsbHostCdc() = default;
    ~UsbHostCdc();

    // Install host stack + line drivers, allocate the RX stream buffer, and
    // start the daemon/hotplug task. Idempotent. Returns false on failure.
    bool init(size_t rx_stream_bytes, ConnChangeCb on_change);

    // True once a VCP device is open and ready for I/O.
    bool isOpen() const { return dev_ != nullptr; }

    // Transmit `len` bytes. Returns bytes written, or -1 on error/closed.
    int  write(const uint8_t* data, size_t len, uint32_t timeout_ms);

    // Read up to `max` bytes from the RX stream. Blocks up to timeout_ms for
    // at least one byte. Returns count read (0 on timeout).
    size_t read(uint8_t* out, size_t max, uint32_t timeout_ms);

    // Change line coding (baud rate). No-op if closed. Returns success.
    bool setBaud(uint32_t baud);

    // Drop everything currently buffered in the RX ring (used between commands
    // to discard stale prompts/echoes).
    void flushRx();

    // Force-close the current device (auto-reconnect will reopen on next plug).
    void close();

private:
    // --- VCP callbacks (static trampolines into the instance) ---------------
    static bool rxTrampoline(const uint8_t* data, size_t len, void* arg);
    static void devEventTrampoline(void* arg);

    // The daemon task that runs usb_host_lib_handle_events() forever.
    static void daemonTask(void* arg);
    // The hotplug task that (re)opens the VCP device when one appears.
    static void hotplugTask(void* arg);

    bool                 handleRx(const uint8_t* data, size_t len);

    void*                dev_         = nullptr;   // CdcAcmDevice* (opaque here)
    StreamBufferHandle_t rx_stream_   = nullptr;
    ConnChangeCb         on_change_;
    uint32_t             baud_        = 0;
    volatile bool        device_present_ = false;  // set by hotplug callback
    bool                 installed_   = false;
};
