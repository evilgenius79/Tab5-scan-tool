// =============================================================================
//  usb_host_cdc.cpp - USB Host VCP transport implementation.
// -----------------------------------------------------------------------------
//  Built against Espressif's esp-usb components:
//     usb_host_vcp     - virtual COM port abstraction (esp_usb::VCP)
//     usb_host_ftdi    - FTDI line driver (genuine OBDLink EX)
//     usb_host_cp210x  - Silicon Labs line driver (clones)
//     usb_host_ch34x   - WCH line driver (clones)
//
//  The VCP::open() factory inspects the attached device's VID/PID and returns
//  a CdcAcmDevice* backed by whichever line driver matches. We register all
//  three so the tool "just works" with whatever adapter is plugged in.
// =============================================================================
#include "usb/usb_host_cdc.h"
#include "app_config.h"

#include <utility>
#include "freertos/task.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "bsp/esp-bsp.h"   // bsp_usb_host_start() - enables USB-A VBUS power

// esp-usb VCP C++ headers.
#include "usb/vcp.hpp"
#include "usb/vcp_ftdi.hpp"
#include "usb/vcp_cp210x.hpp"
#include "usb/vcp_ch34x.hpp"
#include "usb/cdc_acm_host.h"

using namespace esp_usb;

static const char* TAG = "UsbHostCdc";

UsbHostCdc::~UsbHostCdc() {
    close();
}

bool UsbHostCdc::init(size_t rx_stream_bytes, ConnChangeCb on_change) {
    if (installed_) return true;
    on_change_ = std::move(on_change);

    rx_stream_ = xStreamBufferCreate(rx_stream_bytes, 1);
    if (!rx_stream_) {
        ESP_LOGE(TAG, "RX stream buffer alloc failed (%u bytes)",
                 (unsigned)rx_stream_bytes);
        return false;
    }

    // Enable USB-A host VBUS power and install the USB Host library.
    // CRITICAL: on the Tab5 the USB-A port's 5V is gated by an IO-expander pin
    // (BSP_USB_EN). bsp_usb_host_start() flips that on (bsp_feature_enable),
    // installs the host library, AND spawns the lib event task. Calling
    // usb_host_install() directly (as we used to) leaves the port unpowered, so
    // the OBDLink EX never lights up or enumerates. Requires the BSP I2C bus,
    // which bsp_display_start() (run earlier in ui_init) has already brought up.
    esp_err_t err = bsp_usb_host_start(BSP_USB_HOST_POWER_MODE_USB_DEV, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bsp_usb_host_start: %s", esp_err_to_name(err));
        return false;
    }

    // CDC-ACM host driver underpins the VCP layer.
    const cdc_acm_host_driver_config_t cdc_cfg = {
        .driver_task_stack_size = 4096,
        .driver_task_priority   = PRIO_USB_RX,
        .xCoreID                = APP_CORE_IO,
        .new_dev_cb             = nullptr,
    };
    ESP_ERROR_CHECK(cdc_acm_host_install(&cdc_cfg));

    // Register concrete line drivers so VCP::open() can match the attached
    // adapter by VID/PID. FT23x covers the genuine OBDLink EX; the others are
    // fallbacks for clone adapters.
    VCP::register_driver<FT23x>();
    VCP::register_driver<CP210x>();
    VCP::register_driver<CH34x>();

    // NOTE: bsp_usb_host_start() already spawned the task that services
    // usb_host_lib_handle_events(), so we do NOT start our own daemon (only one
    // task may pump the host-lib event loop). We only run the hotplug opener.
    xTaskCreatePinnedToCore(hotplugTask, "usb_hotplug", 4096, this,
                            PRIO_USB_RX - 1, nullptr, APP_CORE_IO);

    installed_ = true;
    ESP_LOGI(TAG, "USB host transport installed");
    return true;
}

// ---------------------------------------------------------------------------
//  Daemon task: keep the host library's internal state machine turning and
//  release devices/clients as they detach.
// ---------------------------------------------------------------------------
void UsbHostCdc::daemonTask(void* arg) {
    auto* self = static_cast<UsbHostCdc*>(arg);
    (void)self;
    while (true) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        if (flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGD(TAG, "all USB devices freed");
        }
    }
}

// ---------------------------------------------------------------------------
//  Hotplug task: blocks in VCP::open() until a supported adapter appears,
//  wires up the RX callback, then sleeps until the device is lost - at which
//  point it loops to reopen. This is the auto-reconnect heart.
// ---------------------------------------------------------------------------
void UsbHostCdc::hotplugTask(void* arg) {
    auto* self = static_cast<UsbHostCdc*>(arg);

    while (true) {
        // Line coding: start at the configured high baud; OBDLink EX FTDI
        // copes with 2 Mbit. 8N1, no flow control.
        cdc_acm_line_coding_t line_coding = {
            .dwDTERate   = self->baud_ ? self->baud_ : OBD_DEFAULT_BAUD,
            .bCharFormat = 0,   // 1 stop bit
            .bParityType = 0,   // none
            .bDataBits   = 8,
        };

        const cdc_acm_host_device_config_t dev_cfg = {
            .connection_timeout_ms = 5000,
            .out_buffer_size       = 512,
            // 0 => use the IN endpoint's max packet size. The FTDI VCP driver
            // rejects an explicit 512 ("RX FIFO size 512 is not supported").
            .in_buffer_size        = 0,
            .event_cb              = nullptr,
            .data_cb               = &UsbHostCdc::rxTrampoline,
            .user_arg              = self,
        };

        ESP_LOGI(TAG, "waiting for OBD adapter...");
        // VCP::open blocks until a matching device enumerates (or errors). It
        // catches the driver constructor's throw and returns nullptr on failure.
        CdcAcmDevice* dev = VCP::open(&dev_cfg);
        if (!dev) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;   // retry
        }

        // Apply the UART line coding now that the interface is open.
        dev->line_coding_set(&line_coding);

        self->dev_  = dev;
        self->baud_ = line_coding.dwDTERate;
        self->device_present_ = true;
        self->flushRx();
        ESP_LOGI(TAG, "adapter connected @ %u baud",
                 (unsigned)self->baud_);
        if (self->on_change_) self->on_change_(true);

        // Idle while the device stays alive. The CDC stack will mark it gone
        // (rx/event errors flip device_present_ via close()).
        while (self->device_present_) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        ESP_LOGW(TAG, "adapter disconnected");
        if (self->on_change_) self->on_change_(false);
        // dev_ is torn down in close(); loop back to reopen.
    }
}

// ---------------------------------------------------------------------------
//  RX path: copy bytes straight into the stream buffer. Runs on the CDC host
//  task; keep it allocation-free and short.
// ---------------------------------------------------------------------------
bool UsbHostCdc::rxTrampoline(const uint8_t* data, size_t len, void* arg) {
    return static_cast<UsbHostCdc*>(arg)->handleRx(data, len);
}

bool UsbHostCdc::handleRx(const uint8_t* data, size_t len) {
    if (rx_stream_ && len) {
        // Best-effort: if the OBD parser has fallen behind and the ring is
        // full, drop the overflow rather than block the USB host task.
        xStreamBufferSend(rx_stream_, data, len, 0);
    }
    return true;   // tell the CDC driver we consumed the buffer
}

// ---------------------------------------------------------------------------
//  TX / RX public API
// ---------------------------------------------------------------------------
int UsbHostCdc::write(const uint8_t* data, size_t len, uint32_t timeout_ms) {
    if (!dev_) return -1;
    auto* dev = static_cast<CdcAcmDevice*>(dev_);
    esp_err_t err = dev->tx_blocking(const_cast<uint8_t*>(data), len,
                                     pdMS_TO_TICKS(timeout_ms));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "tx error: %s", esp_err_to_name(err));
        // A transmit failure usually means the device vanished.
        if (err == ESP_ERR_INVALID_STATE || err == ESP_ERR_TIMEOUT) {
            device_present_ = false;
        }
        return -1;
    }
    return static_cast<int>(len);
}

size_t UsbHostCdc::read(uint8_t* out, size_t max, uint32_t timeout_ms) {
    if (!rx_stream_) return 0;
    return xStreamBufferReceive(rx_stream_, out, max, pdMS_TO_TICKS(timeout_ms));
}

bool UsbHostCdc::setBaud(uint32_t baud) {
    if (!dev_) { baud_ = baud; return false; }
    auto* dev = static_cast<CdcAcmDevice*>(dev_);
    cdc_acm_line_coding_t lc = { .dwDTERate = baud, .bCharFormat = 0,
                                 .bParityType = 0, .bDataBits = 8 };
    if (dev->line_coding_set(&lc) == ESP_OK) {
        baud_ = baud;
        ESP_LOGI(TAG, "baud set to %u", (unsigned)baud);
        return true;
    }
    return false;
}

void UsbHostCdc::flushRx() {
    if (rx_stream_) xStreamBufferReset(rx_stream_);
}

void UsbHostCdc::close() {
    device_present_ = false;
    if (dev_) {
        auto* dev = static_cast<CdcAcmDevice*>(dev_);
        delete dev;          // CdcAcmDevice dtor closes the interface
        dev_ = nullptr;
    }
}
