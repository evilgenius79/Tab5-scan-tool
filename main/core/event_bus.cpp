// =============================================================================
//  event_bus.cpp - singleton coordination hub implementation.
// =============================================================================
#include "core/event_bus.h"
#include "app_config.h"
#include "esp_log.h"

static const char* TAG = "EventBus";

EventBus& EventBus::instance() {
    static EventBus inst;   // Meyers singleton; constructed on first use.
    return inst;
}

bool EventBus::init() {
    telem_mtx_ = xSemaphoreCreateMutex();
    dtc_mtx_   = xSemaphoreCreateMutex();
    veh_mtx_   = xSemaphoreCreateMutex();
    cmd_queue_ = xQueueCreate(OBD_CMD_QUEUE_DEPTH, sizeof(ObdCommand));

    if (!telem_mtx_ || !dtc_mtx_ || !veh_mtx_ || !cmd_queue_) {
        ESP_LOGE(TAG, "primitive allocation failed");
        return false;
    }
    if (!sniffer_ring_.init(FRAME_RING_SLOTS) ||
        !logger_ring_.init(FRAME_RING_SLOTS)) {
        ESP_LOGE(TAG, "frame ring allocation failed");
        return false;
    }
    memset(&telem_, 0, sizeof(telem_));
    ESP_LOGI(TAG, "event bus initialized");
    return true;
}

// --- Telemetry --------------------------------------------------------------
void EventBus::publish(const TelemetryState& s) {
    if (xSemaphoreTake(telem_mtx_, pdMS_TO_TICKS(5)) == pdTRUE) {
        telem_ = s;
        xSemaphoreGive(telem_mtx_);
    }
    // On contention we simply skip this publish; the next one is ~ms away.
}

TelemetryState EventBus::snapshot() {
    TelemetryState copy{};
    if (xSemaphoreTake(telem_mtx_, pdMS_TO_TICKS(5)) == pdTRUE) {
        copy = telem_;
        xSemaphoreGive(telem_mtx_);
    }
    return copy;
}

// --- Commands ---------------------------------------------------------------
bool EventBus::sendCommand(const ObdCommand& cmd, TickType_t wait) {
    return xQueueSend(cmd_queue_, &cmd, wait) == pdTRUE;
}

bool EventBus::recvCommand(ObdCommand& out, TickType_t wait) {
    return xQueueReceive(cmd_queue_, &out, wait) == pdTRUE;
}

// --- DTCs -------------------------------------------------------------------
void EventBus::setDtcs(const DtcRecord* recs, size_t count) {
    if (count > MAX_DTCS) count = MAX_DTCS;
    if (xSemaphoreTake(dtc_mtx_, pdMS_TO_TICKS(50)) == pdTRUE) {
        memcpy(dtcs_, recs, count * sizeof(DtcRecord));
        dtc_count_ = count;
        xSemaphoreGive(dtc_mtx_);
    }
}

size_t EventBus::getDtcs(DtcRecord* out, size_t max) {
    size_t n = 0;
    if (xSemaphoreTake(dtc_mtx_, pdMS_TO_TICKS(50)) == pdTRUE) {
        n = (dtc_count_ < max) ? dtc_count_ : max;
        memcpy(out, dtcs_, n * sizeof(DtcRecord));
        xSemaphoreGive(dtc_mtx_);
    }
    return n;
}

// --- Vehicle info -----------------------------------------------------------
void EventBus::setVehicleInfo(const VehicleInfo& info) {
    if (xSemaphoreTake(veh_mtx_, pdMS_TO_TICKS(50)) == pdTRUE) {
        vehicle_ = info;
        xSemaphoreGive(veh_mtx_);
    }
}

VehicleInfo EventBus::getVehicleInfo() {
    VehicleInfo copy{};
    if (xSemaphoreTake(veh_mtx_, pdMS_TO_TICKS(50)) == pdTRUE) {
        copy = vehicle_;
        xSemaphoreGive(veh_mtx_);
    }
    return copy;
}

// --- I/M readiness ----------------------------------------------------------
void EventBus::setReadiness(const ReadinessInfo& info) {
    if (xSemaphoreTake(veh_mtx_, pdMS_TO_TICKS(50)) == pdTRUE) {
        readiness_ = info;
        xSemaphoreGive(veh_mtx_);
    }
}

ReadinessInfo EventBus::getReadiness() {
    ReadinessInfo copy{};
    if (xSemaphoreTake(veh_mtx_, pdMS_TO_TICKS(50)) == pdTRUE) {
        copy = readiness_;
        xSemaphoreGive(veh_mtx_);
    }
    return copy;
}

// --- Freeze frame -----------------------------------------------------------
void EventBus::setFreezeFrame(const FreezeFrame& ff) {
    if (xSemaphoreTake(veh_mtx_, pdMS_TO_TICKS(50)) == pdTRUE) {
        freeze_ = ff;
        xSemaphoreGive(veh_mtx_);
    }
}

FreezeFrame EventBus::getFreezeFrame() {
    FreezeFrame copy{};
    if (xSemaphoreTake(veh_mtx_, pdMS_TO_TICKS(50)) == pdTRUE) {
        copy = freeze_;
        xSemaphoreGive(veh_mtx_);
    }
    return copy;
}

// --- Module scan results ----------------------------------------------------
void EventBus::setModuleResults(const ModuleResult* mods, size_t count) {
    if (count > MAX_MODULES) count = MAX_MODULES;
    if (xSemaphoreTake(veh_mtx_, pdMS_TO_TICKS(50)) == pdTRUE) {
        memcpy(modules_, mods, count * sizeof(ModuleResult));
        module_count_ = count;
        xSemaphoreGive(veh_mtx_);
    }
}

size_t EventBus::getModuleResults(ModuleResult* out, size_t max) {
    size_t n = 0;
    if (xSemaphoreTake(veh_mtx_, pdMS_TO_TICKS(50)) == pdTRUE) {
        n = (module_count_ < max) ? module_count_ : max;
        memcpy(out, modules_, n * sizeof(ModuleResult));
        xSemaphoreGive(veh_mtx_);
    }
    return n;
}

// --- Supported PIDs ---------------------------------------------------------
void EventBus::setSupportedPids(const SupportedPids& sp) {
    if (xSemaphoreTake(veh_mtx_, pdMS_TO_TICKS(50)) == pdTRUE) {
        supported_ = sp;
        xSemaphoreGive(veh_mtx_);
    }
}

SupportedPids EventBus::getSupportedPids() {
    SupportedPids copy{};
    if (xSemaphoreTake(veh_mtx_, pdMS_TO_TICKS(50)) == pdTRUE) {
        copy = supported_;
        xSemaphoreGive(veh_mtx_);
    }
    return copy;
}
