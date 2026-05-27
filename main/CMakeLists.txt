# =============================================================================
#  `main` component - application entry point and all first-party sources.
# =============================================================================
idf_component_register(
    SRCS
        "main.cpp"

        # --- core: shared state, ring buffers, event plumbing ---------------
        "core/ring_buffer.cpp"
        "core/event_bus.cpp"
        "core/tab5_power.cpp"

        # --- usb host transport ---------------------------------------------
        "usb/usb_host_cdc.cpp"
        "usb/obd_link.cpp"

        # --- obd protocol layer ---------------------------------------------
        "obd/stn_parser.cpp"
        "obd/obd_task.cpp"
        "obd/custom_pids.cpp"
        "obd/dtc_lookup.cpp"
        "obd/vin_decode.cpp"

        # --- gps / gnss -----------------------------------------------------
        "gps/gps.cpp"

        # --- sensors (imu) --------------------------------------------------
        "sensors/imu.cpp"

        # --- sd card logging ------------------------------------------------
        "logging/sd_logger.cpp"

        # --- audio alerts ---------------------------------------------------
        "audio/alerts.cpp"

        # --- lvgl ui --------------------------------------------------------
        "ui/ui.cpp"
        "ui/ui_theme.cpp"
        "ui/ui_task.cpp"
        "ui/screens/screen_home.cpp"
        "ui/screens/screen_dash.cpp"
        "ui/screens/screen_performance.cpp"
        "ui/screens/screen_diagnostics.cpp"
        "ui/screens/screen_readiness.cpp"
        "ui/screens/screen_vehicle.cpp"
        "ui/screens/screen_livedata.cpp"
        "ui/screens/screen_modules.cpp"
        "ui/screens/screen_logging.cpp"
        "ui/screens/screen_sniffer.cpp"
        "ui/screens/screen_graph.cpp"
        "ui/screens/screen_gps.cpp"
        "ui/screens/screen_tree.cpp"
        "ui/screens/screen_settings.cpp"

    INCLUDE_DIRS
        "."

    REQUIRES
        # Public deps surfaced through headers.
        lvgl
        esp_lvgl_port
        usb                    # usb_host.h (host library)
        usb_host_vcp           # esp_usb::VCP
        usb_host_ftdi_vcp      # FT23x line driver (OBDLink EX)
        usb_host_cp210x_vcp    # clone fallback
        usb_host_ch34x_vcp     # clone fallback
        m5stack_tab5         # Tab5 BSP (display/touch/SD/audio)
        esp_codec_dev          # speaker/codec for audible alerts
        sensor_hub             # BMI270 IMU via iot_sensor_hub
        esp_event              # sensor-hub data-ready events
        fatfs
        nvs_flash
        driver
        esp_timer
)
