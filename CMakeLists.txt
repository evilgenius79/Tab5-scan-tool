# =============================================================================
#  Tab5 Scan Tool - top-level ESP-IDF project CMake
# -----------------------------------------------------------------------------
#  Build natively with:
#     idf.py set-target esp32p4
#     idf.py build flash monitor
# =============================================================================
cmake_minimum_required(VERSION 3.16)

# Pull in the ESP-IDF build system.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(tab5_scan_tool)
