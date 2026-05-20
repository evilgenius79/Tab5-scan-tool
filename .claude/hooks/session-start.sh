#!/bin/bash
# =============================================================================
#  SessionStart hook - install the ESP-IDF toolchain for this project so web
#  sessions can run `idf.py set-target esp32p4` / `idf.py build`.
#
#  This project has no npm/pip-style manifests; its "dependencies" are ESP-IDF
#  itself plus the managed components declared in main/idf_component.yml (those
#  are fetched automatically by `idf.py reconfigure`).
#
#  Runs synchronously: the session waits until ESP-IDF is ready, avoiding a race
#  where Claude tries to build before the toolchain exists. Idempotent - a
#  cached container with ESP-IDF already present is detected and reused.
# =============================================================================
set -euo pipefail

# Only meaningful in the remote (web) container; skip locally.
if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
  exit 0
fi

IDF_BRANCH="release/v5.3"
IDF_DIR="$HOME/esp/esp-idf"
IDF_TARGET="esp32p4"

# Persist environment for the rest of the session (sourced by the shell).
persist_env() {
  if [ -n "${CLAUDE_ENV_FILE:-}" ]; then
    {
      echo "export IDF_PATH=\"$IDF_DIR\""
      # export.sh wires up PATH, IDF_PYTHON_ENV_PATH, etc.
      echo "source \"$IDF_DIR/export.sh\" >/dev/null 2>&1 || true"
    } >> "$CLAUDE_ENV_FILE"
  fi
}

# Already installed (cached container)? Just re-export and finish.
if [ -f "$IDF_DIR/export.sh" ] && [ -d "$HOME/.espressif" ]; then
  echo "ESP-IDF already present at $IDF_DIR"
  persist_env
  exit 0
fi

echo "Installing ESP-IDF ($IDF_BRANCH) for target $IDF_TARGET ..."

# Shallow clone keeps the download small; submodules are shallow too.
if [ ! -d "$IDF_DIR/.git" ]; then
  mkdir -p "$HOME/esp"
  git clone --depth 1 --branch "$IDF_BRANCH" --shallow-submodules \
      --recurse-submodules https://github.com/espressif/esp-idf.git "$IDF_DIR"
fi

# Install only the tools needed for the ESP32-P4 target.
"$IDF_DIR/install.sh" "$IDF_TARGET"

persist_env
echo "ESP-IDF install complete."
