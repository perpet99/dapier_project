#!/usr/bin/env bash
set -euo pipefail

# Installs Orbbec's own OpenNI2 Linux runtime (the vendor SDK, not Ubuntu's
# stock libopenni2-0) so ros-jazzy-openni2-camera can actually see the
# Orbbec Astra Pro. Ubuntu's packaged PS1080 driver does not recognize
# Orbbec's OEM vendor ID; Orbbec's own liborbbec.so driver does.
#
# Source: https://www.orbbec.com/developers/openni-sdk/ (official Orbbec
# developer download page) -> dl.orbbec3d.com. This only copies the small
# runtime pieces we need (core lib + driver .so's, ~2.5MB) into
# ros2_slam/third_party/orbbec_openni2/ -- it does not vendor the full
# ~520MB SDK download (samples, docs, other architectures) into the repo.
#
# Usage: ./scripts/install_orbbec_openni2.sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
DEST="${WS_DIR}/third_party/orbbec_openni2"
URL="https://dl.orbbec3d.com/dist/openni2/v2.3.0.86-beta6/Orbbec_OpenNI_v2.3.0.86-beta6_linux_release.zip"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

echo "[1/4] Downloading Orbbec OpenNI2 SDK (~520MB, one-time)..."
curl -fL -o "${WORK}/sdk.zip" "$URL"

echo "[2/4] Extracting..."
unzip -q "${WORK}/sdk.zip" -d "${WORK}/outer"
unzip -q "${WORK}"/outer/*linux_x64.zip -d "${WORK}/x64"
SDK_ROOT="$(find "${WORK}/x64" -maxdepth 1 -mindepth 1 -type d | head -1)"

echo "[3/4] Installing runtime into ${DEST} ..."
rm -rf "$DEST"
mkdir -p "${DEST}/OpenNI2/Drivers"
cp "${SDK_ROOT}/sdk/libs/libOpenNI2.so" "${DEST}/libOpenNI2.so.0"
cp "${SDK_ROOT}"/sdk/libs/OpenNI2/Drivers/*.so "${DEST}/OpenNI2/Drivers/"

echo "[4/4] Installing Orbbec's official udev rule (needs sudo)..."
sudo cp "${SDK_ROOT}/rules/orbbec-usb.rules" /etc/udev/rules.d/558-orbbec-usb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb

echo ""
echo "Done. bringup.launch.py will automatically pick this up via the"
echo "'openni2_lib_override_dir' launch argument (defaults to this path)."
echo ""
echo "Quick standalone check (without ROS):"
echo "  LD_LIBRARY_PATH=${DEST} ros2 run openni2_camera list_devices"
