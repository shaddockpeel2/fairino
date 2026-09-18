#!/usr/bin/env bash
# Recover the Orbbec DaBai DCW2 depth camera after a USB wedge.
#
# The camera (depth 2bc5:06a0 + RGB 2bc5:0561) hangs off a chain of bus-powered
# hubs. When one of those hubs stops answering, both functions stay visible in
# sysfs but every control transfer times out:
#
#   hub 6-1.3:1.0: hub_ext_port_status failed (err = -110)
#   usb 6-1.3-port3: cannot reset (err = -110)
#   -> orbbec_camera: "Device with serial number CH82C5201C9 not found"
#
# A reset issued against the *device* cannot work, because the port reset is
# performed by the very hub that is hung. The fix is to reset the hung hub
# itself, which the (healthy) upstream hub performs. That needs root.
#
# Usage:
#   sudo scripts/camera/recover_depth_camera.sh [--serial CH82C5201C9] [--dry-run]
#
# Exit codes: 0 recovered, 1 still wedged, 2 usage/environment error.

set -uo pipefail

SYS_USB=/sys/bus/usb/devices
SERIAL=CH82C5201C9
DRY_RUN=0
ORBBEC_VID=2bc5
DCW2_PIDS=(06a0 0561)

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROBE="$SCRIPT_DIR/libusb_probe.py"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --serial) SERIAL="$2"; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

log()  { printf '[recover] %s\n' "$*"; }
warn() { printf '[recover] WARNING: %s\n' "$*" >&2; }

read_attr() { cat "$SYS_USB/$1/$2" 2>/dev/null | tr -d '\n'; }

# --- locate the camera ------------------------------------------------------
find_camera_devices() {
  local name
  for name in $(ls "$SYS_USB" 2>/dev/null); do
    [[ -f "$SYS_USB/$name/idVendor" ]] || continue
    [[ "$(read_attr "$name" idVendor)" == "$ORBBEC_VID" ]] || continue
    [[ "$(read_attr "$name" serial)" == "$SERIAL" ]] || continue
    echo "$name"
  done
}

# Walk up from a device to the nearest ancestor that is a hub.
find_parent_hub() {
  local path="$1" parent
  while [[ "$path" == *.* ]]; do
    parent="${path%.*}"
    if [[ -f "$SYS_USB/$parent/bDeviceClass" ]] &&
       [[ "$(read_attr "$parent" bDeviceClass)" == "09" ]]; then
      echo "$parent"
      return 0
    fi
    path="$parent"
  done
  # No external hub in the path: the root hub itself hosts the device.
  [[ "$path" == usb* ]] && echo "$path"
}

# --- verification -----------------------------------------------------------
device_answers() {
  [[ -x "$PROBE" ]] || return 0   # no probe available: do not block recovery
  python3 "$PROBE" >/dev/null 2>&1
}

# --- reset strategies, least invasive first ---------------------------------
reset_via_authorized() {
  local hub="$1" attr="$SYS_USB/$1/authorized"
  [[ -w "$attr" ]] || return 1
  log "deauthorizing $hub (removes the whole subtree)"
  echo 0 > "$attr" || return 1
  sleep 2
  log "reauthorizing $hub (forces re-enumeration)"
  echo 1 > "$attr" || return 1
  sleep 4
}

reset_via_usbreset() {
  local hub="$1" bus dev
  bus="$(read_attr "$hub" busnum)"; dev="$(read_attr "$hub" devnum)"
  command -v usbreset >/dev/null || return 1
  [[ -n "$bus" && -n "$dev" ]] || return 1
  log "USBDEVFS_RESET on hub $hub (bus $bus dev $dev)"
  usbreset "$(printf '%03d/%03d' "$bus" "$dev")"
}

reset_via_unbind_rebind() {
  local hub="$1" iface="$1:1.0"
  [[ -w /sys/bus/usb/drivers/hub/unbind ]] || return 1
  log "unbinding $iface from the hub driver"
  echo "$iface" > /sys/bus/usb/drivers/hub/unbind || return 1
  sleep 2
  log "rebinding $iface"
  echo "$iface" > /sys/bus/usb/drivers/hub/bind || return 1
  sleep 4
}

main() {
  if [[ "${EUID:-$(id -u)}" -ne 0 && $DRY_RUN -eq 0 ]]; then
    warn "must run as root: resetting a hung hub is a privileged USB operation."
    warn "re-run with: sudo $0 $*"
    exit 2
  fi

  local devices
  devices="$(find_camera_devices)"
  if [[ -z "$devices" ]]; then
    warn "camera serial $SERIAL is not present on the USB bus at all."
    warn "Check the cable and port; a full re-enumeration needs a replug."
    exit 1
  fi
  log "found camera functions: $(echo "$devices" | tr '\n' ' ')"

  # All functions of one camera share a hub; take the outermost common ancestor.
  local hubs hub
  hubs="$(for d in $devices; do find_parent_hub "$d"; done | sort -u)"
  log "hosting hub(s): $(echo "$hubs" | tr '\n' ' ')"

  # Deepest hub first: it is the one that failed, and resetting it disturbs least.
  for hub in $(echo "$hubs" | awk -F. '{print NF-1, $0}' | sort -rn | cut -d' ' -f2-); do
    [[ -n "$hub" ]] || continue

    if device_answers && [[ $DRY_RUN -eq 0 ]]; then
      log "camera already answers control transfers; nothing to do"
      exit 0
    fi

    if [[ $DRY_RUN -eq 1 ]]; then
      log "would reset hub: $hub"
      continue
    fi

    log "--- resetting hub $hub ---"
    reset_via_authorized "$hub" && device_answers && { log "recovered via authorized toggle"; exit 0; }
    reset_via_usbreset "$hub" && sleep 3 && device_answers && { log "recovered via usbreset"; exit 0; }
    reset_via_unbind_rebind "$hub" && device_answers && { log "recovered via unbind/rebind"; exit 0; }
    warn "hub $hub did not recover"
  done

  [[ $DRY_RUN -eq 1 ]] && exit 0

  warn "camera still not answering control transfers."
  warn "The hub is wedged in hardware. Physical action is required:"
  warn "  1. unplug the camera (and the hub it is plugged into), wait 10 s, replug"
  warn "  2. plug the camera directly into a board USB port, not a cascaded hub"
  exit 1
}

main
