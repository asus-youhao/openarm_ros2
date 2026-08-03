#!/usr/bin/env bash
# =============================================================================
#  install_can_udev.sh -- USB CAN udev rule installer / helper for x86 laptop
#
#  Naming aligned with NVIDIA Thor (192.168.1.139):
#    can0 <- gs_usb SN 0026003E5641571920373632  (Right O6 Hand)
#    can1 <- gs_usb SN 0035002A5742570C20353230  (Left  O6 Hand)
#    can2 <- PCAN-USB FD                          (Right Arm)
#    can3 <- PCAN-USB FD                          (Left  Arm)
#
#  Usage:
#    ./install_can_udev.sh discover         # scan currently-plugged gs_usb / pcan
#    ./install_can_udev.sh install [up]     # discover + generate + install rules
#                                           #   add 'up' to also bring up interfaces
#    ./install_can_udev.sh uninstall        # remove installed rules
#    ./install_can_udev.sh status           # show rule + current CAN interfaces
#    ./install_can_udev.sh up   [BR=1000000] [FD_BR=5000000]  # bring up can0..can3
#    ./install_can_udev.sh down             # bring down all
#    ./install_can_udev.sh enable-boot      # install + enable can-setup.service
#    ./install_can_udev.sh disable-boot     # disable + remove can-setup.service
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_PATH="$SCRIPT_DIR/$(basename "${BASH_SOURCE[0]}")"
RULE_SRC="$SCRIPT_DIR/80-usb-can.rules"
RULE_DST="/etc/udev/rules.d/80-usb-can.rules"
SERVICE_SRC="$SCRIPT_DIR/can-setup.service"
SERVICE_DST="/etc/systemd/system/can-setup.service"
TEMPLATE_SRC="$SCRIPT_DIR/can-up@.service"
TEMPLATE_DST="/etc/systemd/system/can-up@.service"

# Known gs_usb serials (from Thor)
GS_SN_CAN0="0026003E5641571920373632"
GS_SN_CAN1="0035002A5742570C20353230"

# run as root: use sudo only if we are not already root (so it works inside
# systemd services that already run as root, where sudo may fail w/o TTY)
SUDO() { if [[ $EUID -eq 0 ]]; then "$@"; else sudo "$@"; fi; }

color()  { printf '\033[%sm%s\033[0m' "$1" "$2"; }
ok()     { color "1;32" "[OK]   $*"; echo; }
warn()   { color "1;33" "[WARN] $*"; echo; }
err()    { color "1;31" "[ERR]  $*"; echo; }
info()   { color "1;36" "[INFO] $*"; echo; }

# ----- scanners -----
scan_gs_usb() {
  GS_SNS=()
  for dev in /sys/bus/usb/devices/*; do
    [ -f "$dev/idVendor" ] || continue
    [[ "$(cat "$dev/idVendor" 2>/dev/null)" == "1d50" ]] || continue
    [[ "$(cat "$dev/idProduct" 2>/dev/null)" == "606f" ]] || continue
    sn=$(cat "$dev/serial" 2>/dev/null || echo "")
    [[ -n "$sn" ]] && GS_SNS+=("$sn")
  done
}

scan_pcan() {
  PCAN_PORTS=()
  for dev in /sys/bus/usb/devices/*; do
    [ -f "$dev/idVendor" ] || continue
    [[ "$(cat "$dev/idVendor" 2>/dev/null)" == "0c72" ]] || continue
    [[ "$(cat "$dev/idProduct" 2>/dev/null)" == "0012" ]] || continue
    PCAN_PORTS+=("$(basename "$dev")")
  done
  if [[ ${#PCAN_PORTS[@]} -gt 1 ]]; then
    IFS=$'\n' PCAN_PORTS=($(printf '%s\n' "${PCAN_PORTS[@]}" | sort)); unset IFS
  fi
}

# ----- discover -----
cmd_discover() {
  echo
  info "lsusb entries for CAN devices:"
  lsusb | grep -iE "1d50:606f|0c72:0012|peak|geschwister" \
    || warn "no gs_usb (1d50:606f) or PCAN (0c72:0012) found"
  echo

  info "gs_usb devices (by USB serial):"
  scan_gs_usb
  if [[ ${#GS_SNS[@]} -eq 0 ]]; then
    warn "no gs_usb plugged in right now"
  else
    for sn in "${GS_SNS[@]}"; do
      tag=""
      [[ "$sn" == "$GS_SN_CAN0" ]] && tag="  $(color '1;32' '-> known=can0')"
      [[ "$sn" == "$GS_SN_CAN1" ]] && tag="  $(color '1;32' '-> known=can1')"
      echo "   serial=$sn$tag"
    done
  fi
  echo

  info "PCAN-USB FD devices (by USB port path = udev KERNELS match):"
  scan_pcan
  if [[ ${#PCAN_PORTS[@]} -eq 0 ]]; then
    warn "no PCAN plugged in right now"
  else
    for p in "${PCAN_PORTS[@]}"; do
      echo "   port=$p     udev KERNELS=\"${p}*\""
    done
  fi
  echo

  info "current net interfaces (CAN-like):"
  ip -br link show 2>/dev/null | grep -E "^can|gs|pcan" || echo "   (none)"
  echo
}

# ----- install (auto-generate after discover) -----
cmd_install() {
  info "Step 1/3 -- discover plugged devices"
  echo
  cmd_discover
  echo

  scan_gs_usb
  scan_pcan

  info "Step 2/3 -- generate rules file"

  declare -A GS_PRESENT=()
  for sn in "${GS_SNS[@]}"; do GS_PRESENT[$sn]=1; done

  GS_BLOCK_CAN0=""
  if [[ -n "${GS_PRESENT[$GS_SN_CAN0]:-}" ]]; then
    ok "gs_usb can0 -> SN $GS_SN_CAN0 (plugged in)"
  else
    warn "gs_usb can0 SN not plugged in -- rule still written for future plug-in"
  fi
  GS_BLOCK_CAN0="SUBSYSTEM==\"net\", ACTION==\"add|move\", DRIVERS==\"gs_usb\", ATTRS{serial}==\"$GS_SN_CAN0\", NAME=\"can0\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}=\"can-up@can0.service\""

  GS_BLOCK_CAN1=""
  if [[ -n "${GS_PRESENT[$GS_SN_CAN1]:-}" ]]; then
    ok "gs_usb can1 -> SN $GS_SN_CAN1 (plugged in)"
  else
    warn "gs_usb can1 SN not plugged in -- rule still written for future plug-in"
  fi
  GS_BLOCK_CAN1="SUBSYSTEM==\"net\", ACTION==\"add|move\", DRIVERS==\"gs_usb\", ATTRS{serial}==\"$GS_SN_CAN1\", NAME=\"can1\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}=\"can-up@can1.service\""

  case "${#PCAN_PORTS[@]}" in
    0)
      warn "no PCAN detected -- can2/can3 rules will be commented out"
      PCAN_BLOCK_CAN2="# (can2) no PCAN detected at install time. Plug PCAN and re-run install."
      PCAN_BLOCK_CAN3="# (can3) no PCAN detected at install time."
      ;;
    1)
      warn "only 1 PCAN detected (port=${PCAN_PORTS[0]}) -- assigned as can2"
      PCAN_BLOCK_CAN2="SUBSYSTEM==\"net\", ACTION==\"add|move\", DRIVERS==\"pcan\", KERNELS==\"${PCAN_PORTS[0]}*\", NAME=\"can2\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}=\"can-up@can2.service\""
      PCAN_BLOCK_CAN3="# (can3) only one PCAN plugged in, skipped."
      ;;
    2)
      ok "PCAN can2 -> port ${PCAN_PORTS[0]}"
      ok "PCAN can3 -> port ${PCAN_PORTS[1]}"
      PCAN_BLOCK_CAN2="SUBSYSTEM==\"net\", ACTION==\"add|move\", DRIVERS==\"pcan\", KERNELS==\"${PCAN_PORTS[0]}*\", NAME=\"can2\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}=\"can-up@can2.service\""
      PCAN_BLOCK_CAN3="SUBSYSTEM==\"net\", ACTION==\"add|move\", DRIVERS==\"pcan\", KERNELS==\"${PCAN_PORTS[1]}*\", NAME=\"can3\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}=\"can-up@can3.service\""
      ;;
    *)
      warn "detected ${#PCAN_PORTS[@]} PCAN devices (>2) -- using first two (sorted)"
      ok "PCAN can2 -> port ${PCAN_PORTS[0]}"
      ok "PCAN can3 -> port ${PCAN_PORTS[1]}"
      PCAN_BLOCK_CAN2="SUBSYSTEM==\"net\", ACTION==\"add|move\", DRIVERS==\"pcan\", KERNELS==\"${PCAN_PORTS[0]}*\", NAME=\"can2\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}=\"can-up@can2.service\""
      PCAN_BLOCK_CAN3="SUBSYSTEM==\"net\", ACTION==\"add|move\", DRIVERS==\"pcan\", KERNELS==\"${PCAN_PORTS[1]}*\", NAME=\"can3\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}=\"can-up@can3.service\""
      ;;
  esac

  TMP_RULE=$(mktemp)
  cat > "$TMP_RULE" <<EOF
# =============================================================================
#  /etc/udev/rules.d/80-usb-can.rules
#  Auto-generated by install_can_udev.sh on $(date '+%Y-%m-%d %H:%M:%S')
#  Host: $(hostname)
#  Naming aligned with NVIDIA Thor (192.168.1.139)
# =============================================================================

# --- gs_usb (pinned by USB serial) ---
$GS_BLOCK_CAN0
$GS_BLOCK_CAN1

# --- PCAN-USB FD (pinned by USB port path) ---
$PCAN_BLOCK_CAN2
$PCAN_BLOCK_CAN3
EOF

  echo
  info "generated rules content:"
  sed 's/^/   /' "$TMP_RULE"
  echo

  info "Step 3/3 -- install to $RULE_DST (sudo required)"
  SUDO install -m 0644 "$TMP_RULE" "$RULE_DST"
  rm -f "$TMP_RULE"
  ok "written: $RULE_DST"
  SUDO udevadm control --reload
  SUDO udevadm trigger --subsystem-match=net --action=add
  ok "udev reloaded + triggered"
  echo
  cmd_status_inner
}

cmd_uninstall() {
  if [[ -f "$RULE_DST" ]]; then
    SUDO rm -f "$RULE_DST"
    SUDO udevadm control --reload
    ok "removed $RULE_DST"
  else
    warn "$RULE_DST does not exist, nothing to do"
  fi
}

cmd_status_inner() {
  info "rule install status:"
  if [[ -f "$RULE_DST" ]]; then
    ok "$RULE_DST exists"
    grep -E "NAME=\"can[0-9]\"" "$RULE_DST" | sed 's/^/   /' || true
  else
    warn "not installed at $RULE_DST"
  fi
  echo
  info "current CAN interfaces:"
  ip -br link show type can 2>/dev/null || echo "   (none)"
}
cmd_status() { cmd_status_inner; }

cmd_up() {
  : "${BR:=1000000}"           # standard CAN 1Mbps (can0/1)
  : "${FD_BR:=5000000}"        # CAN-FD data bitrate (can2/3)
  for ifc in can0 can1; do
    if ip link show "$ifc" &>/dev/null; then
      SUDO ip link set "$ifc" down 2>/dev/null || true
      SUDO ip link set "$ifc" type can bitrate "$BR"
      SUDO ip link set "$ifc" up
      ok "$ifc up @ $BR bps"
    else
      warn "$ifc not present (device unplugged or udev rule not applied)"
    fi
  done
  for ifc in can2 can3; do
    if ip link show "$ifc" &>/dev/null; then
      SUDO ip link set "$ifc" down 2>/dev/null || true
      SUDO ip link set "$ifc" type can bitrate "$BR" dbitrate "$FD_BR" fd on
      SUDO ip link set "$ifc" up
      ok "$ifc up @ ${BR}/${FD_BR} bps (FD)"
    else
      warn "$ifc not present"
    fi
  done
  echo
  ip -br link show type can
}

# bring up a single interface (used by can-up@.service template)
cmd_up_one() {
  ifc="${1:-}"
  [[ -z "$ifc" ]] && { err "up-one needs interface name"; exit 1; }
  : "${BR:=1000000}"
  : "${FD_BR:=5000000}"
  if ! ip link show "$ifc" &>/dev/null; then
    err "$ifc does not exist"
    exit 1
  fi
  SUDO ip link set "$ifc" down 2>/dev/null || true
  case "$ifc" in
    can0|can1)
      SUDO ip link set "$ifc" type can bitrate "$BR"
      SUDO ip link set "$ifc" up
      ok "$ifc up @ $BR bps"
      ;;
    can2|can3)
      SUDO ip link set "$ifc" type can bitrate "$BR" dbitrate "$FD_BR" fd on
      SUDO ip link set "$ifc" up
      ok "$ifc up @ ${BR}/${FD_BR} bps (FD)"
      ;;
    *)
      SUDO ip link set "$ifc" type can bitrate "$BR"
      SUDO ip link set "$ifc" up
      ok "$ifc up @ $BR bps"
      ;;
  esac
}

cmd_down() {
  for ifc in can0 can1 can2 can3; do
    if ip link show "$ifc" &>/dev/null; then
      SUDO ip link set "$ifc" down
      ok "$ifc down"
    fi
  done
}

cmd_enable_boot() {
  # ---- 1) per-interface template service (triggered by udev on hotplug) ----
  TMP_TPL=$(mktemp)
  cat > "$TMP_TPL" <<EOF
# /etc/systemd/system/can-up@.service
# Auto-generated by install_can_udev.sh on $(date '+%Y-%m-%d %H:%M:%S')
# Triggered by udev (ENV{SYSTEMD_WANTS}="can-up@canX.service") on plug-in.
# Brings up a single CAN interface using install_can_udev.sh up-one %i
[Unit]
Description=Bring up CAN interface %i
BindsTo=sys-subsystem-net-devices-%i.device
After=sys-subsystem-net-devices-%i.device

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=$SCRIPT_PATH up-one %i
ExecStop=/sbin/ip link set %i down

[Install]
WantedBy=multi-user.target
EOF
  SUDO install -m 0644 "$TMP_TPL" "$TEMPLATE_DST"
  cp "$TMP_TPL" "$TEMPLATE_SRC"
  rm -f "$TMP_TPL"
  ok "installed: $TEMPLATE_DST (triggers on USB hotplug via udev)"

  # ---- 2) one-shot boot service (covers devices already plugged at boot) ----
  TMP_UNIT=$(mktemp)
  cat > "$TMP_UNIT" <<EOF
# /etc/systemd/system/can-setup.service
# Auto-generated by install_can_udev.sh on $(date '+%Y-%m-%d %H:%M:%S')
# Brings up can0..can3 at boot, using install_can_udev.sh up
[Unit]
Description=Bring up USB CAN interfaces (can0..can3) at boot
After=systemd-udev-settle.service network-pre.target
Wants=systemd-udev-settle.service

[Service]
Type=oneshot
RemainAfterExit=yes
# Wait briefly so udev finishes renaming the interfaces
ExecStartPre=/bin/sh -c 'for i in 1 2 3 4 5 6 7 8 9 10; do ip link show can0 >/dev/null 2>&1 && exit 0; sleep 1; done; exit 0'
ExecStart=$SCRIPT_PATH up
ExecStop=$SCRIPT_PATH down

[Install]
WantedBy=multi-user.target
EOF
  SUDO install -m 0644 "$TMP_UNIT" "$SERVICE_DST"
  cp "$TMP_UNIT" "$SERVICE_SRC"
  rm -f "$TMP_UNIT"
  ok "installed: $SERVICE_DST (boot-time bring up)"

  SUDO systemctl daemon-reload
  SUDO systemctl enable can-setup.service
  ok "enabled: can-setup.service"
  echo
  info "How it works now:"
  echo "   - At boot: can-setup.service brings up devices already plugged in"
  echo "   - On hotplug: udev rule (TAG+=systemd) triggers can-up@canX.service"
  echo "     => unplug + replug a USB CAN device will auto re-bring-up that interface"
  echo
  info "Note: udev rules were tagged during 'install'. If you ran 'install'"
  echo "      BEFORE this version of the script, re-run:"
  echo "         ./install_can_udev.sh install"
  echo "      to refresh the rules with SYSTEMD_WANTS tags."
  echo
  info "Useful commands:"
  echo "   SUDO systemctl start can-setup.service       # start boot bring-up now"
  echo "   systemctl status 'can-up@can0.service'       # check a specific iface"
  echo "   journalctl -u 'can-up@*' -f                  # follow hotplug events"
}

cmd_disable_boot() {
  if [[ -f "$SERVICE_DST" ]]; then
    SUDO systemctl disable --now can-setup.service 2>/dev/null || true
    SUDO rm -f "$SERVICE_DST"
    ok "removed $SERVICE_DST"
  else
    warn "$SERVICE_DST does not exist"
  fi
  if [[ -f "$TEMPLATE_DST" ]]; then
    for ifc in can0 can1 can2 can3; do
      SUDO systemctl stop "can-up@${ifc}.service" 2>/dev/null || true
    done
    SUDO rm -f "$TEMPLATE_DST"
    ok "removed $TEMPLATE_DST"
  fi
  SUDO systemctl daemon-reload
}

# allow chaining: 'install up' = install then bring up
case "${1:-help}" in
  discover)     cmd_discover ;;
  install)
    cmd_install
    if [[ "${2:-}" == "up" ]]; then
      echo
      info "chained: bringing up interfaces"
      cmd_up
    fi
    ;;
  uninstall)    cmd_uninstall ;;
  status)       cmd_status ;;
  up)           cmd_up ;;
  up-one)       cmd_up_one "${2:-}" ;;
  down)         cmd_down ;;
  enable-boot)  cmd_enable_boot ;;
  disable-boot) cmd_disable_boot ;;
  *)
    sed -n '2,24p' "$0"
    ;;
esac
