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
#  Two-phase naming (avoids the shared-canN rename race):
#    Phase 1 (udev): each device is renamed on plug-in to a COLLISION-FREE
#      interim name OUTSIDE the kernel's can%d pool, so the rename can never
#      clash with another not-yet-renamed CAN device:
#         o6r  <- gs_usb SN ...3632      armr <- PCAN port #1
#         o6l  <- gs_usb SN ...3230      arml <- PCAN port #2
#      udev only pulls in a single can-settle.service (not per-interface).
#    Phase 2 (can-settle.service): one idempotent, fixed-order sweep renames
#      o6r->can0, o6l->can1, armr->can2, arml->can3 (down -> rename -> cfg ->
#      up). Because no device is ever auto-named canN, the canN targets are
#      always free; plug order no longer matters.
#
#  Usage:
#    ./install_can_udev.sh discover         # scan currently-plugged gs_usb / pcan
#    ./install_can_udev.sh install [up]     # discover + generate + install rules
#                                           #   add 'up' to also settle+bring up now
#    ./install_can_udev.sh uninstall        # remove installed rules
#    ./install_can_udev.sh status           # show rule + current CAN interfaces
#    ./install_can_udev.sh settle [BR=..] [FD_BR=..]  # rename interim->canN + up
#    ./install_can_udev.sh up   [BR=1000000] [FD_BR=5000000]  # bring up can0..can3
#    ./install_can_udev.sh down             # bring down all
#    ./install_can_udev.sh enable-boot      # install + enable can-settle.service
#    ./install_can_udev.sh disable-boot     # disable + remove can-settle.service
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_PATH="$SCRIPT_DIR/$(basename "${BASH_SOURCE[0]}")"
RULE_SRC="$SCRIPT_DIR/80-usb-can.rules"
RULE_DST="/etc/udev/rules.d/80-usb-can.rules"
SETTLE_SRC="$SCRIPT_DIR/can-settle.service"
SETTLE_DST="/etc/systemd/system/can-settle.service"
# Obsolete units from the previous per-interface scheme (cleaned up on
# install / enable-boot / disable-boot so stale rules can't re-trigger).
OLD_SERVICE_DST="/etc/systemd/system/can-setup.service"
OLD_TEMPLATE_DST="/etc/systemd/system/can-up@.service"

# Known gs_usb serials (from Thor)
GS_SN_CAN0="0026003E5641571920373632"
GS_SN_CAN1="0035002A5742570C20353230"

# Two-phase naming: interim (collision-free) name -> final canN + kind.
# Interim names are deliberately NOT canN, since gs_usb and pcan share the
# kernel's can%d pool and renaming straight into canN races on plug-in.
SETTLE_ORDER=(o6r o6l armr arml)
declare -A FINAL_OF=( [o6r]=can0 [o6l]=can1 [armr]=can2 [arml]=can3 )
declare -A KIND_OF=(  [o6r]=std  [o6l]=std  [armr]=fd   [arml]=fd )

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

# Write + load can-settle.service (the single Phase-2 unit). Used by both
# `install` (so hotplug SYSTEMD_WANTS works) and `enable-boot` (which also
# enables it for boot). Idempotent.
write_settle_unit() {
  local tmp
  tmp=$(mktemp)
  cat > "$tmp" <<EOF
# /etc/systemd/system/can-settle.service
# Auto-generated by install_can_udev.sh on $(date '+%Y-%m-%d %H:%M:%S')
# Phase 2 of two-phase CAN naming: rename interim names (o6r/o6l/armr/arml)
# to can0..can3 and bring them up, in one idempotent fixed-order sweep.
# Triggered on hotplug by udev (ENV{SYSTEMD_WANTS}) and, when enabled, at
# boot (WantedBy=multi-user.target). RemainAfterExit=no so every hotplug
# event re-runs the sweep.
[Unit]
Description=Rename + bring up USB CAN interfaces (can0..can3)
After=systemd-udev-settle.service
Wants=systemd-udev-settle.service

[Service]
Type=oneshot
RemainAfterExit=no
ExecStart=$SCRIPT_PATH settle

[Install]
WantedBy=multi-user.target
EOF
  SUDO install -m 0644 "$tmp" "$SETTLE_DST"
  cp "$tmp" "$SETTLE_SRC" 2>/dev/null || true
  rm -f "$tmp"
  # Drop obsolete per-interface units from the old scheme.
  [[ -f "$OLD_TEMPLATE_DST" ]] && SUDO rm -f "$OLD_TEMPLATE_DST"
  SUDO systemctl daemon-reload
  ok "installed: $SETTLE_DST"
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

  # Phase-1 rule template: pin device -> interim name, no per-iface bring-up;
  # pull in the single can-settle.service which does the rename + up sweep.
  WANTS="can-settle.service"

  GS_BLOCK_CAN0=""
  if [[ -n "${GS_PRESENT[$GS_SN_CAN0]:-}" ]]; then
    ok "gs_usb o6r (->can0) -> SN $GS_SN_CAN0 (plugged in)"
  else
    warn "gs_usb o6r SN not plugged in -- rule still written for future plug-in"
  fi
  GS_BLOCK_CAN0="SUBSYSTEM==\"net\", ACTION==\"add|move\", DRIVERS==\"gs_usb\", ATTRS{serial}==\"$GS_SN_CAN0\", NAME=\"o6r\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}=\"$WANTS\""

  GS_BLOCK_CAN1=""
  if [[ -n "${GS_PRESENT[$GS_SN_CAN1]:-}" ]]; then
    ok "gs_usb o6l (->can1) -> SN $GS_SN_CAN1 (plugged in)"
  else
    warn "gs_usb o6l SN not plugged in -- rule still written for future plug-in"
  fi
  GS_BLOCK_CAN1="SUBSYSTEM==\"net\", ACTION==\"add|move\", DRIVERS==\"gs_usb\", ATTRS{serial}==\"$GS_SN_CAN1\", NAME=\"o6l\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}=\"$WANTS\""

  case "${#PCAN_PORTS[@]}" in
    0)
      warn "no PCAN detected -- can2/can3 rules will be commented out"
      PCAN_BLOCK_CAN2="# (armr/can2) no PCAN detected at install time. Plug PCAN and re-run install."
      PCAN_BLOCK_CAN3="# (arml/can3) no PCAN detected at install time."
      ;;
    1)
      warn "only 1 PCAN detected (port=${PCAN_PORTS[0]}) -- assigned as armr (->can2)"
      PCAN_BLOCK_CAN2="SUBSYSTEM==\"net\", ACTION==\"add|move\", DRIVERS==\"pcan\", KERNELS==\"${PCAN_PORTS[0]}*\", NAME=\"armr\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}=\"$WANTS\""
      PCAN_BLOCK_CAN3="# (arml/can3) only one PCAN plugged in, skipped."
      ;;
    2)
      ok "PCAN armr (->can2) -> port ${PCAN_PORTS[0]}"
      ok "PCAN arml (->can3) -> port ${PCAN_PORTS[1]}"
      PCAN_BLOCK_CAN2="SUBSYSTEM==\"net\", ACTION==\"add|move\", DRIVERS==\"pcan\", KERNELS==\"${PCAN_PORTS[0]}*\", NAME=\"armr\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}=\"$WANTS\""
      PCAN_BLOCK_CAN3="SUBSYSTEM==\"net\", ACTION==\"add|move\", DRIVERS==\"pcan\", KERNELS==\"${PCAN_PORTS[1]}*\", NAME=\"arml\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}=\"$WANTS\""
      ;;
    *)
      warn "detected ${#PCAN_PORTS[@]} PCAN devices (>2) -- using first two (sorted)"
      ok "PCAN armr (->can2) -> port ${PCAN_PORTS[0]}"
      ok "PCAN arml (->can3) -> port ${PCAN_PORTS[1]}"
      PCAN_BLOCK_CAN2="SUBSYSTEM==\"net\", ACTION==\"add|move\", DRIVERS==\"pcan\", KERNELS==\"${PCAN_PORTS[0]}*\", NAME=\"armr\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}=\"$WANTS\""
      PCAN_BLOCK_CAN3="SUBSYSTEM==\"net\", ACTION==\"add|move\", DRIVERS==\"pcan\", KERNELS==\"${PCAN_PORTS[1]}*\", NAME=\"arml\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}=\"$WANTS\""
      ;;
  esac

  TMP_RULE=$(mktemp)
  cat > "$TMP_RULE" <<EOF
# =============================================================================
#  /etc/udev/rules.d/80-usb-can.rules
#  Auto-generated by install_can_udev.sh on $(date '+%Y-%m-%d %H:%M:%S')
#  Host: $(hostname)
#  Naming aligned with NVIDIA Thor (192.168.1.139)
#
#  Phase 1: pin each device to a collision-free interim name (o6r/o6l/
#  armr/arml) and pull in can-settle.service, which renames them to
#  can0..can3 and brings them up (see install_can_udev.sh settle).
# =============================================================================

# --- gs_usb (pinned by USB serial) -> interim o6r / o6l ---
$GS_BLOCK_CAN0
$GS_BLOCK_CAN1

# --- PCAN-USB FD (pinned by USB port path) -> interim armr / arml ---
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
  # Make sure the Phase-2 unit exists so udev's SYSTEMD_WANTS resolves on
  # hotplug (enable-boot additionally enables it for boot).
  write_settle_unit
  SUDO udevadm control --reload
  SUDO udevadm trigger --subsystem-match=net --action=add
  ok "udev reloaded + triggered"
  echo
  warn "NOTE: live, already-UP interfaces won't be renamed by the trigger"
  warn "      (rename needs the link DOWN). To apply the new names now, either"
  warn "      replug the USB CAN devices, reboot, or run: $0 settle"
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
    grep -E "NAME=\"(o6r|o6l|armr|arml|can[0-9])\"" "$RULE_DST" | sed 's/^/   /' || true
  else
    warn "not installed at $RULE_DST"
  fi
  echo
  info "can-settle.service:"
  if [[ -f "$SETTLE_DST" ]]; then
    ok "$SETTLE_DST present ($(systemctl is-enabled can-settle.service 2>/dev/null || echo 'not enabled'))"
  else
    warn "not installed (run: $0 enable-boot, or $0 install)"
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

# Phase 2: rename interim names (o6r/o6l/armr/arml) -> can0..can3 and bring
# them up. Idempotent and order-independent: each interface is handled on its
# own, the link is taken DOWN before rename, and because no device is ever
# auto-named canN the rename target is always free -> no plug-order race.
cmd_settle() {
  : "${BR:=1000000}"           # standard CAN 1Mbps  (can0/1)
  : "${FD_BR:=5000000}"        # CAN-FD data bitrate (can2/3)
  for interim in "${SETTLE_ORDER[@]}"; do
    final="${FINAL_OF[$interim]}"
    kind="${KIND_OF[$interim]}"
    if ip link show "$interim" &>/dev/null; then
      # Fresh device sitting under its interim name -> rename into place.
      SUDO ip link set "$interim" down 2>/dev/null || true
      if ! SUDO ip link set "$interim" name "$final" 2>/dev/null; then
        err "rename $interim -> $final failed (is $final already taken?)"
        continue
      fi
      ok "renamed $interim -> $final"
    elif ip link show "$final" &>/dev/null; then
      # Already named (previous sweep / re-run) -> just reconfigure + up.
      SUDO ip link set "$final" down 2>/dev/null || true
    else
      warn "$interim/$final not present"
      continue
    fi
    if [[ "$kind" == "fd" ]]; then
      SUDO ip link set "$final" type can bitrate "$BR" dbitrate "$FD_BR" fd on
      SUDO ip link set "$final" up
      ok "$final up @ ${BR}/${FD_BR} bps (FD)"
    else
      SUDO ip link set "$final" type can bitrate "$BR"
      SUDO ip link set "$final" up
      ok "$final up @ $BR bps"
    fi
  done
  echo
  ip -br link show type can
}

cmd_enable_boot() {
  # Single Phase-2 unit, shared by boot and hotplug. Both the boot path
  # (WantedBy=multi-user.target) and the udev path (ENV{SYSTEMD_WANTS}) run
  # the same idempotent `settle` sweep, so there's no per-interface race.
  write_settle_unit
  SUDO systemctl enable can-settle.service
  ok "enabled: can-settle.service"
  # Drop the obsolete units from the previous per-interface scheme.
  if [[ -f "$OLD_SERVICE_DST" ]]; then
    SUDO systemctl disable --now can-setup.service 2>/dev/null || true
    SUDO rm -f "$OLD_SERVICE_DST"
    SUDO systemctl daemon-reload
    ok "removed obsolete $OLD_SERVICE_DST"
  fi
  echo
  info "How it works now:"
  echo "   - At boot: can-settle.service renames interim->canN and brings up"
  echo "   - On hotplug: udev (ENV{SYSTEMD_WANTS}=can-settle.service) re-runs"
  echo "     the same sweep => replug auto re-applies naming + bring-up"
  echo
  info "Useful commands:"
  echo "   sudo systemctl start can-settle.service     # run the sweep now"
  echo "   systemctl status can-settle.service         # check last run"
  echo "   journalctl -u can-settle.service -f         # follow events"
}

cmd_disable_boot() {
  local removed=0
  if [[ -f "$SETTLE_DST" ]]; then
    SUDO systemctl disable --now can-settle.service 2>/dev/null || true
    SUDO rm -f "$SETTLE_DST"
    ok "removed $SETTLE_DST"
    removed=1
  fi
  # Also clean up obsolete units from the previous per-interface scheme.
  if [[ -f "$OLD_SERVICE_DST" ]]; then
    SUDO systemctl disable --now can-setup.service 2>/dev/null || true
    SUDO rm -f "$OLD_SERVICE_DST"
    ok "removed obsolete $OLD_SERVICE_DST"
    removed=1
  fi
  if [[ -f "$OLD_TEMPLATE_DST" ]]; then
    for ifc in can0 can1 can2 can3; do
      SUDO systemctl stop "can-up@${ifc}.service" 2>/dev/null || true
    done
    SUDO rm -f "$OLD_TEMPLATE_DST"
    ok "removed obsolete $OLD_TEMPLATE_DST"
    removed=1
  fi
  [[ "$removed" -eq 0 ]] && warn "nothing to remove"
  SUDO systemctl daemon-reload
}

# allow chaining: 'install up' = install then settle (rename + bring up)
case "${1:-help}" in
  discover)     cmd_discover ;;
  install)
    cmd_install
    if [[ "${2:-}" == "up" ]]; then
      echo
      info "chained: settling (rename interim -> canN + bring up)"
      cmd_settle
    fi
    ;;
  uninstall)    cmd_uninstall ;;
  status)       cmd_status ;;
  settle)       cmd_settle ;;
  up)           cmd_up ;;
  up-one)       cmd_up_one "${2:-}" ;;
  down)         cmd_down ;;
  enable-boot)  cmd_enable_boot ;;
  disable-boot) cmd_disable_boot ;;
  *)
    sed -n '2,40p' "$0"
    ;;
esac
