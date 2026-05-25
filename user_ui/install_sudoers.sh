#!/usr/bin/env bash
# Install a NOPASSWD sudoers rule so the OpenArm Launcher GUI can run
# `ip link set canN ...` without prompting for a password. Run once with sudo.

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "This script must be run as root. Re-run with:" >&2
  echo "  sudo $0" >&2
  exit 1
fi

DST=/etc/sudoers.d/openarm-ui
USER_NAME="${SUDO_USER:-$(logname)}"

if [[ -z "$USER_NAME" || "$USER_NAME" == "root" ]]; then
  echo "Could not determine the invoking user (got '$USER_NAME')." >&2
  echo "Run via 'sudo' as a regular user, not as root directly." >&2
  exit 1
fi

TMP="$(mktemp)"
trap 'rm -f "$TMP"' EXIT

cat > "$TMP" <<EOF
# Allow $USER_NAME to bring CAN interfaces up via the OpenArm Launcher GUI.
# Installed by user_ui/install_sudoers.sh
$USER_NAME ALL=(root) NOPASSWD: /usr/sbin/ip link set can[0-9] *
$USER_NAME ALL=(root) NOPASSWD: /usr/sbin/ip link set can[1-9][0-9] *
EOF

visudo -cf "$TMP" >/dev/null
install -m 0440 -o root -g root "$TMP" "$DST"

echo "Installed $DST for user '$USER_NAME'."
echo "Quick check (should not prompt for a password):"
echo "  sudo -n /usr/sbin/ip link show can0"
