#!/usr/bin/env bash
# Full test for openarm_description (and the openarm_bringup launches that load it).
#
# Usage:  <ws>/src/openarm_ros2/openarm_description/test/run_full_test.sh
#
# <ws> defaults to two levels above the openarm_ros2 checkout. The script
# re-executes itself in a clean environment (env -i + /opt/ros/<distro> only),
# so another workspace's openarm_description can never leak into the result.
#
# Env overrides:
#   OPENARM_TEST_WS     workspace root
#   OPENARM_ROS_DISTRO  humble / jazzy (default: the only one under /opt/ros)

set -uo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG=$(cd "$HERE/.." && pwd)
REPO=$(cd "$PKG/.." && pwd)
WS=${OPENARM_TEST_WS:-$(cd "$REPO/../.." && pwd)}

if [[ -z "${OPENARM_TEST_CLEAN_ENV:-}" ]]; then
  exec env -i HOME="$HOME" PATH=/usr/local/bin:/usr/bin:/bin TERM="${TERM:-dumb}" \
    LANG=C.UTF-8 OPENARM_TEST_CLEAN_ENV=1 OPENARM_TEST_WS="$WS" \
    OPENARM_ROS_DISTRO="${OPENARM_ROS_DISTRO:-}" \
    bash "$0" "$@"
fi

DISTRO=${OPENARM_ROS_DISTRO:-}
if [[ -z $DISTRO ]]; then
  mapfile -t distros < <(ls /opt/ros)
  if [[ ${#distros[@]} -ne 1 ]]; then
    echo "several ROS distros under /opt/ros (${distros[*]}), set OPENARM_ROS_DISTRO"; exit 2
  fi
  DISTRO=${distros[0]}
fi
# ROS setup files reference unset variables, so source them without -u.
set +u
# shellcheck disable=SC1090
source "/opt/ros/$DISTRO/setup.bash"
set -u

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

FAILED=()
step() { printf '\n=== [%s] %s\n' "$1" "$2"; }
fail() { echo "FAIL: $1"; FAILED+=("$1"); }

echo "repo     : $REPO ($(git -C "$REPO" rev-parse --short HEAD), $(git -C "$REPO" branch --show-current))"
echo "workspace: $WS"
echo "ROS      : $DISTRO"

# ---------------------------------------------------------------------------
step 1 "preflight: exactly one openarm_description in the workspace, and it is this one"
found=$(cd "$WS" && colcon list --names-only --packages-select openarm_description 2>/dev/null | wc -l)
path=$(cd "$WS" && colcon list --packages-select openarm_description 2>/dev/null | awk '{print $2}')
echo "openarm_description: $found found ($path)"
if [[ $found -ne 1 ]] || [[ $(cd "$WS" && realpath "$path") != "$PKG" ]]; then
  fail "preflight: openarm_description must come from $PKG only"
fi

# ---------------------------------------------------------------------------
step 2 "colcon build (clean): openarm_description openarm_bringup"
if ! (cd "$WS" && colcon build --packages-select openarm_description openarm_bringup \
      --cmake-clean-first --cmake-args -DBUILD_TESTING=ON >"$TMP/build.log" 2>&1); then
  tail -20 "$TMP/build.log"
  fail "build"
fi

# ---------------------------------------------------------------------------
step 3 "xmllint on every xacro / urdf in the package"
n_files=0; n_bad=0
while IFS= read -r -d '' f; do
  n_files=$((n_files + 1))
  if ! xmllint --noout "$f" 2>"$TMP/xmllint.err"; then
    n_bad=$((n_bad + 1)); head -3 "$TMP/xmllint.err"
  fi
done < <(find "$PKG/urdf" \( -name '*.xacro' -o -name '*.urdf' \) -print0)
echo "xmllint: $n_files files, $n_bad not well-formed"
[[ $n_bad -eq 0 ]] || fail "xmllint: $n_bad files not well-formed"

# ---------------------------------------------------------------------------
step 4 "ament linters vs test/lint_baseline.txt"
(cd "$WS" && colcon test --packages-select openarm_description openarm_bringup \
   >"$TMP/test.log" 2>&1)
python3 - "$WS/build" "$HERE/lint_baseline.txt" <<'PY' || fail "ament linters above baseline"
import os, sys, xml.etree.ElementTree as ET
build, baseline_file = sys.argv[1:]
ok = True
for line in open(baseline_file):
    line = line.split('#')[0].split()
    if not line:
        continue
    pkg, name, base = line[0], line[1], int(line[2])
    path = os.path.join(build, pkg, 'test_results', pkg, f'{name}.xunit.xml')
    if not os.path.exists(path):
        print(f'  {pkg} {name}: no result file'); ok = False; continue
    r = ET.parse(path).getroot()
    n = int(r.get('failures', 0)) + int(r.get('errors', 0))
    ok &= n <= base
    print(f'  {pkg} {name}: {n} failures (baseline {base}) {"ok" if n <= base else "ABOVE BASELINE"}')
sys.exit(0 if ok else 1)
PY

# ---------------------------------------------------------------------------
step 5 "URDF checks (render matrix, meshes, fake rules, golden)"
python3 "$HERE/urdf_checks.py" --install "$WS/install" --golden "$HERE/golden.json" \
  || fail "urdf checks"

# ---------------------------------------------------------------------------
step 6 "fake launch smoke tests (test/fake_smoke.py)"
if ! (
  set +u
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  set -u
  python3 "$HERE/fake_smoke.py" --log-dir "$WS/log/fake_smoke"
); then
  fail "fake smoke"
fi

# ---------------------------------------------------------------------------
echo
if [[ ${#FAILED[@]} -eq 0 ]]; then
  echo "FULL TEST PASSED"
else
  echo "FULL TEST FAILED (${#FAILED[@]}):"
  printf '  - %s\n' "${FAILED[@]}"
  exit 1
fi
