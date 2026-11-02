#!/usr/bin/env bash
# Run the acceptance tests for every robot config, one simulation at a time, then check that
# bad robot configs are rejected with a clear message. Needs a sourced workspace.
#
#   scripts/run_all_tests.sh            # all configs in rb_description/config/robots
#   scripts/run_all_tests.sh default    # only the named configs
set -u

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
share="$(ros2 pkg prefix rb_description)/share/rb_description/config/robots"

if [ $# -gt 0 ]; then
  robots=("$@")
else
  robots=()
  for f in "$share"/*.yaml; do robots+=("$(basename "$f" .yaml)"); done
fi

failed=()
for robot in "${robots[@]}"; do
  echo
  echo "################ $robot"
  if ! python3 "$here/smoke_test.py" --robot "$robot"; then
    failed+=("$robot")
  fi
done

# The launch file must reject bad configs before starting anything.
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
expect_rejected() {  # name expected-text yaml
  local name="$1" text="$2" body="$3"
  printf '%s\n' "$body" > "$tmp/$name.yaml"
  local out
  out="$(timeout 60 ros2 launch rb_bringup sim.launch.py robot:="$tmp/$name.yaml" gui:=false rviz:=false 2>&1)"
  local code=$?
  if [ $code -ne 0 ] && grep -q "$text" <<<"$out"; then
    echo "[PASS] launch rejects $name"
  else
    echo "[FAIL] launch rejects $name (exit $code, wanted message containing '$text')"
    failed+=("validation:$name")
  fi
}
echo
echo "################ config validation"
expect_rejected unknown_drive "unknown drive 'hover'" $'drive: hover\nsensors: []'
expect_rejected unknown_sensor "unknown type 'sonar'" $'drive: diff_2w\nsensors:\n  - {type: sonar, name: s, mount: top_mount}'
expect_rejected unknown_mount "unknown mount 'roof'" $'drive: diff_2w\nsensors:\n  - {type: imu, name: s, mount: roof}'
expect_rejected duplicate_name "duplicate sensor name 'a'" $'drive: diff_2w\nsensors:\n  - {type: imu, name: a, mount: top_mount}\n  - {type: gps, name: a, mount: top_mount}'

echo
if [ ${#failed[@]} -eq 0 ]; then
  echo "ALL TESTS PASSED (${#robots[@]} robot configs)"
else
  echo "FAILED: ${failed[*]}"
  exit 1
fi
