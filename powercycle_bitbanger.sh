#!/usr/bin/env bash
set -euo pipefail

BASE_URL="http://bitbanger.local"

send_pin() {
  local pin="$1"
  local value="$2"
  curl -4 -sS -X POST "${BASE_URL}/api/pin/${pin}/value?value=${value}" >/dev/null
}

wait_for() {
  local seconds="$1"
  sleep "$seconds"
}

power_off() {
  echo "Powering off..."
  send_pin "D0" "-1"
  wait_for 0.5
  send_pin "D0" "1"
  wait_for 0.5
  send_pin "D0" "-1"
  wait_for 1
  send_pin "D0" "1"
  wait_for 0.5
  send_pin "D0" "-1"
}

power_on() {
  echo "Powering on..."
  send_pin "D1" "-1"
  wait_for 0.5
  send_pin "D1" "1"
  wait_for 0.5
  send_pin "D1" "-1"
  wait_for 1
  send_pin "D1" "1"
  wait_for 0.5
  send_pin "D1" "-1"
}

power_off
power_on

echo "Power cycle sequence completed."
