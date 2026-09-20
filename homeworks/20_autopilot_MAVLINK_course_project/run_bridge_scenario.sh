#!/usr/bin/env bash
# Те саме, що run_scenario.sh, але дрон говорить з автопілотом і оператором через serial_bridge (імітація роботи з залізом (ESP32) на хості)
# drone_sim (serialDevice) <-> пара віртуальних портів (socat) <-> serial_bridge <-> UDP <-> autopilot + operator_sim
# Використання: ./run_bridge_scenario.sh [сценарій-оператора] [номер-тесту] [time-scale]
#   сценарій-оператора - ім'я файлу в data/scenarios/ без розширення .txt. Дефолт: 01_clean_attack
#   номер-тесту        - який тесткейс з testing_data використати, 1=01_sample_circles, 2=02_eliptic_trajectories, etc. Дефолт: 1
#   time-scale         - прискорення часу. Дефолт: 10; 1 - реальний час, як буде на залізі

set -u
cd "$(dirname "$0")"

REPO_ROOT="../.."
BIN="$REPO_ROOT/build/debug/homeworks/20_autopilot_MAVLINK_course_project"

SCENARIO="${1:-01_clean_attack}"
TEST_N="${2:-1}"
TIME_SCALE="${3:-10}"

case "$TEST_N" in
  1)  DIR="01_sample_circles" ;;
  2)  DIR="02_eliptic_trajectories" ;;
  3)  DIR="03_visimky_lissaju_1-2" ;;
  4)  DIR="04_star_trajectories" ;;
  5)  DIR="05_lissaju_complex_curves" ;;
  6)  DIR="06_fast_drone_slow_targets" ;;
  7)  DIR="07_heavy_ammo" ;;
  8)  DIR="08_gliding_ammo" ;;
  9)  DIR="09_cardioids_eptirohoids" ;;
  10) DIR="10_extreme_far_fast" ;;
  *)  echo "невідомий номер тесту: $TEST_N"; exit 1 ;;
esac

NUM=$(printf '%02d' "$TEST_N")
TEST_DIR="data/testing_data/$DIR"
CONFIG="$TEST_DIR/${NUM}_config.json"
TARGETS="$TEST_DIR/${NUM}_targets.json"
AMMO="data/ammo.json"
BALLISTIC="data/ballistic_table.txt"
OPERATOR_SCENARIO="data/scenarios/$SCENARIO.txt"
DRONE_PORT="/tmp/hm20_pty_drone"    # кінець "дроту" для drone_sim
BRIDGE_PORT="/tmp/hm20_pty_bridge"  # кінець "дроту" для serial_bridge

echo "сценарій:  $OPERATOR_SCENARIO"
echo "тест:      $TEST_N ($DIR), time-scale $TIME_SCALE"
echo

PIDS=()

socat "pty,raw,echo=0,link=$DRONE_PORT" "pty,raw,echo=0,link=$BRIDGE_PORT" &
SOCAT_PID="$!"
trap 'kill "${PIDS[@]}" "$SOCAT_PID" 2>/dev/null' INT TERM
sleep 0.5

"$BIN/hm20_drone_sim" --config-path "$CONFIG" --ammo-path "$AMMO" --time-scale "$TIME_SCALE" --serial-device "$DRONE_PORT" &
PIDS+=("$!")
"$BIN/hm20_serial_bridge" --serial-device "$BRIDGE_PORT" &
PIDS+=("$!")
"$BIN/hm20_autopilot" --config-path "$CONFIG" --ammo-path "$AMMO" --ballistic-table "$BALLISTIC" --time-scale "$TIME_SCALE" &
PIDS+=("$!")
sleep 0.2
"$BIN/hm20_operator_sim" --operator-scenario "$OPERATOR_SCENARIO" --config-path "$CONFIG" --ammo-path "$AMMO" --targets "$TARGETS" --time-scale "$TIME_SCALE" --sim-output "$REPO_ROOT/simulation.json" &
PIDS+=("$!")

# закриття дрота після того як сценарій відпрацював
wait "${PIDS[@]}"
kill "$SOCAT_PID" 2>/dev/null
