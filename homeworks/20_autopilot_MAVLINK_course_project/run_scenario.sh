#!/usr/bin/env bash
# Спрощений запуск vehicle_sim + autopilot + operator_sim для тестування.
# Використання: ./run_scenario.sh [сценарій-оператора] [номер-тесту]
#   сценарій-оператора - ім'я файлу в data/scenarios/ без розширення .txt. Дефолт: 01_clean_attack
#   номер-тесту        - який тесткейс з testing_data використати, 1=01_sample_circles, 2=02_eliptic_trajectories, etc. Дефолт: 1
set -u
cd "$(dirname "$0")"

REPO_ROOT="../.."
BIN="$REPO_ROOT/build/debug/homeworks/20_autopilot_MAVLINK_course_project"

SCENARIO="${1:-01_clean_attack}"
TEST_N="${2:-1}"

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
TIME_SCALE="1"

echo "сценарій:  $OPERATOR_SCENARIO"
echo "тест:      $TEST_N ($DIR)"
echo

trap 'kill 0' EXIT INT TERM

"$BIN/hm20_vehicle_sim" --config-path "$CONFIG" --ammo-path "$AMMO" --time-scale "$TIME_SCALE" &
"$BIN/hm20_autopilot" --config-path "$CONFIG" --ammo-path "$AMMO" --ballistic-table "$BALLISTIC" --sim-output "$REPO_ROOT/simulation.json" &
sleep 1
"$BIN/hm20_operator_sim" --operator-scenario "$OPERATOR_SCENARIO" --config-path "$CONFIG" --ammo-path "$AMMO" --targets "$TARGETS" --time-scale "$TIME_SCALE" &

wait
