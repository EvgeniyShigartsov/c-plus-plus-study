#!/usr/bin/env bash
# Запуск оператора (operator_sim), ЧАСТИНА 2 з 2. Стартувати ПІСЛЯ рядка "READY" у run_hardware_pi.sh на Pi
# Автопілот і міст працюють на Pi, оператор шле їм HEARTBEAT, команди, цілі та RC override по мережі.
# simulation.json записується тут, де запущено оператора.
# Використання: ./run_hardware_operator.sh <сценарій-оператора> <номер-тесту> <ip-pi>
#   сценарій-оператора - ім'я файлу в data/scenarios/ без розширення .txt, наприклад 01_clean_attack
#   номер-тесту        - той самий, що й у run_hardware_pi.sh
#   ip-pi              - числовий IP Raspberry Pi, який бачить оператор (hostname -I на Pi)

set -u
cd "$(dirname "$0")"

REPO_ROOT="../.."
BIN="$REPO_ROOT/build/debug/homeworks/20_autopilot_MAVLINK_course_project"

if [ $# -lt 3 ]; then
  echo "Використання: $0 <сценарій-оператора> <номер-тесту> <ip-pi>"
  echo "Приклад:      $0 03_link_lost_failsafe 4 192.168.2.6"
  exit 1
fi

SCENARIO="$1"
TEST_N="$2"
PI_HOST="$3"
TIME_SCALE=1  # на залізі завжди реальний час (у прошивці kTimeScale = 1), без цього оператор візьме time-scale з config.json

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
OPERATOR_SCENARIO="data/scenarios/$SCENARIO.txt"

echo "сценарій:  $OPERATOR_SCENARIO"
echo "тест:      $TEST_N ($DIR)"
echo "Pi:        $PI_HOST (автопілот :14560, міст :14555)"
echo

# Автопілот і дрон (через міст) на Pi. Свій порт для телеметрії, статусу і SimStep оператор слухає сам (14550)
"$BIN/hm20_operator_sim" --operator-scenario "$OPERATOR_SCENARIO" --config-path "$CONFIG" --ammo-path "$AMMO" --targets "$TARGETS" \
  --time-scale "$TIME_SCALE" --ap-host "$PI_HOST" --drone-host "$PI_HOST" --sim-output "$REPO_ROOT/simulation.json"
