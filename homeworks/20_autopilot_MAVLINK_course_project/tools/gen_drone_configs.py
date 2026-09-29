#!/usr/bin/env python3
"""Генерує include/drone/EmbeddedConfigs.hpp з data/testing_data/NN_*/NN_config.json.

Оскільки на залізі нема файлової системи і парсера JSON, конфіги дрона генеруються та вшиваються в прошивку готовими числами.

Запуск: cd homeworks/20_autopilot_MAVLINK_course_project && python3 tools/gen_drone_configs.py
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TESTING_DATA = ROOT / "data" / "testing_data"
OUTPUT = ROOT / "include" / "drone" / "EmbeddedConfigs.hpp"


def literal(value):
    return f"{float(value)!r}f"


def entry(directory):
    number = directory.name.split("_")[0]
    data = json.loads((directory / f"{number}_config.json").read_text())
    drone, simulation = data["drone"], data["simulation"]

    return f"""  {{.name = "{directory.name}",
   .drone = {{.startPos = {{.x = {literal(drone["position"]["x"])}, .y = {literal(drone["position"]["y"])}}},
             .altitude = {literal(drone["altitude"])},
             .initialDir = {literal(drone["initialDirection"])},
             .v0 = {literal(drone["attackSpeed"])},
             .accelerationPath = {literal(drone["accelerationPath"])},
             .simTimeStep = {literal(simulation["timeStep"])},
             .hitRadius = {literal(simulation["hitRadius"])},
             .angularSpeed = {literal(drone["angularSpeed"])},
             .turnThreshold = {literal(drone["turnThreshold"])}}},
   .physicsTimeStep = {literal(simulation["physicsTimeStep"])}}}"""


def main():
    directories = sorted(d for d in TESTING_DATA.iterdir() if d.is_dir())
    entries = ",\n".join(entry(d) for d in directories)

    OUTPUT.write_text(f"""#pragma once
// ЗГЕНЕРОВАНО tools/gen_drone_configs.py з data/testing_data/*/NN_config.json - НЕ РЕДАГУВАТИ ВРУЧНУ.
// Конфіги дрона, вшиті в прошивку ESP32 готовими числами.
#include <array>

#include "types.hpp"

struct EmbeddedDroneConfig {{
  const char* name;
  DroneConfig drone;
  float physicsTimeStep;
}};

// Номер тесту N (1..{len(directories)}) - елемент з індексом N - 1
inline constexpr std::array<EmbeddedDroneConfig, {len(directories)}> kEmbeddedDroneConfigs{{{{
{entries},
}}}};
""")
    print(f"{OUTPUT.relative_to(ROOT)}: Згенеровано {len(directories)} конфігів")


if __name__ == "__main__":
    main()
