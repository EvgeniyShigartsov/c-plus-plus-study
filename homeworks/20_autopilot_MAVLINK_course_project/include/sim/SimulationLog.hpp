#pragma once
#include <string>
#include <vector>

#include "types.hpp"

bool writeSimulationJson(const std::vector<SimStep>& stepsLog, const std::string& path);
