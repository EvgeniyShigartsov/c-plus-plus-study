#include "sim/SimulationLog.hpp"

#include <fstream>

#include "third_party/json.hpp"

namespace {

using json = nlohmann::json;

json toJsonXY(const Coord& coord)
{
  return {{"x", coord.x}, {"y", coord.y}};
}

}  // namespace

bool writeSimulationJson(const std::vector<SimStep>& stepsLog, const std::string& path)
{
  json out;

  out["totalSteps"] = stepsLog.size();
  out["steps"] = json::array();

  for (const SimStep& step : stepsLog) {
    json outStep;

    outStep["position"] = toJsonXY(step.pos);
    outStep["direction"] = step.direction;
    outStep["state"] = step.state;
    outStep["targetIndex"] = step.targetIdx;
    outStep["dropPoint"] = toJsonXY(step.dropPoint);
    outStep["aimPoint"] = toJsonXY(step.aimPoint);
    outStep["predictedTarget"] = toJsonXY(step.predictedTarget);
    outStep["timeSecSinceStart"] = step.timeSecSinceStart;

    out["steps"].push_back(outStep);
  }

  std::ofstream outJsonFile(path);
  outJsonFile << out.dump(2);
  return static_cast<bool>(outJsonFile);
}
