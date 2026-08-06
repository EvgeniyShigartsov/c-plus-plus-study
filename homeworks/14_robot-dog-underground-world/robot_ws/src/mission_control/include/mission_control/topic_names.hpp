#pragma once

namespace mission_control {

constexpr auto kLocalScanTopic = "/robot/local_scan";
constexpr auto kResultTopic = "/robot/result";
constexpr auto kMoveTopic = "/robot/cmd_move";
constexpr auto kStudentStatusTopic = "/student/status";
constexpr auto kEnemyDownTopic = "/payload/enemy_down";
constexpr auto kTriggerService = "/payload/trigger";

}  // namespace mission_control
