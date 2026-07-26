# Homework 14 — robot-dog underground world

## Збірка

```bash
cd homeworks/14_robot-dog-underground-world/robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Запуск

```bash
ros2 launch underground_world system.launch.py scenario:=training_corridor.yaml
```
Сценарії: `training_corridor.yaml`, `small_rooms.yaml`, `branching_trench.yaml`, `dead_end_bunker.yaml`.

## Перевірка результату

```bash
ros2 topic echo /robot/result --once
ros2 topic echo /robot/metrics --once
```

## Запис rosbag

`move_commit_period_ms:=300` для сповільнення місії щоб rosbag устиг підписатися на всі топіки

```bash
# Термінал 1 (рекордер):
ros2 bag record -a -o ../bags/<scenario>

# Термінал 2 (система):
ros2 launch underground_world system.launch.py scenario:=<scenario>.yaml move_commit_period_ms:=300

# Після завершення місії стопнути термінали 1, 2 та перевірити результат
ros2 bag info ../bags/<scenario>
```
