# ДЗ 13 — antidrone_turret (ROS 2)

## Сплощив структуру пакету

У шаблоні курсу пакет лежав у `homework_13/robot_ws/src/antidrone_turret/`.
Тут свідомо підняв його на один рівень, думав мені так зручніше буде.

```
homeworks/13_ros_2_antidrone_turret/   // І корінь ДЗ і сам пакет antidrone_turret
```

Тому всі команди виконуються з цього каталогу, а не з `robot_ws/`, як у ТЗ.

## Передумови

ROS 2 Jazzy встановлено в devcontainer. У кожному новому терміналі спершу:

```bash
source /opt/ros/jazzy/setup.bash
```

Ця команда «вмикає» ROS: додає `ros2`, `colcon` і шляхи до `rclcpp`/`rosidl`.

## Збірка

З каталогу пакета (`homeworks/13_ros_2_antidrone_turret`):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select antidrone_turret
source install/setup.bash
```

## Тести

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select antidrone_turret
colcon test-result --verbose
```

## Запуск наживо

Усі 5 нод одним launch-файлом:

```bash
ros2 launch antidrone_turret system.launch.py
```

Без аргументів проганяються всі треки послідовно (`track:=all`). Окремий трек:

```bash
ros2 launch antidrone_turret system.launch.py track:=approach_trigger.csv
ros2 launch antidrone_turret system.launch.py track:=far_flyby_no_trigger.csv
ros2 launch antidrone_turret system.launch.py track:=low_confidence_no_trigger.csv
ros2 launch antidrone_turret system.launch.py track:=reload_pressure.csv
```

Більше діагностики — вищий рівень логів:

```bash
ros2 launch antidrone_turret system.launch.py track:=reload_pressure.csv log_level:=debug
```

## Перевірка графа ROS

У другому терміналі (теж після `source /opt/ros/jazzy/setup.bash` та
`source install/setup.bash`):

```bash
ros2 node list       # 5 нод
ros2 topic list      # /perception/target /turret/status /gimbal/cmd /servo/cmd /actuator/status
ros2 service list    # /actuator/trigger

ros2 topic echo /turret/status   # рішення контролера на кожен кадр
ros2 topic echo /gimbal/cmd
ros2 topic echo /servo/cmd
```
