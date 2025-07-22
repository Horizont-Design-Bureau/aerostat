# aerostat

Система управления для модели аэростата в Gazebo

## 🛠 Подготовка

1. [Установите ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) и [Gazebo Harmonic 8.9.0](https://gazebosim.org/docs/harmonic/ros_installation/)

Рекомендуется использовать Ubuntu Noble 24.04

2. Клонируйте репозиторий:

```
$ git clone https://github.com/Horizont-Design-Bureau/aerostat.git
```

3. Переместитесь в каталог проекта:

```
$ cd aerostat
```

4. Соберите проект:

```
$ colcon build --symlink-install
```

5. Обновите окружение оболочки

```
$ source install/setup.bash
```

Замените "setup.bash" под используемую вами оболочку. Если вы используете fish, то установите "bass" и используйте команду "bass source install/setup.bash"

6. Запустите симуляцию

```
$ ros2 launch aerostat_control flight_controller_aerostate.py
```
