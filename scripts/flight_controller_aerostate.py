#!/usr/bin/env python3
import rclpy  # Импортируем библиотеку ROS 2 для работы с узлами
from rclpy.node import Node  # Импортируем базовый класс Node для создания узлов ROS 2
from nav_msgs.msg import Odometry  # Импортируем тип сообщения Odometry для получения данных о положении и ориентации
from gz_msgs.msg import Actuators  # Импортируем пользовательский тип сообщения Actuators для управления моторами (предполагается, что он определен в gz_msgs)
from geometry_msgs.msg import Point, Quaternion  # Импортируем типы сообщений Point и Quaternion для представления точек и кватернионов
from rclpy.qos import QoSProfile, ReliabilityPolicy  # Импортируем классы для настройки качества обслуживания (QoS)
import math  # Импортируем модуль math для математических операций, таких как sin, cos, atan2
import numpy as np  # Импортируем библиотеку NumPy для работы с массивами и математическими операциями (клиппинг)

class FlightController(Node):
    """
    Класс FlightController, представляющий узел управления полетом квадрокоптера.
    Отвечает за обработку данных одометрии, вычисление управляющих сигналов и отправку команд моторам.
    """
    def __init__(self):
        """
        Инициализация узла FlightController.
        """
        super().__init__('flight_controller')  # Инициализация базового класса Node с именем узла 'flight_controller'
        
        # Параметры PID (настройте под свою модель!)
        self.pid_gains = {
            'x': {'p': 0.5, 'i': 0.01, 'd': 0.1},  # Коэффициенты PID для управления положением по оси X
            'y': {'p': 0.5, 'i': 0.01, 'd': 0.1},  # Коэффициенты PID для управления положением по оси Y
            'z': {'p': 0.8, 'i': 0.02, 'd': 0.2},  # Коэффициенты PID для управления высотой (по оси Z)
            'yaw': {'p': 0.3, 'i': 0.0, 'd': 0.05}  # Коэффициенты PID для управления углом рысканья (yaw)
        }
        
        # Точки маршрута (квадрат 2x2 м, высота 1.5 м)
        self.waypoints = [
            Point(x=0.0, y=0.0, z=1.5),  # Взлёт - начальная точка, где квадрокоптер должен подняться на высоту 1.5 м
            Point(x=2.0, y=0.0, z=1.5),  # Угол 1 - первая точка на маршруте
            Point(x=2.0, y=2.0, z=1.5),  # Угол 2 - вторая точка на маршруте
            Point(x=0.0, y=2.0, z=1.5),  # Угол 3 - третья точка на маршруте
            Point(x=0.0, y=0.0, z=1.5),  # Возврат - возврат в исходную точку на высоте 1.5 м
            Point(x=0.0, y=0.0, z=0.0)   # Посадка - посадка в исходной точке (z=0)
        ]
        self.current_waypoint_idx = 0  # Индекс текущей целевой точки маршрута
        
        # Ошибки и интеграторы PID
        self.errors = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}  # Текущие ошибки по осям
        self.integrals = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}  # Интеграторы PID для каждой оси (для борьбы со статическими ошибками)
        self.last_errors = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}  # Предыдущие ошибки (для вычисления производной ошибки)
        
        # Подписка и публикация
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)  # Настройка QoS для подписок и публикаций (лучшая попытка доставки)
        self.odom_sub = self.create_subscription(
            Odometry,  # Тип сообщения, которое мы получаем
            '/aerostat/odometry',  # Имя топика, на который мы подписываемся (данные одометрии)
            self.odom_callback,  # Callback-функция для обработки полученных данных одометрии
            qos)  # QoS для подписки
        self.motor_pub = self.create_publisher(
            Actuators,  # Тип сообщения, которое мы публикуем (команды управления моторами)
            '/X3/gazebo/command/motor_speed',  # Имя топика, в который мы публикуем (команды управления моторами)
            10)  # QoS depth - размер очереди для публикации сообщений (10 сообщений)
        
        # Таймер для управления (частота 10 Гц)
        self.control_timer = self.create_timer(0.1, self.control_loop)  # Создаем таймер, который вызывает control_loop каждые 0.1 секунды (10 Гц)
        
        self.current_pose = None  # Текущая позиция и ориентация квадрокоптера (инициализируется как None)
        self.get_logger().info("Flight Controller Node запущен")  # Вывод информационного сообщения в лог о запуске узла

    def odom_callback(self, msg):
        """
        Обратный вызов для обработки данных одометрии.
        Сохраняет текущую позицию и ориентацию квадрокоптера.

        Args:
            msg (Odometry): Сообщение с данными одометрии.
        """
        self.current_pose = msg.pose.pose  # Сохраняем текущую позицию и ориентацию из сообщения одометрии

    def quaternion_to_yaw(self, q: Quaternion) -> float:
        """
        Конвертация кватерниона в угол yaw (угол рысканья) в радианах.

        Args:
            q (Quaternion): Кватернион, представляющий ориентацию.

        Returns:
            float: Угол yaw в радианах.
        """
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)  # Компонент для вычисления sin(yaw) * cos(pitch)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)  # Компонент для вычисления cos(yaw) * cos(pitch)
        return math.atan2(siny_cosp, cosy_cosp)  # Вычисляем угол yaw с помощью atan2

    def calculate_pid(self, axis: str, error: float) -> float:
        """
        Вычисление PID-коррекции для заданной оси.

        Args:
            axis (str): Ось, для которой вычисляется PID-коррекция ('x', 'y', 'z', 'yaw').
            error (float): Ошибка (разница между целевым и текущим значением) для данной оси.

        Returns:
            float: Вычисленное значение PID-коррекции.
        """
        p = self.pid_gains[axis]['p'] * error  # P-компонент: пропорциональная ошибка
        self.integrals[axis] += self.pid_gains[axis]['i'] * error  # I-компонент: интегратор, накапливает ошибку
        d = self.pid_gains[axis]['d'] * (error - self.last_errors[axis])  # D-компонент: производная ошибки (изменение ошибки)
        self.last_errors[axis] = error  # Сохраняем текущую ошибку как предыдущую для следующей итерации
        return p + self.integrals[axis] + d  # Возвращаем сумму P, I и D компонентов

    def control_loop(self):
        """
        Основной цикл управления.
        Выполняется с заданной частотой (10 Гц) и вычисляет управляющие сигналы для моторов.
        """
        if not self.current_pose:
            return  # Если данные о положении не получены, ничего не делаем

        # Текущая целевая точка
        target = self.waypoints[self.current_waypoint_idx]  # Получаем текущую целевую точку из списка

        # Ошибки позиции
        error_x = target.x - self.current_pose.position.x  # Вычисляем ошибку по оси X
        error_y = target.y - self.current_pose.position.y  # Вычисляем ошибку по оси Y
        error_z = target.z - self.current_pose.position.z  # Вычисляем ошибку по оси Z

        # Ошибка ориентации (yaw)
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)  # Получаем текущий угол рысканья
        target_yaw = 0.0  # Задаем целевой угол рысканья (по умолчанию "вперед" по оси X)
        error_yaw = target_yaw - current_yaw  # Вычисляем ошибку по углу рысканья

        # Нормализация угла
        if error_yaw > math.pi:
            error_yaw -= 2 * math.pi  # Нормализуем угол, чтобы он был в диапазоне [-pi, pi]
        elif error_yaw < -math.pi:
            error_yaw += 2 * math.pi  # Нормализуем угол

        # Вычисление PID
        u_x = self.calculate_pid('x', error_x)  # Вычисляем PID-коррекцию для оси X
        u_y = self.calculate_pid('y', error_y)  # Вычисляем PID-коррекцию для оси Y
        u_z = self.calculate_pid('z', error_z)  # Вычисляем PID-коррекцию для оси Z (высота)
        u_yaw = self.calculate_pid('yaw', error_yaw)  # Вычисляем PID-коррекцию для угла рысканья

        # Преобразование в скорости моторов (минимальная базовая тяга + корректировки)
        base_thrust = 400.0  # Базовая скорость моторов для удержания высоты (настройте этот параметр)
        motor_speeds = [
            base_thrust + u_z - u_yaw,  # Мотор 0 (задний правый, CCW)
            base_thrust + u_z + u_yaw,  # Мотор 1 (задний левый, CW)
            base_thrust + u_z - u_x + u_y,  # Мотор 2 (верхний, CCW)
            base_thrust + u_z - u_x - u_y   # Мотор 3 (нижний, CW)
        ]

        # Ограничение скоростей (0-800 по спецификации из мира Gazebo)
        motor_speeds = np.clip(motor_speeds, 0.0, 800.0).tolist()  # Ограничиваем значения скоростей моторов, используя NumPy

        # Публикация команд
        msg = Actuators()  # Создаем экземпляр сообщения Actuators
        msg.velocity = motor_speeds  # Заполняем поле velocity сообщения скоростями моторов
        self.motor_pub.publish(msg)  # Публикуем команды моторам

        # Проверка достижения точки
        if (abs(error_x) < 0.1 and abs(error_y) < 0.1 and abs(error_z) < 0.1):  # Проверяем, достигнута ли целевая точка
            self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)  # Переходим к следующей точке маршрута
            self.get_logger().info(f"Достигнута точка {self.current_waypoint_idx}")  # Выводим сообщение в лог о достижении точки маршрута

def main(args=None):
    """
    Главная функция.
    Инициализирует ROS 2, создает узел FlightController и запускает цикл обработки событий.
    """
    rclpy.init(args=args)  # Инициализация ROS 2
    node = FlightController()  # Создание экземпляра узла FlightController
    try:
        rclpy.spin(node)  # Запуск цикла обработки событий, пока узел не будет уничтожен
    except KeyboardInterrupt:
        pass  # Обработка прерывания (Ctrl+C)
    finally:
        node.destroy_node()  # Уничтожение узла при завершении работы
        rclpy.shutdown()  # Завершение работы ROS 2

if __name__ == '__main__':
    main()  # Запуск главной функции, если скрипт запускается напрямую