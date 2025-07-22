#!/usr/bin/env python3
import rclpy  # Импортируем библиотеку ROS 2 для работы с узлами
from rclpy.node import Node  # Импортируем класс Node для создания узлов
from sensor_msgs.msg import Imu, Image  # Импортируем типы сообщений для данных IMU и изображений с камеры
from nav_msgs.msg import Odometry  # Импортируем тип сообщения для данных одометрии
from geometry_msgs.msg import PoseStamped  # Импортируем тип сообщения для координат в глобальной системе
from cv_bridge import CvBridge  # Импортируем класс для преобразования между форматами изображений ROS и OpenCV
import cv2  # Импортируем библиотеку OpenCV для обработки изображений
import numpy as np  # Импортируем библиотеку NumPy для работы с массивами
from rclpy.qos import QoSProfile, ReliabilityPolicy  # Импортируем классы для настройки качества обслуживания (QoS)

class SensorFusion(Node):
    """
    Класс узла SensorFusion для объединения данных с используемых сенсоров (IMU, UWB, камера)
    и публикации данных одометрии.
    """
    def __init__(self):
        """
        Инициализация узла.
        """
        super().__init__('sensor_fusion')  # Инициализация узла с именем 'sensor_fusion'
        
        # QoS для сенсоров (лучшая совместимость с Gazebo)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # depth=10 - очередь в 10 сообщений (для каждого топика)
        # reliability=BEST_EFFORT - попытки доставить сообщения, но без гарантии

        # Подписки
        self.imu_sub = self.create_subscription(
            Imu,  # Тип сообщения
            '/imu/data',  # Имя топика, на который подписываемся
            self.imu_callback,  # Callback-функция для обработки данных IMU
            qos)  # QoS для подписки
        self.uwb_sub = self.create_subscription(
            PoseStamped,  # Тип сообщения
            '/world/quadcopter/dynamic_pose/info',  # Имя топика, на который подписываемся (UWB позиция)
            self.uwb_callback,  # Callback-функция для обработки данных UWB
            qos)  # QoS для подписки
        self.camera_sub = self.create_subscription(
            Image,  # Тип сообщения
            '/camera/image_raw',  # Имя топика, на который подписываемся (изображение с камеры)
            self.camera_callback,  # Callback-функция для обработки данных с камеры
            qos)  # QoS для подписки
        
        # Публикация объединённых данных
        self.odom_pub = self.create_publisher(
            Odometry,  # Тип сообщения, которое публикуем
            '/aerostat/odometry',  # Имя топика, куда публикуем
            10)  # QoS depth: размер очереди публикаций - 10 сообщений
        
        # Инициализация
        self.cv_bridge = CvBridge()  # Создаем объект CvBridge для преобразования изображений
        self.last_imu = None  # Переменная для хранения последних данных с IMU
        self.last_uwb = None  # Переменная для хранения последних данных с UWB
        self.last_frame = None  # Переменная для хранения последнего кадра с камеры
        self.optical_flow = None  # Переменная для хранения оптического потока
        self.odom_msg = Odometry()  # Создаем сообщение типа Odometry для публикации
        
        self.get_logger().info("Sensor Fusion Node запущен")  # Вывод сообщения в лог о запуске узла

    def imu_callback(self, msg):
        """
        Обработчик данных с IMU.
        Сохраняет полученные данные и вызывает функцию обновления одометрии.

        Args:
            msg (Imu): Сообщение с данными IMU.
        """
        self.last_imu = msg  # Сохраняем данные IMU
        self.update_odometry()  # Вызываем функцию обновления одометрии

    def uwb_callback(self, msg):
        """
        Обработчик данных с UWB.
        Сохраняет полученные данные и вызывает функцию обновления одометрии.

        Args:
            msg (PoseStamped): Сообщение с данными о позиции UWB.
        """
        self.last_uwb = msg.pose  # Сохраняем данные о позиции UWB
        self.update_odometry()  # Вызываем функцию обновления одометрии

    def camera_callback(self, msg):
        """
        Обработчик данных с камеры.
        Вычисляет оптический поток и сохраняет его для дальнейшего использования.

        Args:
            msg (Image): Сообщение с изображением с камеры.
        """
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")  # Преобразуем изображение из ROS-формата в формат OpenCV
            if self.last_frame is not None:
                # Рассчитываем оптический поток (Farneback)
                gray_prev = cv2.cvtColor(self.last_frame, cv2.COLOR_BGR2GRAY)  # Преобразуем предыдущий кадр в grayscale
                gray_current = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Преобразуем текущий кадр в grayscale
                flow = cv2.calcOpticalFlowFarneback(
                    gray_prev, gray_current, None, 0.5, 3, 15, 3, 5, 1.2, 0
                )  # Расчет оптического потока
                self.optical_flow = np.mean(flow, axis=(0, 1))  # Усредняем оптический поток по кадру
            self.last_frame = frame  # Сохраняем текущий кадр как предыдущий для следующей итерации
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки камеры: {e}")  # Выводим ошибку в лог, если что-то пошло не так

    def update_odometry(self):
        """
        Объединяет данные с различных сенсоров и публикует данные одометрии.
        """
        if not (self.last_imu and self.last_uwb):
            return  # Если нет данных ни с IMU, ни с UWB, то ничего не делаем
        
        # Основные координаты из UWB
        self.odom_msg.pose.pose = self.last_uwb  # Устанавливаем позицию из UWB в сообщение одометрии
        
        # Коррекция через оптический поток (если есть)
        if self.optical_flow is not None:
            self.odom_msg.pose.pose.position.x += self.optical_flow[0] * 0.01  # Коррекция позиции по X
            self.odom_msg.pose.pose.position.y += self.optical_flow[1] * 0.01  # Коррекция позиции по Y
        
        # Ориентация из IMU
        self.odom_msg.pose.pose.orientation = self.last_imu.orientation  # Устанавливаем ориентацию из IMU
        
        # Временная метка
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()  # Устанавливаем текущее время
        self.odom_msg.header.frame_id = "odom"  # Устанавливаем ID фрейма
        
        # Публикация
        self.odom_pub.publish(self.odom_msg)  # Публикуем сообщение одометрии

def main(args=None):
    """
    Главная функция.
    Инициализирует ROS 2, создает узел SensorFusion и запускает цикл обработки сообщений.
    """
    rclpy.init(args=args)  # Инициализируем ROS 2
    node = SensorFusion()  # Создаем экземпляр узла SensorFusion
    try:
        rclpy.spin(node)  # Запускаем цикл обработки сообщений
    except KeyboardInterrupt:
        pass  # Обрабатываем прерывание с клавиатуры (Ctrl+C)
    finally:
        node.destroy_node()  # Уничтожаем узел
        rclpy.shutdown()  # Завершаем работу ROS 2

if __name__ == '__main__':
    main()  # Запускаем главную функцию, если скрипт запущен напрямую