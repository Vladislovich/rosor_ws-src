#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from collections import deque

class WheelVelocityPublisher:
    def __init__(self):
        # Подписка на топики энкодеров
        self.left_encoder_sub = rospy.Subscriber("/robot/left_wheel/encoder", Float64, self.left_encoder_callback)
        self.right_encoder_sub = rospy.Subscriber("/robot/right_wheel/encoder", Float64, self.right_encoder_callback)

        # Публикация текущей скорости колёс
        self.left_velocity_pub = rospy.Publisher("/robot/left_wheel/current_velocity", Float64, queue_size=10)
        self.right_velocity_pub = rospy.Publisher("/robot/right_wheel/current_velocity", Float64, queue_size=10)

        # Инициализация переменных
        self.prev_left_rad = 0.0
        self.prev_right_rad = 0.0
        self.prev_time = rospy.Time.now()
        self.velocity_koef = 100

        # Очереди для фильтра скользящего среднего
        self.left_velocity_history = deque(maxlen=5)  # Храним последние 5 значений скорости
        self.right_velocity_history = deque(maxlen=5)

    def left_encoder_callback(self, msg):
        current_time = rospy.Time.now()
        delta_time = (current_time - self.prev_time).to_sec()

        if delta_time > 0:
            # Расчёт текущей скорости левого колеса
            delta_left_rad = msg.data - self.prev_left_rad
            left_velocity = delta_left_rad / delta_time

            # Добавление скорости в очередь для фильтрации
            self.left_velocity_history.append(left_velocity)

            # Вычисление среднего значения
            smoothed_left_velocity = sum(self.left_velocity_history) / len(self.left_velocity_history)

            # Публикация сглаженной скорости
            self.left_velocity_pub.publish(Float64(smoothed_left_velocity / self.velocity_koef))

            # Обновление переменных
            self.prev_left_rad = msg.data
            self.prev_time = current_time

    def right_encoder_callback(self, msg):
        current_time = rospy.Time.now()
        delta_time = (current_time - self.prev_time).to_sec()

        if delta_time > 0:
            # Расчёт текущей скорости правого колеса
            delta_right_rad = msg.data - self.prev_right_rad
            right_velocity = delta_right_rad / delta_time

            # Добавление скорости в очередь для фильтрации
            self.right_velocity_history.append(right_velocity)

            # Вычисление среднего значения
            smoothed_right_velocity = sum(self.right_velocity_history) / len(self.right_velocity_history)

            # Публикация сглаженной скорости
            self.right_velocity_pub.publish(Float64(smoothed_right_velocity / self.velocity_koef))

            # Обновление переменных
            self.prev_right_rad = msg.data
            self.prev_time = current_time

if __name__ == "__main__":
    rospy.init_node("wheel_velocity_publisher")
    wheel_velocity_publisher = WheelVelocityPublisher()
    rospy.spin()
