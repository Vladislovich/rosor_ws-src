#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

class MyPidNode:
    def __init__(self):
        rospy.init_node("my_pid")

        # Задаем коэффициент для преобразования скорости в PWM
        self.coefficient = rospy.get_param("coefficient", 127.0)

        # Подписчики на целевые скорости
        self.left_wheel_sub = rospy.Subscriber("/robot/left_wheel/target_velocity", Float64, self.left_wheel_callback)
        self.right_wheel_sub = rospy.Subscriber("/robot/right_wheel/target_velocity", Float64, self.right_wheel_callback)

        # Публишеры для PWM сигналов
        self.left_wheel_pwm_pub = rospy.Publisher("/robot/left_wheel/pwm", Float64, queue_size=10)
        self.right_wheel_pwm_pub = rospy.Publisher("/robot/right_wheel/pwm", Float64, queue_size=10)

        # Инициализация переменных для хранения скоростей
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        # Частота обновления в Гц
        self.rate = rospy.Rate(50)

    def left_wheel_callback(self, msg):
        self.left_wheel_velocity = msg.data

    def right_wheel_callback(self, msg):
        self.right_wheel_velocity = msg.data

    def velocity_to_pwm(self, velocity):
        pwm = velocity * self.coefficient
        return max(-255, min(255, pwm))

    def run(self):
        while not rospy.is_shutdown():
            # Преобразование скоростей в PWM
            left_pwm = self.velocity_to_pwm(self.left_wheel_velocity)
            right_pwm = self.velocity_to_pwm(self.right_wheel_velocity)

            # Публикация значений PWM
            self.left_wheel_pwm_pub.publish(left_pwm)
            self.right_wheel_pwm_pub.publish(right_pwm)

            # Задержка для поддержания частоты 50 Гц
            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = MyPidNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

class MyPidNode:
    def __init__(self):
        rospy.init_node("my_pid")

        # Задаем коэффициент для преобразования скорости в PWM
        self.coefficient = rospy.get_param("coefficient", 127.0)

        # Подписчики на целевые скорости
        self.left_wheel_sub = rospy.Subscriber("/robot/left_wheel/target_velocity", Float64, self.left_wheel_callback)
        self.right_wheel_sub = rospy.Subscriber("/robot/right_wheel/target_velocity", Float64, self.right_wheel_callback)

        # Публишеры для PWM сигналов
        self.left_wheel_pwm_pub = rospy.Publisher("/robot/left_wheel/pwm", Float64, queue_size=10)
        self.right_wheel_pwm_pub = rospy.Publisher("/robot/right_wheel/pwm", Float64, queue_size=10)

        # Инициализация переменных для хранения скоростей
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        # Частота обновления в Гц
        self.rate = rospy.Rate(50)

    def left_wheel_callback(self, msg):
        self.left_wheel_velocity = msg.data

    def right_wheel_callback(self, msg):
        self.right_wheel_velocity = msg.data

    def velocity_to_pwm(self, velocity):
        pwm = velocity * self.coefficient
        return max(-255, min(255, pwm))

    def run(self):
        while not rospy.is_shutdown():
            # Преобразование скоростей в PWM
            left_pwm = self.velocity_to_pwm(self.left_wheel_velocity)
            right_pwm = self.velocity_to_pwm(self.right_wheel_velocity)

            # Публикация значений PWM
            self.left_wheel_pwm_pub.publish(left_pwm)
            self.right_wheel_pwm_pub.publish(right_pwm)

            # Задержка для поддержания частоты 50 Гц
            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = MyPidNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
