#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtleBotRotate(Node):
    def __init__(self):
        super().__init__('turtlebot_rotate')

        # Publisher dla tematu /cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Cykliczny timer, co 1 sekundę
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('TurtleBot will start rotating!')

    def timer_callback(self):
        # Tworzymy wiadomość typu Twist
        msg = Twist()

        # Ustawiamy prędkość kątową (zamiast prędkości liniowej)
        msg.angular.z = 0.5  # Rotacja w prawo lub w lewo

        # Publikujemy wiadomość
        self.publisher.publish(msg)

        # Logowanie w terminalu
        self.get_logger().info(f'Publishing rotation command: angular.z = {msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)

    # Inicjalizacja węzła
    node = TurtleBotRotate()

    # Uruchamiamy węzeł
    rclpy.spin(node)

    # Sprzątanie
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

