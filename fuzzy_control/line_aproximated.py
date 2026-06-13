#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twisthttps://github.com/AndresMC2077/fuzzy_control/pulse
import math
import numpy as np

class NonLinearControllerNode(Node):
    def __init__(self):
        super().__init__('nonlinear_math_follower')
        self.get_logger().info("Iniciando Controlador No Lineal (Gaussiana + Tanh)...")

        # Suscriptor al error normalizado
        self.error_sub = self.create_subscription(
            Float32, 
            '/line_error', 
            self.error_callback, 
            10
        )
        # Publicador de velocidades
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def error_callback(self, msg):
        # 1. Recibir error (Asegurando rango seguro -1.0 a 1.0)
        x = np.clip(msg.data, -1.0, 1.0)
        # Calculo de vel lineal x
        v_out = 0.0
        
        # Aplicamos el truncado estricto a cero si el desvío es mayor o igual a 0.5
        if abs(x) >= 0.5:
            v_out = 0.0
        else:
            A_v = 0.15     # Altura máxima en el pico
            mu = 0.0       # Centro del pico
            sigma = 0.165  # Decaimiento
            
            # Ecuación Gaussiana para valores escalares
            v_out = A_v * math.exp(-((x - mu) ** 2) / (2 * sigma ** 2))

        #Calculo de vel angular w
        A_w = 0.1455  # Amplitud de la tanh
        k = 4.0       # Pendiente / Inclinación
        w_raw = A_w * math.tanh(k * x)
        
        # Aplicar la saturación exacta en los límites cinemáticos
        w_out = np.clip(w_raw, -0.1453, 0.1453)

        # publish
        twist = Twist()
        twist.linear.x = float(v_out)
        twist.angular.z = float(w_out)
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = NonLinearControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
