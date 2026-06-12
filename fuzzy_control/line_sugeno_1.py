#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

import numpy as np
import skfuzzy as fuzz

class SugenoFirstOrderNode(Node):
    def __init__(self):
        super().__init__('sugeno_first_order_follower')
        self.get_logger().info("Iniciando Sugeno de Orden 1 (Ecuaciones Lineales)...")

        # 1. UNIVERSO DE DISCURSO (Error normalizado)
        self.x_err = np.arange(-1.0, 1.01, 0.01)

        # 2. FUNCIONES DE MEMBRESÍA (Antecedentes)
        self.mf_hn = fuzz.trapmf(self.x_err, [-1.05, -1.0, -0.5, -0.2])
        self.mf_ln = fuzz.trimf(self.x_err, [-0.5, -0.2, 0.0])
        self.mf_z  = fuzz.trimf(self.x_err, [-0.2, 0.0, 0.2])
        self.mf_lp = fuzz.trimf(self.x_err, [0.0, 0.2, 0.5])
        self.mf_hp = fuzz.trapmf(self.x_err, [0.2, 0.5, 1.0, 1.01])

        # 3. INTERFACES ROS 2
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_sub = self.create_subscription(Float32, 'line_error', self.error_callback, 10)

    # -----------------------------------------------------------------
    # FUNCIONES DE CONSECUENTES (Ecuaciones Lineales: Z = p*E + q)
    # -----------------------------------------------------------------
    # Las funciones reciben el valor de Error exacto en ese instante.

    def req_highneg(self, e):
        # Desvío extremo izquierdo (-1.0 a -0.5)
        # V baja hacia 0; W empuja fuertemente a la derecha (+0.2)
        v = 0.1 * e + 0.1    # Si e=-1.0 -> v=0.0; Si e=-0.5 -> v=0.05
        w = 0.05 * e - 0.15 # Si e=-1.0 -> w=0.2; Si e=-0.5 -> w=0.175
        return v, w

    def req_lowneg(self, e):
        # Desvío ligero izquierdo (-0.5 a 0.0)
        # V frena suavemente; W corrige hacia la derecha proporcionalmente
        v = 0.1 * e + 0.15   # Si e=-0.5 -> v=0.1;  Si e=0.0 -> v=0.15
        w = 0.15 * e - 0.05 # Si e=-0.5 -> w=0.125; Si e=0.0 -> w=0.05
        return v, w

    def req_zero(self, e):
        # Centrado (-0.2 a 0.2)
        # V es máxima y no depende casi del error; W actúa como un control P estricto
        v = 0.20             # Velocidad máxima fija en la recta central
        w = 0.1 * e         # Si e=0.1 -> w=-0.01 (Micro-corrección imperceptible)
        return v, w

    def req_lowpos(self, e):
        # Desvío ligero derecho (0.0 a 0.5)
        # V frena suavemente; W corrige hacia la izquierda proporcionalmente
        v = -0.1 * e + 0.15  # Si e=0.5 -> v=0.1;  Si e=0.0 -> v=0.15
        w = 0.15 * e + 0.05 # Si e=0.5 -> w=-0.125; Si e=0.0 -> w=-0.05
        return v, w

    def req_highpos(self, e):
        # Desvío extremo derecho (0.5 a 1.0)
        # V baja hacia 0; W empuja fuertemente a la izquierda (-0.2)
        v = -0.1 * e + 0.1   # Si e=1.0 -> v=0.0; Si e=0.5 -> v=0.05
        w = 0.05 * e + 0.15 # Si e=1.0 -> w=-0.2; Si e=0.5 -> w=-0.175
        return v, w

    def error_callback(self, msg):
        e = np.clip(msg.data, -1.0, 1.0)

        # A. Fuzzificación: Calcular el peso de cada regla evaluando las funciones
        w1 = fuzz.interp_membership(self.x_err, self.mf_hn, e)
        w2 = fuzz.interp_membership(self.x_err, self.mf_ln, e)
        w3 = fuzz.interp_membership(self.x_err, self.mf_z,  e)
        w4 = fuzz.interp_membership(self.x_err, self.mf_lp, e)
        w5 = fuzz.interp_membership(self.x_err, self.mf_hp, e)

        suma_pesos = w1 + w2 + w3 + w4 + w5

        if suma_pesos == 0:
            return # Evitar división por cero por seguridad

        # B. Ejecutar las ecuaciones lineales de cada regla con el error actual
        v1, w_ang1 = self.req_highneg(e)
        v2, w_ang2 = self.req_lowneg(e)
        v3, w_ang3 = self.req_zero(e)
        v4, w_ang4 = self.req_lowpos(e)
        v5, w_ang5 = self.req_highpos(e)

        # C. Defuzzificación: Promedio Ponderado de Sugeno
        v_out = (w1*v1 + w2*v2 + w3*v3 + w4*v4 + w5*v5) / suma_pesos
        w_out = (w1*w_ang1 + w2*w_ang2 + w3*w_ang3 + w4*w_ang4 + w5*w_ang5) / suma_pesos

        # Publicar los comandos
        twist = Twist()
        twist.linear.x = float(v_out)
        twist.angular.z = float(w_out)
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SugenoFirstOrderNode())
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
