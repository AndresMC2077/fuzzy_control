#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

import numpy as np
import skfuzzy as fuzz

class SugenoTunedNode(Node):
    def __init__(self):
        super().__init__('sugeno_tuned_follower')
        self.get_logger().info("Iniciando Sugeno Orden 0 (TUNEADO)...")
        #universo del error y funciones de membresia
        self.x_err = np.arange(-1.0, 1.01, 0.01)

        self.mf_hn = fuzz.trapmf(self.x_err, [-1.05, -1.0, -0.5, -0.2])
        self.mf_ln = fuzz.trimf(self.x_err, [-0.5, -0.2, 0.0])
        self.mf_z  = fuzz.trimf(self.x_err, [-0.2, 0.0, 0.2])
        self.mf_lp = fuzz.trimf(self.x_err, [0.0, 0.2, 0.5])
        self.mf_hp = fuzz.trapmf(self.x_err, [0.2, 0.5, 1.0, 1.01])

        #COnstantes de sujeno
        # R1: HighNeg (Desvío Extremo Izquierda) -> Freno total frontal, giro rápido para recuperar.
        self.v_hn, self.w_hn = 0.0, -0.20   
        
        # R2: LowNeg (Desvío Ligero Izquierda) -> Desacelera suavemente, aplica corrección proporcional.
        self.v_ln, self.w_ln = 0.12, -0.08   
        
        # R3: Zero (Alineado) -> Máxima velocidad, nada de giro.
        self.v_z, self.w_z   = 0.20, 0.0   
        
        # R4: LowPos (Desvío Ligero Derecha) -> Desacelera a 0.12, gira suave a -0.08.
        self.v_lp, self.w_lp = 0.12, 0.08  
        
        # R5: HighPos (Desvío Extremo Derecha) -> Freno total frontal, giro rápido a la derecha.
        self.v_hp, self.w_hp = 0.0, 0.20  

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_sub = self.create_subscription(Float32, 'line_error', self.error_callback, 10)

    def error_callback(self, msg):
        error_val = np.clip(msg.data, -1.0, 1.0)

        # Grados de activación (Fuzzificación)
        w1 = fuzz.interp_membership(self.x_err, self.mf_hn, error_val)
        w2 = fuzz.interp_membership(self.x_err, self.mf_ln, error_val)
        w3 = fuzz.interp_membership(self.x_err, self.mf_z,  error_val)
        w4 = fuzz.interp_membership(self.x_err, self.mf_lp, error_val)
        w5 = fuzz.interp_membership(self.x_err, self.mf_hp, error_val)

        suma_pesos = w1 + w2 + w3 + w4 + w5

        if suma_pesos == 0:
            return

        # Defuzzificación Sugeno (suma ponderada)
        v_out = (w1*self.v_hn + w2*self.v_ln + w3*self.v_z + w4*self.v_lp + w5*self.v_hp) / suma_pesos
        w_out = (w1*self.w_hn + w2*self.w_ln + w3*self.w_z + w4*self.w_lp + w5*self.w_hp) / suma_pesos

        twist = Twist()
        twist.linear.x = float(v_out)
        twist.angular.z = float(w_out)
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SugenoTunedNode())
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
