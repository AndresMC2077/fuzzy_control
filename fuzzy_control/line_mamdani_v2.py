#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl


class MamdaniControllerNode(Node):
    def __init__(self):
        super().__init__('mamdani_line_follower_v2')
        
        self.get_logger().info("Inicializando Controlador Difuso Mamdani (Entrada Normalizada)...")

        # ---------------- 1. UNIVERSO DE DISCURSO ----------------
        # Entrada: Error Normalizado [-1.0, 1.0]
        # Usamos un paso de 0.01 para tener buena resolución en la inferencia
        self.E = ctrl.Antecedent(np.arange(-1.0, 1.01, 0.01), 'Error')
        
        # Salidas: Velocidad Lineal (V) en m/s y Angular (W) en rad/s
        self.V = ctrl.Consequent(np.arange(0.0, 0.21, 0.001), 'V')
        self.W = ctrl.Consequent(np.arange(-0.2, 0.21, 0.001), 'W')

        # ---------------- 2. FUNCIONES DE MEMBRESÍA ----------------
        # Escaladas al nuevo rango [-1.0, 1.0] manteniendo la lógica de trapezoides en los extremos
        
        self.E['highneg'] = fuzz.trapmf(self.E.universe, [-1.0, -1.0, -0.5, -0.2])
        self.E['lowneg']  = fuzz.trimf(self.E.universe, [-0.5, -0.2, 0.0])
        self.E['zero']    = fuzz.trimf(self.E.universe, [-0.2, 0.0, 0.2])
        self.E['lowpos']  = fuzz.trimf(self.E.universe, [0.0, 0.2, 0.5])
        self.E['highpos'] = fuzz.trapmf(self.E.universe, [0.2, 0.5, 1.0, 1.01])

        # Las salidas se mantienen intactas según tus especificaciones cinemáticas
        self.V['zero'] = fuzz.trapmf(self.V.universe, [0.0, 0.0, 0.02, 0.06])
        self.V['low']  = fuzz.trimf(self.V.universe, [0.03, 0.08, 0.13])
        self.V['high'] = fuzz.trapmf(self.V.universe, [0.10, 0.15, 0.20, 0.20])

        self.W['highder'] = fuzz.trapmf(self.W.universe, [-0.2, -0.2, -0.15, -0.05])
        self.W['lowder']  = fuzz.trimf(self.W.universe, [-0.15, -0.05, 0.0])
        self.W['zero']    = fuzz.trimf(self.W.universe, [-0.05, 0.0, 0.05])
        self.W['lowizq']  = fuzz.trimf(self.W.universe, [0.0, 0.05, 0.15])
        self.W['highizq'] = fuzz.trapmf(self.W.universe, [0.05, 0.15, 0.2, 0.2])

        # ---------------- 3. BASE DE REGLAS ----------------
        rule1 = ctrl.Rule(self.E['zero'], (self.V['high'], self.W['zero']))
        rule2 = ctrl.Rule(self.E['lowpos'], (self.V['low'], self.W['lowizq']))
        rule3 = ctrl.Rule(self.E['highpos'], (self.V['zero'], self.W['highizq']))
        rule4 = ctrl.Rule(self.E['lowneg'], (self.V['low'], self.W['lowder']))
        rule5 = ctrl.Rule(self.E['highneg'], (self.V['zero'], self.W['highder']))

        # ---------------- 4. SISTEMA DE INFERENCIA ----------------
        fuzzy_system = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])
        self.fuzzy_sim = ctrl.ControlSystemSimulation(fuzzy_system)

        # ---------------- PUBLISHERS Y SUBSCRIBERS ----------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_sub = self.create_subscription(Float32, 'line_error', self.error_callback, 10)
        
        self.get_logger().info("Controlador listo y esperando el tópico 'line_error'...")

    def error_callback(self, msg):
        error_val = msg.data
        
        # Limitar el error al universo de discurso normalizado [-1.0, 1.0]
        # Esto previene fallos matemáticos si el módulo de visión llegara a mandar un 1.05 por error de redondeo
        error_val = np.clip(error_val, -1.0, 1.0)

        # ---------------- DEFUZZIFICACIÓN Y EJECUCIÓN ----------------
        try:
            self.fuzzy_sim.input['Error'] = error_val
            self.fuzzy_sim.compute()
            
            v_out = self.fuzzy_sim.output['V']
            w_out = self.fuzzy_sim.output['W']
            
            twist = Twist()
            twist.linear.x = float(v_out)
            twist.angular.z = float(w_out)
            self.cmd_pub.publish(twist)
            
            # Loguear para depuración (puedes comentarlo si satura la terminal)
            self.get_logger().debug(f"Error Normalizado: {error_val:.3f} | V: {v_out:.3f}, W: {w_out:.3f}")
            
        except Exception as e:
            self.get_logger().error(f"Error en la inferencia difusa: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MamdaniControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()