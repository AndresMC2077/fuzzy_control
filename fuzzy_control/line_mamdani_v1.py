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
        super().__init__('mamdani_line_follower_v1')
        
        self.get_logger().info("Inicializando Controlador Difuso Mamdani (Entrada Normalizada)...")

        # Universo de discurso
        # Entrada: Error Normalizado [-1.0, 1.0]
        self.E = ctrl.Antecedent(np.arange(-1.0, 1.01, 0.01), 'Error')
        
        # Salidas: Velocidad Lineal (V) en m/s y Angular (W) en rad/s
        self.V = ctrl.Consequent(np.arange(0.0, 0.21, 0.001), 'V')
        self.W = ctrl.Consequent(np.arange(-0.2, 0.21, 0.001), 'W')

        # Funciones de membresía   
        self.E['highneg'] = fuzz.zmf(self.E.universe,-0.9,-0.6) #-0.7,-0.45
        self.E['lowneg']  = fuzz.trapmf(self.E.universe, [-0.65, -0.4, -0.2, 0.05])
        self.E['zero']    = fuzz.trimf(self.E.universe, [-0.15, 0.0, 0.15])
        self.E['lowpos']  = fuzz.trapmf(self.E.universe, [0.05, 0.2, 0.4,0.65])
        self.E['highpos'] = fuzz.smf(self.E.universe,0.6,0.9)

        #Salidas de velocidad lineal y angular
        self.V['zero'] = fuzz.zmf(self.V.universe, 0.02, 0.06)
        self.V['low']  = fuzz.trimf(self.V.universe, [0.045, 0.085, 0.125])
        self.V['high'] = fuzz.smf(self.V.universe, 0.11, 0.15)

        self.W['highder'] = fuzz.zmf(self.W.universe,-0.18,-0.1)
        self.W['lowder']  = fuzz.trapmf(self.W.universe, [-0.135,-0.095,-0.055,-0.04])
        self.W['zero']    = fuzz.trimf(self.W.universe, [-0.05,0,0.05])
        self.W['lowizq']  = fuzz.trapmf(self.W.universe, [0.04,0.055,0.095,0.135])
        self.W['highizq'] = fuzz.smf(self.W.universe, 0.1,0.18)

        # Reglas
        rule1 = ctrl.Rule(self.E['zero'], (self.V['high'], self.W['zero']))
        rule2 = ctrl.Rule(self.E['lowpos'], (self.V['low'], self.W['lowizq']))
        rule3 = ctrl.Rule(self.E['highpos'], (self.V['zero'], self.W['highizq']))
        rule4 = ctrl.Rule(self.E['lowneg'], (self.V['low'], self.W['lowder']))
        rule5 = ctrl.Rule(self.E['highneg'], (self.V['zero'], self.W['highder']))

        # Motor de inferencia
        fuzzy_system = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])
        self.fuzzy_sim = ctrl.ControlSystemSimulation(fuzzy_system)

        # publishers y subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_sub = self.create_subscription(Float32, '/line_error', self.error_callback, 10)
        
        self.get_logger().info("Controlador listo, hola")

    def error_callback(self, msg):
        error_val = msg.data
        
        # Limitar el error por si hay errores de redondeo
        error_val = np.clip(error_val, -1.0, 1.0)

        # Defuzzificación y ejecución
        try:
            self.fuzzy_sim.input['Error'] = error_val
            self.fuzzy_sim.compute()
            
            v_out = self.fuzzy_sim.output['V']
            w_out = self.fuzzy_sim.output['W']
            
            twist = Twist()
            twist.linear.x = float(v_out)
            twist.angular.z = float(w_out)
            self.cmd_pub.publish(twist)
            
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
