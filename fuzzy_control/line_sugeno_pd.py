#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

import numpy as np
import skfuzzy as fuzz

class SugenoPDController(Node):
    def __init__(self):
        super().__init__('sugeno_pd_follower')
        self.get_logger().info("Iniciando Sugeno PD (Error + Derivada)...")

        # Variables de estado
        self.prev_error = 0.0

        # ---------------- 1. UNIVERSOS DE DISCURSO ----------------
        self.x_e  = np.arange(-1.0, 1.01, 0.01) # Error Normalizado
        self.x_de = np.arange(-1.0, 1.01, 0.01) # Cambio de Error (Derivada) Normalizado

        # ---------------- 2. FUNCIONES DE MEMBRESÍA ----------------
        # Etiquetas: HN (HighNeg), LN (LowNeg), Z (Zero), LP (LowPos), HP (HighPos)
        
        # Membresías para el ERROR (Posición)
        self.e_hn = fuzz.trapmf(self.x_e, [-1.05, -1.0, -0.5, -0.2])
        self.e_ln = fuzz.trimf(self.x_e, [-0.5, -0.2, 0.0])
        self.e_z  = fuzz.trimf(self.x_e, [-0.2, 0.0, 0.2])
        self.e_lp = fuzz.trimf(self.x_e, [0.0, 0.2, 0.5])
        self.e_hp = fuzz.trapmf(self.x_e, [0.2, 0.5, 1.0, 1.01])

        # Membresías para el CAMBIO DE ERROR (Velocidad de cruce)
        self.de_hn = fuzz.trapmf(self.x_de, [-1.05, -1.0, -0.5, -0.2])
        self.de_ln = fuzz.trimf(self.x_de, [-0.5, -0.2, 0.0])
        self.de_z  = fuzz.trimf(self.x_de, [-0.2, 0.0, 0.2])
        self.de_lp = fuzz.trimf(self.x_de, [0.0, 0.2, 0.5])
        self.de_hp = fuzz.trapmf(self.x_de, [0.2, 0.5, 1.0, 1.01])

        # ---------------- 3. LA MATRIZ DE REGLAS (AQUÍ PONES TUS CONSTANTES) ----------------
        # Formato: ('Etiqueta_Error', 'Etiqueta_Derivada') : (Velocidad_Lineal, Velocidad_Angular)
        self.reglas = {
            # --- ZONA: ERROR CERO (El robot está en el centro) ---
            ('z', 'z'):  (0.20, 0.0),    # Perfecto: En el centro y estable. Máxima velocidad.
            ('z', 'lp'): (0.18, 0.010),  # En el centro, pero moviéndose a la derecha: Pequeña corrección izq.
            ('z', 'hp'): (0.15, 0.15),  # En el centro, pero derrapando rápido a la derecha: Freno preventivo.
            ('z', 'ln'): (0.18, -0.10),   # En el centro, moviéndose a la izq.
            ('z', 'hn'): (0.15, -0.15),   # En el centro, derrapando rápido a la izq.

            # --- ZONA: ERROR LOW_POS (Ligeramente a la derecha) ---
            ('lp', 'z'):  (0.15, 0.13), # Desviado pero estable: Corrección normal.
            ('lp', 'lp'): (0.12, 0.20), # Desviado Y alejándose más: Corrección agresiva.
            ('lp', 'hp'): (0.05, 0.25), # PELIGRO: Alejándose rapidísimo. Freno casi total, giro máximo.
            ('lp', 'ln'): (0.18, 0.0),   # MAGIA DEL PD: Desviado, PERO ya se está acercando al centro. NO gires, déjate llevar.
            ('lp', 'hn'): (0.15, -0.10),  # Desviado, pero regresando DEMASIADO rápido: Frena el giro (contra-volante) para no pasarte.

            # --- ZONA: ERROR HIGH_POS (Muy a la derecha, casi fuera) ---
            ('hp', 'z'):  (0.05, 0.25), # Muy desviado: Frena y gira fuerte.
            ('hp', 'lp'): (0.0, 0.25),  # Muy desviado y alejándose: Freno de mano, giro máximo.
            ('hp', 'hp'): (0.0, 0.25),  # Peligro inminente.
            ('hp', 'ln'): (0.08, 0.15), # Muy desviado, pero ya empezó a regresar: Relaja el giro.
            ('hp', 'hn'): (0.10, 0.0),   # MAGIA: Muy desviado, pero regresando como bala. Pon la llanta recta.

            # --- ZONA: ERROR LOW_NEG (Ligeramente a la izquierda) ---
            ('ln', 'z'):  (0.15, -0.13),  # Es el espejo exacto de LowPos...
            ('ln', 'ln'): (0.12, -0.20),  
            ('ln', 'hn'): (0.05, -0.25),  
            ('ln', 'lp'): (0.18, 0.0),   
            ('ln', 'hp'): (0.15, 0.10), 

            # --- ZONA: ERROR HIGH_NEG (Muy a la izquierda) ---
            ('hn', 'z'):  (0.05, -0.25),  # Es el espejo exacto de HighPos...
            ('hn', 'ln'): (0.0, -0.25),   
            ('hn', 'hn'): (0.0, -0.25),   
            ('hn', 'lp'): (0.08, -0.15),  
            ('hn', 'hp'): (0.10, 0.0),   
        }

        # Nombres de las etiquetas para iterar fácilmente
        self.labels = ['hn', 'ln', 'z', 'lp', 'hp']
        self.mfs_e = [self.e_hn, self.e_ln, self.e_z, self.e_lp, self.e_hp]
        self.mfs_de = [self.de_hn, self.de_ln, self.de_z, self.de_lp, self.de_hp]

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_sub = self.create_subscription(Float32, 'line_error', self.error_callback, 10)

    def error_callback(self, msg):
        # 1. Obtener Error y calcular Derivada
        e = np.clip(msg.data, -1.0, 1.0)
        
        # Derivada cruda (diferencia entre cuadros). Multiplicamos por un factor (ej. 5.0) 
        # para que los cambios pequeños se amplifiquen y quepan en el universo [-1, 1]
        de_raw = (e - self.prev_error) * 3.0 
        de = np.clip(de_raw, -1.0, 1.0)
        
        self.prev_error = e # Guardar para el siguiente ciclo

        # 2. Fuzzificación: Obtener grados de pertenencia
        weights_e = {lbl: fuzz.interp_membership(self.x_e, mf, e) for lbl, mf in zip(self.labels, self.mfs_e)}
        weights_de = {lbl: fuzz.interp_membership(self.x_de, mf, de) for lbl, mf in zip(self.labels, self.mfs_de)}

        # 3. Inferencia y Defuzzificación Sugeno (Promedio Ponderado de 25 reglas)
        num_v = 0.0
        num_w = 0.0
        den = 0.0

        for l_e in self.labels:
            for l_de in self.labels:
                # El peso de la regla es el mínimo entre la certeza del Error y de la Derivada (AND difuso)
                w_rule = min(weights_e[l_e], weights_de[l_de])
                
                if w_rule > 0:
                    v_const, w_const = self.reglas[(l_e, l_de)]
                    num_v += w_rule * v_const
                    num_w += w_rule * w_const
                    den += w_rule

        # 4. Publicar
        if den > 0:
            twist = Twist()
            twist.linear.x = float(num_v / den)
            twist.angular.z = float(num_w / den)
            self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SugenoPDController())
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
