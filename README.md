# fuzzy_control

Paquete de ROS 2 con varios controladores difusos para un robot seguidor de línea. Cada controlador recibe el error de posición respecto a la línea, calcula las velocidades lineal y angular, y las publica para mover el robot.

## Cómo funciona

El paquete expone cinco nodos, cada uno con un enfoque distinto para resolver el mismo problema (seguir la línea a partir del error). Todos comparten la misma interfaz de tópicos:

- **Suscripción:** `line_error` (`std_msgs/Float32`) — error normalizado en el rango `[-1.0, 1.0]`, donde `0` significa que el robot está centrado sobre la línea.
- **Publicación:** `/cmd_vel` (`geometry_msgs/Twist`) — velocidad lineal (`linear.x`) y angular (`angular.z`).

Las velocidades están acotadas a los límites del robot: lineal entre `0` y `0.2 m/s`, angular entre `-0.2` y `0.2 rad/s`.

## Nodos disponibles

| Ejecutable | Archivo | Enfoque |
|---|---|---|
| `mamdani_v1` | `line_mamdani_v1.py` | Mamdani con funciones de membresía Z/S/trapezoidales/triangulares ajustadas a mano. |
| `mamdani_v2` | `line_mamdani_v2.py` | Mamdani con membresías triangulares y trapezoidales más simétricas. |
| `mamdani_aprox` | `line_aproximated.py` | Aproximación no lineal sin lógica difusa: gaussiana para la velocidad lineal y `tanh` para la angular. |
| `sugeno_0` | `line_sugeno_0.py` | Sugeno de orden 0 (constantes de salida), 5 reglas, defuzzificación por promedio ponderado. |
| `sugeno_pd` | `line_sugeno_pd.py` | Sugeno tipo PD: usa el error y su derivada como entradas, con una matriz de 25 reglas. |

Todos usan las mismas cinco etiquetas para el error: `highneg`, `lowneg`, `zero`, `lowpos`, `highpos`.

## Dependencias

- ROS 2 (probado con la distribución sobre Python 3.10)
- `rclpy`, `std_msgs`, `geometry_msgs` (vienen con ROS 2)
- `numpy`
- `scikit-fuzzy` (`skfuzzy`) — requerido por todos los nodos excepto `mamdani_aprox`

Instalación de las dependencias de Python:

```bash
pip install numpy scikit-fuzzy
```

## Compilación

Clona el paquete dentro del directorio `src` de tu workspace de ROS 2 y compílalo con colcon:

```bash
cd ~/ros2_ws/src
git clone https://github.com/AndresMC2077/fuzzy_control.git
cd ~/ros2_ws
colcon build --packages-select fuzzy_control
source install/setup.bash
```

## Ejecución

Lanza el controlador que quieras probar (uno a la vez):

```bash
ros2 run fuzzy_control mamdani_v1
ros2 run fuzzy_control mamdani_v2
ros2 run fuzzy_control mamdani_aprox
ros2 run fuzzy_control sugeno_0
ros2 run fuzzy_control sugeno_pd
```

El nodo se queda esperando datos en el tópico `line_error`. Necesitas otra fuente (un nodo de visión, un simulador, etc.) que publique ahí el error de la línea. Para una prueba rápida puedes publicarlo a mano:

```bash
ros2 topic pub /line_error std_msgs/Float32 "{data: 0.3}"
```

Y verificar la salida:

```bash
ros2 topic echo /cmd_vel
```

## Estructura

```
fuzzy_control/
├── fuzzy_control/
│   ├── line_mamdani_v1.py
│   ├── line_mamdani_v2.py
│   ├── line_aproximated.py
│   ├── line_sugeno_0.py
│   └── line_sugeno_pd.py
├── test/
├── package.xml
├── setup.py
└── setup.cfg
```

## Licencia

Apache-2.0
