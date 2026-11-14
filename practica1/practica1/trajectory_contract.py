"""
Contrato entre el ejecutor de trayectorias y el registrador cinemático.

El ejecutor publica en TRAJECTORY_TOPIC el nombre de la trayectoria
(JOINT_TRAJECTORY, LINEAR_TRAJECTORY o CIRCULAR_TRAJECTORY) justo antes de
ejecutarla, y NO_TRAJECTORY en cuanto termina. El registrador se suscribe a
ese topic para separar las muestras de cada trayectoria.

Los dos nodos deben usar TRAJECTORY_QOS. Su durabilidad TRANSIENT_LOCAL hace
que el último aviso llegue también a un nodo que se suscriba más tarde; con
perfiles distintos, los nodos podrían no llegar a conectarse.
"""

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

TRAJECTORY_TOPIC = '/current_trajectory'
TRAJECTORY_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)
NO_TRAJECTORY = ''
JOINT_TRAJECTORY = 'joint'
LINEAR_TRAJECTORY = 'linear'
CIRCULAR_TRAJECTORY = 'circular'
