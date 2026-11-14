#!/usr/bin/python3
"""
Nodo de ROS 2 que lee el estado de las articulaciones del brazo UR3e.

Se suscribe a /joint_states y guarda la posición y la velocidad de las seis
articulaciones del brazo, ordenadas de la base a la muñeca, junto con el
instante en que se tomó la medida. Cada PRINT_PERIOD_SEC segundos muestra
en el log el último estado recibido.

En /joint_states publican dos nodos: joint_state_broadcaster (el brazo, a
unos 500 Hz) y rg2_control_node (la pinza). Los mensajes de la pinza se
descartan y, como los nombres llegan en orden alfabético, cada valor se
busca por el nombre de su articulación.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

NODE_NAME = 'kinematics_recorder'
JOINT_STATES_TOPIC = '/joint_states'
QUEUE_SIZE = 10
ARM_JOINT_NAMES = (
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
)
PRINT_PERIOD_SEC = 1.0
NANOSECONDS_PER_SECOND = 1e9


class KinematicsRecorder(Node):
    """
    Nodo que registra la posición y la velocidad de las articulaciones del brazo.

    Solo conserva el último estado recibido. Un temporizador lo muestra en el
    log cada PRINT_PERIOD_SEC segundos para no saturar la salida.
    """

    def __init__(self) -> None:
        """Crea la suscripción a /joint_states y el temporizador del log."""
        super().__init__(NODE_NAME)
        self.positions: list[float] | None = None
        self.velocities: list[float] | None = None
        self.stamp_sec: float | None = None
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joint_state, QUEUE_SIZE)
        self.create_timer(PRINT_PERIOD_SEC, self._log_latest_state)

    def _on_joint_state(self, estado: JointState) -> None:
        """
        Guarda la posición, la velocidad y el instante de un mensaje del brazo.

        Los mensajes de la pinza se descartan. Los valores se reordenan según
        ARM_JOINT_NAMES, porque el mensaje trae los nombres en orden alfabético.

        Parameters
        ----------
        estado : JointState
            Mensaje recibido en /joint_states.

        """
        if not set(ARM_JOINT_NAMES).issubset(estado.name):
            return
        name_to_index = {joint: index for index, joint in enumerate(estado.name)}
        self.positions = [estado.position[name_to_index[joint]] for joint in ARM_JOINT_NAMES]
        self.velocities = [estado.velocity[name_to_index[joint]] for joint in ARM_JOINT_NAMES]
        stamp = estado.header.stamp
        self.stamp_sec = stamp.sec + stamp.nanosec / NANOSECONDS_PER_SECOND

    def _log_latest_state(self) -> None:
        """Muestra en el log el último estado recibido, si ya ha llegado alguno."""
        if self.positions is not None:
            self.get_logger().info(
                f'Time: {self.stamp_sec:.3f}'
            )
            for joint, position, velocity in zip(ARM_JOINT_NAMES, self.positions, self.velocities):
                self.get_logger().info(
                    f'Joint: {joint}, Position: {position:.3f}, Velocity: {velocity:.3f}'
                )


def main(args: list[str] | None = None) -> None:
    """
    Arranca el nodo y lo mantiene activo hasta que se pulse Ctrl+C.

    Parameters
    ----------
    args : list[str] | None
        Argumentos de línea de comandos para ROS. Si es None, se usan los
        del proceso.

    """
    rclpy.init(args=args)
    nodo = KinematicsRecorder()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
