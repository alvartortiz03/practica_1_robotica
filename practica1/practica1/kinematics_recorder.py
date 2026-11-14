#!/usr/bin/python3
"""
Nodo de ROS 2 que registra la cinemática del brazo UR3e por trayectorias.

Se suscribe a /joint_states y guarda la posición y la velocidad de las seis
articulaciones del brazo, ordenadas de la base a la muñeca, junto con el
instante en que se tomó la medida. El mensaje no trae la aceleración, así que
se calcula por diferencias finitas entre dos mensajes consecutivos:
a = (v_actual - v_anterior) / (t_actual - t_anterior). Cada PRINT_PERIOD_SEC
segundos muestra en el log el último estado calculado.

Además escucha los avisos del ejecutor de trayectorias (ver trajectory_contract):
mientras hay una trayectoria en curso acumula sus muestras y, cuando termina,
las guarda en un CSV propio dentro de la carpeta data/.

En /joint_states publican dos nodos: joint_state_broadcaster (el brazo, a
unos 500 Hz) y rg2_control_node (la pinza). Los mensajes de la pinza se
descartan y, como los nombres llegan en orden alfabético, cada valor se
busca por el nombre de su articulación.
"""

import csv
from dataclasses import dataclass
import datetime
from pathlib import Path

from practica1.trajectory_contract import NO_TRAJECTORY, TRAJECTORY_QOS, TRAJECTORY_TOPIC
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


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
OUTPUT_DIR = 'data'


@dataclass(frozen=True)
class KinematicSample:
    """Estado del brazo en un instante: tiempo, posición, velocidad y aceleración."""

    time_sec: float
    positions: list[float]
    velocities: list[float]
    accelerations: list[float]


class KinematicsRecorder(Node):
    """
    Nodo que registra posición, velocidad y aceleración de las articulaciones.

    Conserva el último estado y el anterior, que hace falta para calcular la
    aceleración, y acumula las muestras de la trayectoria en curso para
    guardarlas en un CSV cuando termina. Un temporizador muestra el último
    estado en el log cada PRINT_PERIOD_SEC segundos para no saturar la salida.
    """

    def __init__(self) -> None:
        """Inicializa el estado del nodo, sus dos suscripciones y el temporizador."""
        super().__init__(NODE_NAME)
        self.positions: list[float] | None = None
        self.velocities: list[float] | None = None
        self.prev_velocities: list[float] | None = None
        self.accelerations: list[float] | None = None
        self.stamp_sec: float | None = None
        self.prev_stamp_sec: float | None = None
        self.output_dir: Path = Path(OUTPUT_DIR)
        self.samples: list[KinematicSample] = []
        self.current_trajectory: str | None = None
        self.create_subscription(
            JointState,
            JOINT_STATES_TOPIC,
            self._on_joint_state,
            QUEUE_SIZE
        )
        self.create_subscription(
            String,
            TRAJECTORY_TOPIC,
            self._on_trajectory_change,
            TRAJECTORY_QOS
        )
        self.create_timer(PRINT_PERIOD_SEC, self._log_latest_state)

    def _on_joint_state(self, estado: JointState) -> None:
        """
        Guarda el estado de un mensaje del brazo y calcula su aceleración.

        Los mensajes de la pinza se descartan. Los valores se reordenan según
        ARM_JOINT_NAMES, porque el mensaje trae los nombres en orden alfabético.
        Antes de sobrescribirlos, la velocidad y el instante actuales pasan a
        ser los anteriores, que son los que usa el cálculo de la aceleración.

        Parameters
        ----------
        estado : JointState
            Mensaje recibido en /joint_states.

        """
        if not set(ARM_JOINT_NAMES).issubset(estado.name):
            return
        name_to_index = {joint: index for index, joint in enumerate(estado.name)}
        self.positions = [estado.position[name_to_index[joint]] for joint in ARM_JOINT_NAMES]
        self.prev_velocities = self.velocities
        self.velocities = [estado.velocity[name_to_index[joint]] for joint in ARM_JOINT_NAMES]
        stamp = estado.header.stamp
        self.prev_stamp_sec = self.stamp_sec
        self.stamp_sec = stamp.sec + stamp.nanosec / NANOSECONDS_PER_SECOND
        self._compute_accelerations()
        if self.current_trajectory is not None and self.accelerations is not None:
            sample = KinematicSample(
                time_sec=self.stamp_sec,
                positions=self.positions,
                velocities=self.velocities,
                accelerations=self.accelerations,
            )
            self.samples.append(sample)

    def _compute_accelerations(self) -> None:
        """Calcula la aceleración de cada articulación con las dos últimas muestras."""
        if self.prev_stamp_sec is not None and self.stamp_sec - self.prev_stamp_sec > 0:
            diff_time = self.stamp_sec - self.prev_stamp_sec
            self.accelerations = [
                (v - pv) / diff_time
                for v, pv in zip(
                    self.velocities,
                    self.prev_velocities,
                )
            ]
        else:
            self.accelerations = None

    def _on_trajectory_change(self, mensaje: String) -> None:
        """
        Atiende un aviso del ejecutor de trayectorias.

        Un nombre de trayectoria indica que empieza a ejecutarse y NO_TRAJECTORY
        que ha terminado. Antes de cambiar de estado se guarda en CSV la
        trayectoria que estaba en curso, y la lista de muestras se vacía para
        que no se mezclen dos trayectorias. Un aviso que no cambia el estado
        se ignora.

        Parameters
        ----------
        mensaje : String
            Aviso recibido en TRAJECTORY_TOPIC.

        """
        if mensaje.data == NO_TRAJECTORY:
            new_trajectory = None
        else:
            new_trajectory = mensaje.data

        if new_trajectory == self.current_trajectory:
            return

        if self.current_trajectory is not None:
            self.get_logger().info(
                f'Se finalizó la trayectoria {self.current_trajectory}'
            )
            self._save_trajectory()
        self.current_trajectory = new_trajectory
        self.samples = []
        if new_trajectory is not None:
            self.get_logger().info(
                f'Se inicia una nueva trayectoria {new_trajectory}'
            )

    def _save_trajectory(self) -> None:
        """
        Guarda en un CSV las muestras de la trayectoria en curso.

        El fichero se crea en la carpeta data/ con el nombre de la trayectoria
        y la fecha y hora, para no sobrescribir ejecuciones anteriores. Tiene
        una fila por muestra: el instante de la medida y, para cada
        articulación, su posición, su velocidad y su aceleración. Si no hay
        muestras, no se crea ningún fichero.
        """
        if not self.samples:
            self.get_logger().info(
                f'No hay nada que guardar de la trayectoria {self.current_trajectory}'
            )
            return
        self.output_dir.mkdir(parents=True, exist_ok=True)
        ruta = self.output_dir / (
            self.current_trajectory +
            '_' +
            datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S') +
            '.csv'
        )
        header = (
            ['time_sec']
            + [f'pos_{joint}' for joint in ARM_JOINT_NAMES]
            + [f'vel_{joint}' for joint in ARM_JOINT_NAMES]
            + [f'acc_{joint}' for joint in ARM_JOINT_NAMES]
        )
        filas = [
            [s.time_sec, *s.positions, *s.velocities, *s.accelerations]
            for s in self.samples
        ]
        with open(ruta, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(filas)
        self.get_logger().info(
            f'Se ha guardado la trayectoria {self.current_trajectory} en {ruta.resolve()} y '
            f'tiene {len(filas)} observaciones'
        )

    def _log_latest_state(self) -> None:
        """Muestra en el log el último estado recibido, si ya ha llegado alguno."""
        if self.accelerations is not None:
            self.get_logger().info(
                f'Time: {self.stamp_sec:.3f}'
            )
            for joint, position, velocity, acceleration in zip(
                ARM_JOINT_NAMES,
                self.positions,
                self.velocities,
                self.accelerations,
            ):
                self.get_logger().info(
                    f'Joint: {joint}, '
                    f'Position: {position:.3f}, '
                    f'Velocity: {velocity:.3f}, '
                    f'Acceleration: {acceleration:.3f}'
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
