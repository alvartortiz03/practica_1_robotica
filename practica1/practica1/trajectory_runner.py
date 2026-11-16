#!/usr/bin/python3
"""
Nodo ejecutor de trayectorias del brazo UR3e.

Mueve el robot con ControlRobot y avisa al registrador cinemático de qué
trayectoria se está ejecutando: publica su nombre
justo antes de empezarla y NO_TRAJECTORY en cuanto termina, también en caso de que
movimiento fallara, para que el registrador cierre siempre la trayectoria.
"""

from collections.abc import Callable
import copy
import math
import time

from practica1.robot_controller import ControlRobot
from practica1.trajectory_contract import (
    JOINT_TRAJECTORY,
    LINEAR_TRAJECTORY,
    NO_TRAJECTORY,
    TRAJECTORY_QOS,
    TRAJECTORY_TOPIC,
)
import rclpy
from std_msgs.msg import String

ESPERA_REGISTRADOR_SEG = 5.0
PERIODO_COMPROBACION_SEG = 0.1
PAUSA_CIERRE_SEG = 0.5
POSTURA_INICIO = [0.0, -math.pi / 2, -math.pi / 2, -math.pi / 2, math.pi / 2, 0.0]
POSTURA_DESTINO_JOINT = [1.0, -1.2, -math.pi / 2, -math.pi / 2, math.pi / 2, 0.5]
DISTANCIA_LINEAL_M = 0.10


class EjecutorTrayectorias:
    """
    Ejecuta trayectorias con el robot y avisa si está en curso.

    Tiene un ControlRobot, que es el nodo que mueve el robot, y crea sobre él
    el publicador de avisos del contrato. Cada trayectoria se lanza con
    _ejecutar_trayectoria, que garantiza un aviso de inicio a fin.
    """

    def __init__(self) -> None:
        """Crea el control del robot y, sobre su nodo, el publicador de avisos."""
        self._robot = ControlRobot()
        self._publicador = self._robot.create_publisher(String, TRAJECTORY_TOPIC, TRAJECTORY_QOS)
        self._log = self._robot.get_logger()

    def _avisar(self, trayectoria: str) -> None:
        """Publica el nombre de la trayectoria en curso, o NO_TRAJECTORY si ha terminado."""
        self._publicador.publish(String(data=trayectoria))

    def _esperar_registrador(self) -> None:
        """
        Espera a que el registrador esté suscrito al topic de avisos.

        Sin esta espera, el registrador podría perder las primeras muestras de
        la primera trayectoria. Si no aparece en ESPERA_REGISTRADOR_SEG segundos,
        se avisa en el log y se continúa.
        """
        limite = time.time() + ESPERA_REGISTRADOR_SEG
        while self._publicador.get_subscription_count() == 0:
            if time.time() > limite:
                self._log.warning(
                    f'Ningún registrador escucha {TRAJECTORY_TOPIC}; se continúa sin él'
                )
                return
            time.sleep(PERIODO_COMPROBACION_SEG)

    def _ejecutar_trayectoria(self, nombre: str, movimiento: Callable[[], bool]) -> bool:
        """
        Avisa del inicio de una trayectoria, la ejecuta y avisa del final.

        El aviso de fin va en un finally para que se publique siempre, también
        si el movimiento falla o lanza una excepción. Así el registrador cierra
        la trayectoria y no mezcla sus muestras con las de la siguiente.

        Parámetros:
            nombre: nombre de la trayectoria, uno de los definidos en el contrato.
            movimiento: función sin parámetros que mueve el robot y devuelve si
                el movimiento tuvo éxito.

        Devuelve:
            True si el movimiento tuvo éxito.
        """
        self._log.info(f'Empieza la trayectoria {nombre}')
        self._avisar(nombre)
        exito = False
        try:
            exito = movimiento()
            if not exito:
                self._log.error(f'La trayectoria {nombre} ha fallado')
        finally:
            self._avisar(NO_TRAJECTORY)
        return exito

    def _trayectoria_joint(self) -> None:
        """
        Ejecuta la trayectoria Joint: de POSTURA_INICIO a POSTURA_DESTINO_JOINT.

        El robot va primero a la postura de inicio sin avisar, para que esa
        aproximación no se grabe. Solo el movimiento hasta el destino se
        ejecuta con aviso, y es el que guarda el registrador.
        """
        if not self._robot.mover_articulaciones(POSTURA_INICIO):
            self._log.error('No se ha podido llevar el robot a la postura de inicio')
            return
        # lambda retrasa el movimiento hasta que _ejecutar_trayectoria haya avisado
        self._ejecutar_trayectoria(
            JOINT_TRAJECTORY,
            lambda: self._robot.mover_articulaciones(POSTURA_DESTINO_JOINT),
        )

    def _trayectoria_lineal(self) -> None:
        """
        Ejecuta la trayectoria lineal: baja la punta del robot en línea 
        recta sin cambiar su inclinación.

        Primero lleva el robot a POSTURA_INICIO en silencio. A partir de 
        la pose actual del extremo, calcula un nuevo punto justo debajo reduciendo 
        su altura, Z, de modo que el extremo solo se desplaza en vertical.
        """
        if not self._robot.mover_articulaciones(POSTURA_INICIO):
            self._log.error('No se ha podido llevar el robot a la postura de inicio')
            return
        # Se copia la pose para no modificar la que guarda ControlRobot
        destino = copy.deepcopy(self._robot.pose_actual())
        destino.position.z -= DISTANCIA_LINEAL_M
        self._ejecutar_trayectoria(
            LINEAR_TRAJECTORY,
            lambda: self._robot.mover_trayectoria([destino]),
        )

    def ejecutar(self) -> None:
        """Ejecuta todas las trayectorias, una detrás de otra."""
        self._esperar_registrador()
        self._trayectoria_joint()
        self._trayectoria_lineal()

    def cerrar(self) -> None:
        """Deja salir el último aviso y cierra el nodo del robot."""
        time.sleep(PAUSA_CIERRE_SEG)
        self._robot.ejecutor.shutdown()
        self._robot.destroy_node()


def main(args: list[str] | None = None) -> None:
    """
    Arranca ROS, ejecuta las trayectorias y cierra.

    Parámetros:
        args: argumentos de línea de comandos para ROS. Si es None, se usan los del proceso.
    """
    rclpy.init(args=args)
    ejecutor = EjecutorTrayectorias()
    try:
        ejecutor.ejecutar()
    finally:
        # Se cierra también si algo falla a mitad, para no dejar el nodo colgado
        ejecutor.cerrar()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
