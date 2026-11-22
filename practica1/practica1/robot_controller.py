#!/usr/bin/python3

import copy
import threading
import time
from typing import Any

from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningScene,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

SISTEMA_REFERENCIA = 'world'  # En ROS 2 el sistema de referencia raíz del robot es "world"
GRUPO = 'robot'
EFECTOR_FINAL = 'wrist_3_link'
ARTICULACIONES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                  'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
FACTOR_ESCALA = 0.1  # Velocidad y aceleración máximas
TAMAÑO_COLA = 10
PERIODO_ESPERA_ESTADO_SEG = 0.05
PERIODO_ESPERA_FUTURE_SEG = 0.01
INTENTOS_PLANIFICACION = 10
TIEMPO_PLANIFICACION_SEG = 5.0
TOLERANCIA_ARTICULAR_RAD = 0.001
TIEMPO_ESPERA_TF_SEG = 5.0
PASO_CARTESIANO_M = 0.01
UMBRAL_SALTO = 0.0
FRACCION_COMPLETA = 1.0
TAMAÑO_CAJA_M = (0.1, 0.1, 0.1)
ALTURA_SUELO_M = -0.026
TAMAÑO_SUELO_M = (2.0, 2.0, 0.05)


class ControlRobot(Node):
    """Control del robot con MoveIt 2 a través de sus acciones y servicios (move_group)."""

    def __init__(self, id_robot: str = '') -> None:
        """
        Crea el nodo y sus clientes, y espera a que MoveIt esté disponible.

        Parámetros:
            id_robot: "" para un robot individual, o "203", "204"... en las
                escenas con dos robots.
        """
        super().__init__('control_robot')
        if id_robot:
            self.group_name = f'robot_{id_robot}'
            self.efector_final = f'{id_robot}_{EFECTOR_FINAL}'
            self.articulaciones = [f'{id_robot}_{j}' for j in ARTICULACIONES]
        else:
            self.group_name = GRUPO
            self.efector_final = EFECTOR_FINAL
            self.articulaciones = list(ARTICULACIONES)
        self._estado_articulaciones: dict[str, float] = {}

        self.create_subscription(JointState, '/joint_states', self._cb_joint_states, TAMAÑO_COLA)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self.execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self.cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.scene_client = self.create_client(ApplyPlanningScene, 'apply_planning_scene')

        # Se atienden los callbacks (joint_states, tf, ...) en un hilo aparte
        self.ejecutor = MultiThreadedExecutor()
        self.ejecutor.add_node(self)
        threading.Thread(target=self.ejecutor.spin, daemon=True).start()

        self.get_logger().info('Esperando a MoveIt (move_group)...')
        self.move_group_client.wait_for_server()
        self.execute_client.wait_for_server()
        self.cartesian_client.wait_for_service()
        self.scene_client.wait_for_service()
        while not all(j in self._estado_articulaciones for j in self.articulaciones):
            time.sleep(PERIODO_ESPERA_ESTADO_SEG)

        self.añadir_suelo()

    def _cb_joint_states(self, msg: JointState) -> None:
        """Guarda la última posición recibida de cada articulación."""
        for nombre, posicion in zip(msg.name, msg.position):
            self._estado_articulaciones[nombre] = posicion

    @staticmethod
    def _esperar(future: Future) -> Any:
        """
        Bloquea hasta que el future termine y devuelve su resultado.

        Funciona porque el ejecutor gira en otro hilo; si no, se quedaría colgado.
        """
        while not future.done():
            time.sleep(PERIODO_ESPERA_FUTURE_SEG)
        return future.result()

    def _ir_a(self, restricciones: Constraints, wait: bool) -> bool:
        """
        Envía un objetivo a move_group para que planifique y ejecute.

        Parámetros:
            restricciones: objetivo (articular o de pose) a alcanzar.
            wait: si es True, espera a que termine el movimiento.

        Devuelve:
            True si el objetivo se aceptó (y, si se espera, si terminó con éxito).
        """
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = INTENTOS_PLANIFICACION
        goal.request.allowed_planning_time = TIEMPO_PLANIFICACION_SEG
        goal.request.max_velocity_scaling_factor = FACTOR_ESCALA
        goal.request.max_acceleration_scaling_factor = FACTOR_ESCALA
        goal.request.goal_constraints.append(restricciones)
        goal.request.start_state.is_diff = True
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = False

        goal_handle = self._esperar(self.move_group_client.send_goal_async(goal))
        if not goal_handle.accepted:
            return False
        if not wait:
            return True
        resultado = self._esperar(goal_handle.get_result_async()).result
        return resultado.error_code.val == MoveItErrorCodes.SUCCESS

    def mover_articulaciones(self, joint_goal: list[float], wait: bool = True) -> bool:
        """
        Movimiento Joint: lleva cada articulación al ángulo indicado.

        Parámetros:
            joint_goal: 6 ángulos en radianes, en el orden de ARTICULACIONES.
            wait: si es True, bloquea hasta que termine el movimiento.

        Devuelve:
            True si el movimiento tuvo éxito.
        """
        restricciones = Constraints()
        for nombre, valor in zip(self.articulaciones, joint_goal):
            restricciones.joint_constraints.append(JointConstraint(
                joint_name=nombre, position=float(valor),
                tolerance_above=TOLERANCIA_ARTICULAR_RAD,
                tolerance_below=TOLERANCIA_ARTICULAR_RAD, weight=1.0))
        return self._ir_a(restricciones, wait)

    def pose_actual(self) -> Pose:
        """Devuelve la pose actual del efector final respecto a SISTEMA_REFERENCIA."""
        transformacion = self.tf_buffer.lookup_transform(
            SISTEMA_REFERENCIA, self.efector_final, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=TIEMPO_ESPERA_TF_SEG))
        pose = Pose()
        pose.position.x = transformacion.transform.translation.x
        pose.position.y = transformacion.transform.translation.y
        pose.position.z = transformacion.transform.translation.z
        pose.orientation = transformacion.transform.rotation
        return pose

    def añadir_caja_a_escena_de_planificacion(
            self, pose_caja: Pose, name: str,
            tamaño: tuple[float, float, float] = TAMAÑO_CAJA_M) -> None:
        """
        Añade una caja como obstáculo a la escena de planificación.

        Parámetros:
            pose_caja: pose del centro de la caja.
            name: identificador único del objeto.
            tamaño: dimensiones (x, y, z) en metros.
        """
        caja = CollisionObject()
        caja.id = name
        caja.header.frame_id = SISTEMA_REFERENCIA
        caja.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(tamaño)))
        caja.primitive_poses.append(pose_caja)
        caja.operation = CollisionObject.ADD

        peticion = ApplyPlanningScene.Request()
        peticion.scene = PlanningScene()
        peticion.scene.is_diff = True
        peticion.scene.robot_state = RobotState(is_diff=True)
        peticion.scene.world.collision_objects.append(caja)
        self._esperar(self.scene_client.call_async(peticion))

    def mover_trayectoria(self, poses: list[Pose], wait: bool = True) -> bool:
        """
        Movimiento cartesiano: la punta va en línea recta por cada pose.

        Parámetros:
            poses: poses intermedias por las que debe pasar el efector final.
            wait: si es True, bloquea hasta que termine el movimiento.

        Devuelve:
            False: si no se puede calcular el camino completo o falla la ejecución.
        """
        poses_aux = copy.deepcopy(poses)
        poses_aux.insert(0, self.pose_actual())

        peticion = GetCartesianPath.Request()
        peticion.header.frame_id = SISTEMA_REFERENCIA
        peticion.start_state.is_diff = True
        peticion.group_name = self.group_name
        peticion.link_name = self.efector_final
        peticion.waypoints = poses_aux
        peticion.max_step = PASO_CARTESIANO_M
        peticion.jump_threshold = UMBRAL_SALTO
        peticion.avoid_collisions = True
        peticion.max_velocity_scaling_factor = FACTOR_ESCALA
        peticion.max_acceleration_scaling_factor = FACTOR_ESCALA
        respuesta = self._esperar(self.cartesian_client.call_async(peticion))

        if respuesta.fraction != FRACCION_COMPLETA:
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = respuesta.solution
        goal_handle = self._esperar(self.execute_client.send_goal_async(goal))
        if not goal_handle.accepted:
            return False
        if not wait:
            return True
        resultado = self._esperar(goal_handle.get_result_async()).result
        return resultado.error_code.val == MoveItErrorCodes.SUCCESS

    def añadir_suelo(self) -> None:
        """Añade el suelo a la escena para que el planificador lo evite."""
        pose_suelo = Pose()
        pose_suelo.position.z = ALTURA_SUELO_M
        pose_suelo.orientation.w = 1.0
        self.añadir_caja_a_escena_de_planificacion(pose_suelo, 'suelo', TAMAÑO_SUELO_M)
