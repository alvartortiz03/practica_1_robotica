"""Launch que arranca el registrador cinemático y el ejecutor de trayectorias."""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Devuelve los dos nodos de la práctica; la simulación se lanza aparte.

    Cuando el ejecutor termina la última trayectoria, se cierra todo el launch
    y con él el registrador, que ya ha guardado los CSV al recibir cada aviso
    de fin de trayectoria.
    """
    kinematics_recorder = Node(
        package='practica1',
        executable='kinematics_recorder',
        output='screen',
    )
    trajectory_runner = Node(
        package='practica1',
        executable='trajectory_runner',
        output='screen',
    )
    return LaunchDescription([
        kinematics_recorder,
        trajectory_runner,
        RegisterEventHandler(OnProcessExit(target_action=trajectory_runner, on_exit=[Shutdown()])),
    ])
