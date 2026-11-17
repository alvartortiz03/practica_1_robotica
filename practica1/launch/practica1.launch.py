"""Launch que arranca el registrador cinemático y el ejecutor de trayectorias."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Devuelve los dos nodos de la práctica; la simulación se lanza aparte."""
    return LaunchDescription([
        Node(package='practica1', executable='kinematics_recorder', output='screen'),
        Node(package='practica1', executable='trajectory_runner', output='screen'),
    ])
