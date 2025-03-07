from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_args():
    return [
        DeclareLaunchArgument(
            'signaler_port',
            default_value='8080',
            description='The port of the signaler server'),
    ]


def generate_launch_description():
    signaler_port = LaunchConfiguration('signaler_port')

    run_signaler = ExecuteProcess(
        cmd=[
            'node', 'signaler/build/index.js', signaler_port
        ],
        output='screen',
        name='signaler'
    )

    return LaunchDescription(
        [
            *generate_launch_args(),
            run_signaler,
        ]
    )
