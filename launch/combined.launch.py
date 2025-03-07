from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_args():
    return [
        DeclareLaunchArgument(
            'chromium_path',
            description='The path to the chromium executable'),
        DeclareLaunchArgument(
            'signaler_port',
            default_value='8080',
            description='The port of the signaler server'),
        DeclareLaunchArgument(
            'id',
            default_value='source',
            description='The id of the camera rtc stream')
    ]


def generate_launch_description():
    chromium_path = LaunchConfiguration('chromium_path')
    signaler_port = LaunchConfiguration('signaler_port')
    id = LaunchConfiguration('id')

    run_signaler = ExecuteProcess(
        cmd=[
            'node', 'signaler/build/index.js', signaler_port
        ],
        output='screen',
        name='signaler'
    )

    run_source = ExecuteProcess(
        cmd=[
            'node', 'streamer/build/index.js',
            chromium_path, 'localhost', signaler_port, id
        ],
        output='screen',
        name=id
    )

    return LaunchDescription(
        [
            *generate_launch_args(),
            run_signaler,
            TimerAction(
                period=0.5,
                actions=[
                    run_source
                ]
            )
        ]
    )
