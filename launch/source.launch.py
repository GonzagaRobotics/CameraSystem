from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_args():
    return [
        DeclareLaunchArgument(
            'chromium_path',
            description='The path to the chromium executable'),
        DeclareLaunchArgument(
            'signaler_hostname',
            default_value='localhost',
            description='The hostname of the signaler server'),
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
    signaler_hostname = LaunchConfiguration('signaler_hostname')
    signaler_port = LaunchConfiguration('signaler_port')
    id = LaunchConfiguration('id')

    run_source = ExecuteProcess(
        cmd=[
            'node', 'streamer/build/index.js',
            chromium_path, signaler_hostname, signaler_port, id
        ],
        output='screen',
        name=id
    )

    return LaunchDescription(
        [
            *generate_launch_args(),
            run_source,
        ]
    )
