import subprocess
import os
import rclpy


def main():
    rclpy.init()
    node = rclpy.create_node('cameras_bootstrap')

    # process = subprocess.Popen(
    #     ['node', 'index.js'],
    #     cwd=os.path.join(os.path.dirname(__file__), 'server/'),
    #     stdout=subprocess.PIPE,
    # )

    try:
        while rclpy.ok():
            # Have ROS log the stdout of the subprocess
            # stdout = process.stdout.readline()
            # if stdout:
            #     node.get_logger().info(stdout.decode('utf-8').strip())

            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        # process.terminate()
        node.destroy_node()
