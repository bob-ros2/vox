# NOTE: This requires rclpy and std_msgs to be installed.
import logging
try:
    import rclpy
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

class OutputHandler:
    def __init__(self, **kwargs):
        if not ROS2_AVAILABLE:
            raise ImportError(
                "ROS2 libraries not found. The 'ros2' handler cannot be used.\n"
                "Please ensure rclpy is installed and your ROS2 environment is sourced "
                "(e.g., 'source /opt/ros/humble/setup.bash')."
            )
        rclpy.init()
        self.node = rclpy.create_node('vox_transcriber')
        self.publisher = self.node.create_publisher(String, 'transcription', 10)
        self.node.get_logger().info('ROS2 output handler initialized.')

    def send(self, text):
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.node.get_logger().info(f'Published: "{text}"')

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
