# NOTE: This requires rclpy and std_msgs to be installed.
import logging
import argparse
try:
    import rclpy
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

class OutputHandler:
    def __init__(self,  handler_args=None):
        parser = argparse.ArgumentParser()
        parser.add_argument('--ros2-topic', type=str, default='transcription',
            help='ROS2 topic to publish transcriptions to.')
        args, _ = parser.parse_known_args(handler_args)
       
        if not ROS2_AVAILABLE:
            raise ImportError(
                "ROS2 libraries not found. The 'ros2' handler cannot be used.\n"
                "Please ensure rclpy is installed and your ROS2 environment is sourced "
                "(e.g., 'source /opt/ros/humble/setup.bash')."
            )
        rclpy.init()
        self.node = rclpy.create_node('vox_transcriber')
        self.publisher = self.node.create_publisher(String, args.ros2_topic, 10)
        self.node.get_logger().info('ROS2 output handler initialized.')

    def send(self, text):
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.node.get_logger().info(f'Published: "{text}"')

    def close(self):
        # Give the publisher a moment to send the message before shutting down
        try:
            self.node.create_rate(0.5).sleep()
        except Exception as e:
            self.node.get_logger().warning(f"Error during shutdown sleep: {e}")
            
        self.node.destroy_node()
        rclpy.shutdown()
