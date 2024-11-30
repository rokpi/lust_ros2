import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import MotionWrite    

class SubscribeSetMotion(Node):

    def __init__(self):
        super().__init__('subscribe_set_motion')
        self.subscription = self.create_subscription(
            MotionWrite, 
            'setMotionData', 
            self.listener_callback,
            10)     
        self.subscription     

    def listener_callback(self, msg):
            self.get_logger().info('I heard: "%d"' % msg.position) 


def main(args=None):
    rclpy.init(args=args)

    subscribe_set_motion = SubscribeSetMotion()

    rclpy.spin(subscribe_set_motion)

    subscribe_set_motion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()