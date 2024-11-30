import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import AxisData   

class SubscribeAxisData(Node):

    def __init__(self):
        super().__init__('subscribe_axis_data')
        self.subscription = self.create_subscription(
            AxisData, 
            'axisData', 
            self.listener_callback,
            10)     
        self.subscription     

    def listener_callback(self, msg):
            self.get_logger().info('I heard: "%d"' % msg.velocity1) 


def main(args=None):
    rclpy.init(args=args)

    subscribe_axis_data = SubscribeAxisData()

    rclpy.spin(subscribe_axis_data)

    subscribe_axis_data.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()