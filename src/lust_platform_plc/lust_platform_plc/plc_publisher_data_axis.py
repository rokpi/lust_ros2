import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import AxisData    

class PublishAxisData(Node):

    def __init__(self):
        super().__init__('publish_axis_data')
        self.publisher_ = self.create_publisher(AxisData, 'axisData', 10)     
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        msg = AxisData()
        msg.velocity1 = self.i
        msg.velocity2 = 1
        msg.error1 = 1
        msg.error2 = 1              
        self.publisher_.publish(msg)
        
        self.get_logger().info('Publishing position: "%d"' % msg.velocity1)  
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    publish_axis_data = PublishAxisData()

    rclpy.spin(publish_axis_data)

    publish_axis_data.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

