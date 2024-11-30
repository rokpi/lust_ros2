import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import MotionWrite    

class PublishSetMotion(Node):

    def __init__(self):
        super().__init__('publish_set_motion')
        self.publisher_ = self.create_publisher(MotionWrite, 'setMotionData', 10)     
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        msg = MotionWrite()
        msg.stop1 = 1
        msg.stop2 = 1
        msg.position = self.i
        msg.ref_velocity = 100
        msg.per_rad_velocity = 0.5
        msg.per_forward_velocity = 0.5              
        self.publisher_.publish(msg)
        
        self.get_logger().info('Publishing position: "%d"' % msg.position)  
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    publish_set_motion = PublishSetMotion()

    rclpy.spin(publish_set_motion)

    publish_set_motion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

