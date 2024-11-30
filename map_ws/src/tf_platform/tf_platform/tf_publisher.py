import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler

from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, Twist
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class PublishTF(Node):

    def __init__(self):
        super().__init__('publish_tf')
        
        # QoS profiles
        qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        qos_profile_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.publisher_tf = self.create_publisher(TFMessage, 'tf', qos_profile_reliable)
        self.publisher_clock = self.create_publisher(Clock, '/clock', qos_profile_reliable)
        self.publisher_tf_static = StaticTransformBroadcaster(self)
        self.publisher_odom = self.create_publisher(Odometry, '/odom', qos_profile_reliable)
        self.publisher_scan = self.create_publisher(LaserScan, '/scan', qos_profile_reliable)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', qos_profile_reliable)
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scanMerge',
            self.scan_callback,
            qos_profile_best_effort
        )
        
        self.latest_scan_msg = None
        self.timer_period = 0.1  # Update interval in seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.simulated_time = self.get_clock().now()

        self.broadcast_static_transforms()

    def broadcast_static_transforms(self):
        self.send_transform(-1.9996435284546594, -0.5000001131079675, 0.007832291129754549, 1.5707, 0, -1.5707, 'odom', 'base_link')
        self.send_transform(0.0, 0.0, 0.2, 0, 0, 0, 'base_link', 'base_scan')

    def send_transform(self, x, y, z, roll, pitch, yaw, parent_frame, child_frame):
        transform = TransformStamped()
        transform.header.stamp = self.simulated_time.to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        
        transform.transform.translation = Vector3(x=x, y=y, z=z)
        
        q = quaternion_from_euler(roll, pitch, yaw)
        transform.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        self.publisher_tf_static.sendTransform(transform)

    def scan_callback(self, msg):
        current_time = self.get_clock().now()
        msg.header.stamp = current_time.to_msg()
        self.latest_scan_msg = msg
        self.get_logger().info('Received LaserScan message')

    def timer_callback(self):
        self.simulated_time = self.get_clock().now()

        # Publish all necessary messages
        self.publish_messages()

        self.get_logger().info('Publishing transform: "%d"' % self.i)
        self.i += 1

    def publish_messages(self):
        # Publish clock
        clock_msg = Clock()
        clock_msg.clock = self.simulated_time.to_msg()
        self.publisher_clock.publish(clock_msg)

        # Publish tf
        self.send_transform(-1.9996435284546594, -0.5000001131079675, 0.007832291129754549, 1.5707, 0, -1.5707, 'odom', 'base_link')
        self.send_transform(0.0, 0.0, 0.2, 0, 0, 0, 'base_link', 'base_scan')

        # Publish odometry
        odom_msg = self.create_odom_msg()
        self.publisher_odom.publish(odom_msg)

        # Publish cmd_vel
        cmd_vel_msg = Twist()
        self.publisher_cmd_vel.publish(cmd_vel_msg)

        # Publish modified scan
        if self.latest_scan_msg:
            self.publisher_scan.publish(self.latest_scan_msg)
            self.get_logger().info('Published LaserScan message')
        else:
            self.get_logger().warn('No LaserScan message to publish')

    def create_odom_msg(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.simulated_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = -1.9996435284546594
        odom_msg.pose.pose.position.y = -0.5000001131079675
        odom_msg.pose.pose.position.z = 0.007832291129754549
        
        q = quaternion_from_euler(1.5707, 0, -1.5707)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        return odom_msg

def main(args=None):
    rclpy.init(args=args)
    publish_tf = PublishTF()
    rclpy.spin(publish_tf)
    publish_tf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
