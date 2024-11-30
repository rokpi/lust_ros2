import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler

from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, Twist
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from builtin_interfaces.msg import Time
import time

class PublishTF(Node):

    def __init__(self):
        super().__init__('publish_tf')
        
        # QoS profile for /tf_static
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=100  # Increase the buffer size
        )
        
        # QoS profile for /scan
        qos_profile1 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Increase the buffer size
        )
        
        self.publisher_tf = self.create_publisher(TFMessage, 'tf', qos_profile)
        self.publisher_clock = self.create_publisher(Clock, '/clock', qos_profile)
        self.publisher_tf_static = StaticTransformBroadcaster(self)
        self.publisher_odom = self.create_publisher(Odometry, '/odom', qos_profile)
        self.publisher_scan = self.create_publisher(LaserScan, '/scan', qos_profile1)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scanMerge',
            self.scan_callback,
            qos_profile1)
        
        self.latest_scan_msg = None
        self.scan_time = None
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.broadcast_static_transforms()

    def broadcast_static_transforms(self):
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'odom'
        static_transform.child_frame_id = 'base_link'
        
        static_transform.transform.translation = Vector3(
            x=-1.9996435284546594,
            y=-0.5000001131079675,
            z=0.007832291129754549
        )
        
        q = quaternion_from_euler(1.5707, 0, -1.5707)
        static_transform.transform.rotation = Quaternion(
            x=q[0],
            y=q[1],
            z=q[2],
            w=q[3]
        )
        
        self.publisher_tf_static.sendTransform(static_transform)

    def scan_callback(self, msg):
        self.latest_scan_msg = msg
        self.scan_time = msg.header.stamp

    def timer_callback(self):
        if self.scan_time is None:
            self.get_logger().info('Waiting for first scan message...')
            return

        # Add a small delay to ensure transform data is available
        time.sleep(0.1)

        q = quaternion_from_euler(1.5707, 0, -1.5707)
        
        # TF Message
        transform = TransformStamped()
        transform.header.stamp = self.scan_time
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        transform.transform.translation = Vector3(
            x=-1.9996435284546594,
            y=-0.5000001131079675,
            z=0.007832291129754549
        )
        
        transform.transform.rotation = Quaternion(
            x=q[0],
            y=q[1],
            z=q[2],
            w=q[3]
        )
        
        tf_msg = TFMessage()
        tf_msg.transforms.append(transform)
        
        self.publisher_tf.publish(tf_msg)
        
        # Clock Message
        clock_msg = Clock()
        clock_msg.clock = self.scan_time
        self.publisher_clock.publish(clock_msg)
        
        # TF Static Message
        self.publisher_tf_static.sendTransform(transform)
        
        # Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.scan_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = transform.transform.translation.x
        odom_msg.pose.pose.position.y = transform.transform.translation.y
        odom_msg.pose.pose.position.z = transform.transform.translation.z
        
        odom_msg.pose.pose.orientation.x = transform.transform.rotation.x
        odom_msg.pose.pose.orientation.y = transform.transform.rotation.y
        odom_msg.pose.pose.orientation.z = transform.transform.rotation.z
        odom_msg.pose.pose.orientation.w = transform.transform.rotation.w
        
        self.publisher_odom.publish(odom_msg)

        # Publish the scan message to /scan
        if self.latest_scan_msg:
            self.publisher_scan.publish(self.latest_scan_msg)
        
        # Publish a dummy Twist message to /cmd_vel
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.linear.z = 0.0
        cmd_vel_msg.angular.x = 0.0
        cmd_vel_msg.angular.y = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.publisher_cmd_vel.publish(cmd_vel_msg)

        self.get_logger().info('Publishing transform: "%d"' % self.i)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    publish_tf = PublishTF()

    rclpy.spin(publish_tf)

    publish_tf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

