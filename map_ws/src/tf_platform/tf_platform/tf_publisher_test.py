import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import  Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler

class PublishTFCloud(Node):

    def __init__(self):
        super().__init__('tf_publisher_cloud')
        
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

        self.publisher_tf_static = StaticTransformBroadcaster(self)
        self.publisher_odom = self.create_publisher(Odometry, '/odom', qos_profile_reliable)
        
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/cloud_in',
            self.pointcloud_callback,
            qos_profile_best_effort
        )
        
        self.position_x = 0.0
        self.position_z = 0.0
        self.position_y = 0.0

        self.latest_pointcloud_msg = None
        self.simulated_time = self.get_clock().now()

    '''def send_transform(self, x, y, z, roll, pitch, yaw, parent_frame, child_frame):
        transform = TransformStamped()
        transform.header.stamp = self.simulated_time.to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        
        transform.transform.translation = Vector3(x=x, y=y, z=z)
        
        q = quaternion_from_euler(roll, pitch, yaw)
        transform.transform.rotation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
        
        self.publisher_tf_static.sendTransform(transform)'''

    def pointcloud_callback(self, msg):
        # Publish all necessary messages
        self.publish_messages(msg)        

    def publish_messages(self,msg):
        # Publish trasformacije med 'map' 'odom' in base_link', 'laser'
        #self.send_transform(0.0, 0.0, 0.0, 0, 0, 0, 'map', 'odom')
        #self.send_transform(0.0, 0.0, 0.2, 0, 0, 0, 'base_link', 'laser')

        # Publish odometry
        odom_msg = self.create_odom_msg(msg)
        self.publisher_odom.publish(odom_msg)

    def create_odom_msg(self, msg):
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.position_x
        odom_msg.pose.pose.position.y = self.position_y
        odom_msg.pose.pose.position.z = self.position_z
        
        q = quaternion_from_euler(0, 0, 0)
        odom_msg.pose.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])

        return odom_msg

def main(args=None):
    rclpy.init(args=args)
    publish_tf_cloud = PublishTFCloud()
    rclpy.spin(publish_tf_cloud)
    publish_tf_cloud.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

