
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs_py.point_cloud2 import read_points_numpy
from sensor_msgs_py import point_cloud2

from std_msgs.msg import Int8

from sensor_msgs.msg import PointCloud2



class SubscribeScanner(Node):

    def __init__(self):
        super().__init__('subscribe_scanner')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth= 1
        )

        self.subscription = self.create_subscription(
            PointCloud2, 
            'cloud_in', 
            self.listener_callback,
            qos_profile)     
        self.subscription   

        self.publisher_ = self.create_publisher(Int8, 'safety', 10)




    def listener_callback(self, msg): 
            zone_robot = []
            zone_one = []
            zone_two = []
            zone_width = 0.3
            points_in_zone_warning = 10
            border_x = 0.48 #+ 0.06 (0.96)
            border_y = 0.30 #+ 0.07 (0.60)
            points = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans= True)               
            for index, element in enumerate(points):
                #zone one
                '''if  abs(element[0]) <= border_x + 0.05 and abs(element[1]) <= border_y + 0.05:
                    if abs(element[1]) > border_y + 0.05 or abs(element[0]) > border_x + 0.05:
                        zone_robot.append(element)
                #zone two
                el'''
                if abs(element[0]) <= (border_x + zone_width) and abs(element[1]) <= (border_y + zone_width):
                    if abs(element[0]) > (border_x) or abs(element[1]) > (border_y):
                        zone_one.append(element)
                #zone three
                elif abs(element[0]) <= (border_x + 2*zone_width) and abs(element[1]) <= (border_y + 2*zone_width):
                    if abs(element[0]) > (border_x + zone_width) or abs(element[1]) > (border_y + zone_width):
                        zone_two.append(element)
            

            if len(zone_robot) > points_in_zone_warning: 
                print(f"\n Nemogoče!! \n  Število {len(zone_robot)} \n Točke v prekršku: {zone_robot}")
                self.publisher_.publish(Int8(data=0))
            elif len(zone_one) > points_in_zone_warning:
                print(f"\n STOP!! nazaj \n  Število {len(zone_one)} \n Točke v prekršku: {zone_one}")
                self.publisher_.publish(Int8(data=1))
            elif len(zone_two) > points_in_zone_warning:
                print(f"\n cona 2 \n  Število {len(zone_two)}") 
                self.publisher_.publish(Int8(data=2))                   
            else:
                print("OK")    
            #self.get_logger().info('\nPerRad: "%f"\nPerForw: "%f"\nRef: "%f"' % (msg.ranges[0], msg.ranges[1], msg.ranges[7])) 
            #print("\nprikaz") 
            #print(points)


            


def main(args=None):
    rclpy.init(args=args)

    subscribe_scanner = SubscribeScanner()


    rclpy.spin(subscribe_scanner)

    subscribe_scanner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
