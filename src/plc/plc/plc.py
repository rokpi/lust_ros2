
import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Joy
from std_msgs.msg import Int8
from tutorial_interfaces.msg import PositionInfo  
import pyads
import atexit


class PLC_mod(Node):

    def __init__(self):
        super().__init__('subscribe_joy')


        # QoS profiles
        qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Connect to PLC
        self.plc = pyads.Connection('169.254.146.194.1.1', pyads.PORT_TC3PLC1,"192.168.64.110")
        self.plc.open() 
        self.safety = True

        self.joy_subscription = self.create_subscription(
            Joy, 
            'joy', 
            self.joy_listener_callback,
            10)     
        self.joy_subscription   

        self.safety_subscription = self.create_subscription(
            Int8, 
            'safety', 
            self.safety_listener_callback,
            10)     
        self.safety_subscription    

        self.publisher_position = self.create_publisher(PositionInfo, '/positionInfo', qos_profile_reliable)

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)   

    def timer_callback(self):
            position_msg = self.create_position_msg()
            self.publisher_position.publish(position_msg)         


    def joy_listener_callback(self, msg):
            if self.safety:
                refVelocity = (2-msg.axes[5])*0.1
                self.plc.write_by_name('G_MAIN.PerRadVelocity', float(msg.axes[4]), pyads.PLCTYPE_REAL)
                self.plc.write_by_name('G_MAIN.PerForwardVelocity', float(msg.axes[1]), pyads.PLCTYPE_REAL)
		
                if msg.buttons[1]:
                    self.plc.write_by_name('G_MAIN.Stop', True, pyads.PLCTYPE_BOOL)
                
                self.plc.write_by_name('G_MAIN.Halt', msg.buttons[3], pyads.PLCTYPE_BOOL)
                self.plc.write_by_name('G_MAIN.Home', msg.buttons[2], pyads.PLCTYPE_BOOL)
                if msg.buttons[0]:
                    self.plc.write_by_name('G_MAIN.Stop', False, pyads.PLCTYPE_BOOL)
                    self.plc.write_by_name('G_MAIN.bGo', True, pyads.PLCTYPE_BOOL) 
   
        
                self.plc.write_by_name('G_MAIN.RefVelocity', refVelocity, pyads.PLCTYPE_REAL)   
                self.plc.write_by_name('G_MAIN.bBoolean', True, pyads.PLCTYPE_BOOL)                              
                self.get_logger().info('\nPerRad: "%f"\nPerForw: "%f"\nRef: "%d"' % (msg.axes[0], msg.axes[1], refVelocity)) 


    def safety_listener_callback(self, msg):
            if msg.data == 1:
                self.safety = False
                self.plc.write_by_name('G_MAIN.Stop', True, pyads.PLCTYPE_BOOL) 
                self.plc.write_by_name('G_MAIN.bBoolean', True, pyads.PLCTYPE_BOOL) 
                #self.get_logger().info('STOP!')

            else:
                 self.safety = True

    def create_position_msg(self):
         position_msg = PositionInfo()
         position_msg.axis1_pos = self.plc.read_by_name('G_MAIN.Axis1.NcToPlc.ActPos', pyads.PLCTYPE_LREAL)
         position_msg.axis2_pos = self.plc.read_by_name('G_MAIN.Axis2.NcToPlc.ActPos', pyads.PLCTYPE_LREAL)
         position_msg.axis1_vel = self.plc.read_by_name('G_MAIN.Axis1.NcToPlc.ActVelo', pyads.PLCTYPE_LREAL)
         position_msg.axis2_vel = self.plc.read_by_name('G_MAIN.Axis2.NcToPlc.ActVelo', pyads.PLCTYPE_LREAL)

         return position_msg           
            

    def close_plc(self):
        self.plc.close()
        self.get_logger().info('PLC connection closed.')

def main(args=None):
    rclpy.init(args=args)

    subscribe_joy = PLC_mod()
    atexit.register(subscribe_joy.close_plc)

    rclpy.spin(subscribe_joy)

    subscribe_joy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
