
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Int8

import pyads
import atexit


class SubscribeJoy(Node):

    def __init__(self):
        super().__init__('subscribe_joy')

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




    def joy_listener_callback(self, msg):
            if self.safety:
                refVelocity = (2-msg.axes[5])*10
                self.plc.write_by_name('G_MAIN.PerRadVelocity', float(msg.axes[0]), pyads.PLCTYPE_REAL)
                self.plc.write_by_name('G_MAIN.PerForwardVelocity', float(msg.axes[1]), pyads.PLCTYPE_REAL)

                self.plc.write_by_name('G_MAIN.Stop', msg.buttons[1], pyads.PLCTYPE_BOOL)
                self.plc.write_by_name('G_MAIN.Halt', msg.buttons[3], pyads.PLCTYPE_BOOL)
                if msg.buttons[0]:
                    self.plc.write_by_name('G_MAIN.bGo', True, pyads.PLCTYPE_BOOL)                 
        
                self.plc.write_by_name('G_MAIN.RefVelocity', refVelocity, pyads.PLCTYPE_REAL)   
                self.plc.write_by_name('G_MAIN.bBoolean', True, pyads.PLCTYPE_BOOL)                              
                self.get_logger().info('\nPerRad: "%f"\nPerForw: "%f"\nRef: "%d"' % (msg.axes[0], msg.axes[1], refVelocity)) 


    def safety_listener_callback(self, msg):
            if msg.data == 1:
                self.safety = False
                self.plc.write_by_name('G_MAIN.Stop', True, pyads.PLCTYPE_BOOL) 
                self.plc.write_by_name('G_MAIN.bBoolean', True, pyads.PLCTYPE_BOOL) 
                self.get_logger().info('STOP!')

            else:
                 self.safety = True
            
            

    def close_plc(self):
        self.plc.close()
        self.get_logger().info('PLC connection closed.')

def main(args=None):
    rclpy.init(args=args)

    subscribe_joy = SubscribeJoy()
    atexit.register(subscribe_joy.close_plc)

    rclpy.spin(subscribe_joy)

    subscribe_joy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
