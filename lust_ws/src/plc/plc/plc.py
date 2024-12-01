
import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8
from tutorial_interfaces.msg import PositionInfo, MotionWrite
import pyads
import atexit

from dataclasses import dataclass
import time
from typing import List

@dataclass
class Message:
    lin_vel: float = 0.0
    ang_vel: float = 0.0
    stop: bool = False
    halt: bool = False
    go: bool = False
    home: bool = False

    def update(self, **kwargs):
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
        self.last_updated = time.time()

    def update_from_vector(self, vector: List):
        if len(vector) != 6:
            raise ValueError("Update from vector: Vektor mora imeti točno 6 elementov.")
        self.lin_vel, self.ang_vel, self.stop, self.halt, self.go, self.home = vector

        self.last_updated = time.time()

class MessageDict:
    def __init__(self, priorities):
        self.messages = {key: Message() for key in priorities}
        self.priorities = priorities
    
    def update_message(self,key,**kwargs):
        if key in self.messages:
            self.messages[key].update(**kwargs)
        else:
            raise KeyError(f"Message with key '{key}' does not exist.")

    def update_message_from_vector(self,key,vector):
        if key in self.messages:
            self.messages[key].update_from_vector(vector)
        else:
            raise KeyError(f"Message with key '{key}' does not exist.")        
    
    def get_highest_priority(self):
        current_time = time.time()

        # Filtriraj sporočila, ki so bila posodobljena v zadnjih 5 sekundah
        valid_messages = [
            (key, msg) for key, msg in self.messages.items()
            if current_time - msg.last_updated <= 5
        ]

        if not valid_messages:
            return None  # Ni ustreznih sporočil

        # Razvrsti preostala sporočila po prioriteti in času posodobitve
        sorted_messages = sorted(
            valid_messages,
            key=lambda item: (self.priorities[item[0]], -item[1].last_updated)
        )
        return sorted_messages[0]  # Vrni najpomembnejše sporočilo

    def __getitem__(self, key):
        return self.messages[key]

    def __setitem__(self, key, value):
        if not isinstance(value, Message):
            raise ValueError("Value must be an instance of Message.")
        self.messages[key] = value

    def __repr__(self):
        return f"MessageDict({self.messages})"
    
    
class PLC_mod(Node):

    def __init__(self):
        super().__init__('connection_plc')
        
        # QoS profiles
        qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        priorities = {
            "joystick" : 1,
            "automated" : 2,
            }
        
        self.msg_dict = MessageDict(priorities)

        # Connect to PLC
        self.plc = pyads.Connection('169.254.146.194.1.1', pyads.PORT_TC3PLC1,"192.168.64.110")
        self.plc.open() 

        self.joy_subscription = self.create_subscription(
            Joy, 
            'joy', 
            self.joy_listener_callback,
            10)     
        self.joy_subscription   

        '''self.auto_subscription = self.create_subscription(
            MotionWrite, 
            'auto', 
            self.auto_listener_callback,
            10)     
        self.auto_subscription  '''

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

    def auto_listener_callback(self, msg):
        self.msg_dict.update_message_from_vector("automated", msg)

    def joy_listener_callback(self, msg):
        refVelocity = (2-msg.axes[5])*0.1
        lin_velocity = float(msg.axes[4]) * refVelocity
        ang_velocity = float(msg.axes[1]) * refVelocity
        #self.plc.write_by_name('G_MAIN.PerRadVelocity', float(msg.axes[4]), pyads.PLCTYPE_REAL)
        #self.plc.write_by_name('G_MAIN.PerForwardVelocity', float(msg.axes[1]), pyads.PLCTYPE_REAL)
        
        halt = msg.buttons[3]
        home = msg.buttons[2]

        #self.plc.write_by_name('G_MAIN.Halt', msg.buttons[3], pyads.PLCTYPE_BOOL)
        #self.plc.write_by_name('G_MAIN.Home', msg.buttons[2], pyads.PLCTYPE_BOOL)

        if msg.buttons[1]:
            #self.plc.write_by_name('G_MAIN.Stop', True, pyads.PLCTYPE_BOOL)
            stop = True
            self.msg_dict.update_message("joystick", lin_vel = lin_velocity, ang_vel = ang_velocity, stop = stop, halt = halt, home = home)
        elif msg.buttons[0]:
            stop = False
            go = True
            self.msg_dict.update_message("joystick", lin_vel = lin_velocity, ang_vel = ang_velocity, stop = stop, halt = halt, home = home, go=go)
            #self.plc.write_by_name('G_MAIN.Stop', False, pyads.PLCTYPE_BOOL)
            #self.plc.write_by_name('G_MAIN.bGo', True, pyads.PLCTYPE_BOOL)
        else:
            self.msg_dict.update_message("joystick", lin_vel = lin_velocity, ang_vel = ang_velocity, halt = halt, home = home)


        #self.plc.write_by_name('G_MAIN.RefVelocity', refVelocity, pyads.PLCTYPE_REAL)   
        #self.plc.write_by_name('G_MAIN.bBoolean', True, pyads.PLCTYPE_BOOL)                              
        #self.get_logger().info('\nPerRad: "%f"\nPerForw: "%f"\nRef: "%d"' % (msg.axes[0], msg.axes[1], refVelocity)) 

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

    connection_plc = PLC_mod()
    atexit.register(connection_plc.close_plc)

    rclpy.spin(connection_plc)

    connection_plc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
