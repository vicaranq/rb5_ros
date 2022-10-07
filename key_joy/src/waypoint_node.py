#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import Joy
from key_parser import  save_terminal_settings, restore_terminal_settings

class WaypointNode:
    def __init__(self):
        self.pub_joy = rospy.Publisher("/joy", Joy, queue_size=1)
        self.settings = save_terminal_settings()


    def run(self):

        self.move_front(1) #move 1m?
        
        self.stop()

    def move_front(self, x):
        '''
        Args:
        x -> int type represeting meters
        '''
        # calibrate 

        joy_msg = Joy()
        joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        if x >= 0: #key == 'w':
            joy_msg.axes[1] = x
        else:
            joy_msg.axes[1] = -x
        
        
        self.pub_joy.publish(joy_msg)


        return x
    '''
    def run(self):
        while True:
            # parse keyboard control
            key = get_key(self.settings, timeout=0.1)

            # interpret keyboard control as joy
            joy_msg, flag = self.key_to_joy(key)
            if flag is False:
                break

            # publish joy
            self.pub_joy.publish(joy_msg)
    
        self.stop()
    
    def key_to_joy(self, key):
        flag = True
        joy_msg = Joy()
        joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        if key == 'w':
            joy_msg.axes[1] = 1.0
        elif key == 's':
            joy_msg.axes[1] = -1.0
        elif key == 'a':
            joy_msg.axes[0] = -1.0
        elif key == 'd':
            joy_msg.axes[0] = 1.0
        elif key == 'q':
            joy_msg.axes[2] = -1.0
        elif key == 'e':
            joy_msg.axes[2] = 1.0
        elif (len(key) > 0 and ord(key) == 27) or (key == '\x03'):
            flag = False
        return joy_msg, flag
    '''
    

    def stop(self):
        restore_terminal_settings(self.settings)


if __name__ == "__main__":
    waypoint_node = WaypointNode()
    rospy.init_node("waypoint")
    waypoint_node.run()
    