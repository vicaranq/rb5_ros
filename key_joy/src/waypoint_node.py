#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import Joy
from key_parser import get_key, save_terminal_settings, restore_terminal_settings
import time

class WaypointNode:
    def __init__(self):
        self.pub_joy = rospy.Publisher("/joy", Joy, queue_size=1)
        self.settings = save_terminal_settings()

        # Initialize position of robot 
        # (assuming -x to front, +y to the left, theta opening from -x towards y axis)
        self.x_pos = 0
        self.y_pos = 0
        self.theta_pos = 0

    def get_current_pos(self):
        return (self.x_pos, self.y_pos, self.theta_pos)

    def get_deltas(self, curr_pos):
        pass

    def run(self):
        #testing msg
        x, y, theta =  (1, 0, 0)

        msg = (x, y, theta)

        #delta_x, delta_y, delta_theta = self.get_deltas(msg, self.get_current_pos())
        delta_x = 1 

        # move X axis
        if delta_x != 0:
            self.move_front(1.2) # arg is speed
        # move Y axis

        # move angle

        self.stop()

    def move_front(self, x):
        '''
        Args:
        x -> int type represeting meters
        '''
        # calibrate 

        # translate x to time needed to get to that position
        # whats the speed? -> T m/s?  get moving forward speed

        
        joy_msg = Joy()
        joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]

        target_time = 3   # [seconds]

        # ideal: target_time = x / (speed [m/s])

        t_start = time.time()
        
        
        while time.time() < t_start + target_time:
            key = get_key(self.settings, timeout=0.1)

            joy_msg.axes[1] = 1.2 # >0.1  would going forward have the same speed as moving backwards?
            self.pub_joy.publish(joy_msg)

            if (len(key) > 0 and ord(key) == 27) or (key == '\x03'):
                break

        joy_msg.axes[1] = 0 # reset 
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
    