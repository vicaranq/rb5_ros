#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import Joy
from key_parser import get_key, save_terminal_settings, restore_terminal_settings
import time, math

X = 1
Y = 0 
THETA = 2

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
    
    def get_joy_msg(self):
        joy_msg = Joy()
        joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        return  joy_msg

    def run_straight_calibration(self):
        joy_msg = Joy()
        joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]

        target_time = 1.94   # [seconds to get to a meter]
        # ideal: target_time = x / (speed [m/s])

        t_start = time.time()

        joy_msg.axes[X] = 1.2 # >0.1
        self.pub_joy.publish(joy_msg)
        # do I need to publis every cycle, or should I do only once and wait for some time?
        #         
        while time.time() < t_start + target_time:
            pass # just wait for target_time          

        joy_msg.axes[X] = 0 # reset 
        self.pub_joy.publish(joy_msg)

        self.stop()

    def run_rotation_calibration(self):
        joy_msg = Joy()
        joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]

        target_time = 2   # [seconds to get to a meter]
        # ideal: target_time = x / (speed [m/s])

        t_start = time.time()

        joy_msg.axes[THETA] = 1 # >0.1
        self.pub_joy.publish(joy_msg)
        # do I need to publis every cycle, or should I do only once and wait for some time?
        #         
        while time.time() < t_start + target_time:
            pass # just wait for target_time          

        joy_msg.axes[2] = 0 # reset 
        self.pub_joy.publish(joy_msg)

        return target_time
    

    def run(self):
        #testing msg
        x, y, theta =  (1, 0, 0)

        target_postion = (x, y, theta)

        joy_msg = self.get_joy_msg()

        #delta_x, delta_y, delta_theta = self.get_deltas(msg, self.get_current_pos())
        #delta_x, delta_y, delta_theta  = (1, 0, 0)
        delta_x, delta_y, delta_theta  = (0, 1, 0)  

        # move X axis
        if delta_x != 0:
            self.move_front(1.2) # arg is speed
        # move Y axis
        if delta_y != 0:        
            # self.move_sideways(1)
            self.move_sideways_no_slide(delta_y, joy_msg)
        # move angle

        self.stop()

    def move_sideways_no_slide(self, y, joy_msg):
        ''' function to move robot on the y-axis using rotation instead of sliding'''

        # If moving to the left, first turn depending of sign of y then move for abs(y) meters to the front
        if y > 0:
            self.turn(math.pi/2, joy_msg) # turn left 90deg
        elif y < 0:
            self.turn(-math.pi/2, joy_msg) # turn right 90 deg
            
        self.move_front(abs(y))

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

        target_time = 1.94   # [seconds to get to a meter]
        # ideal: target_time = x / (speed [m/s])

        t_start = time.time()
        
        
        while time.time() < t_start + target_time:
            key = get_key(self.settings, timeout=0.1)
            joy_msg.axes[1] = 1.2 # >0.1 
            self.pub_joy.publish(joy_msg)
            if (len(key) > 0 and ord(key) == 27) or (key == '\x03'):
                break

        joy_msg.axes[1] = 0 # reset 
        self.pub_joy.publish(joy_msg)

        return x

    def move_sideways(self, y):
        '''
        Args:
        x -> int type represeting meters

        NOTE: Sliding not woeking as expected, it may be due to wheel setup basedon TA's input
        '''
        # calibrate 

        # translate x to time needed to get to that position
        # whats the speed? -> T m/s?  get moving forward speed

        
        joy_msg = Joy()
        joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]

        target_time = 3  # [seconds to get to a meter]
        t_start = time.time()        
        # UNCOMMENT AFTER EXPERIMENTS
        # while time.time() < t_start + target_time:
        #     joy_msg.axes[0] = 1.2 # >0.1 
        #     self.pub_joy.publish(joy_msg)
        
        # for experiments:
        joy_msg.axes[0] = 1.2 # >0.1 
        self.pub_joy.publish(joy_msg)
        time.sleep(14)
        joy_msg.axes[0] = 0 # reset 
        self.pub_joy.publish(joy_msg)

        return y        
    
    def turn(self, theta, joy_msg):
        # joy_msg[THETA] = 1

        # joy_msg = Joy()
        # joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        # joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]

        target_time = 1.94   # [seconds to get to 90 degree angle ]
        ''' Note: Need to experiment to find out how much time it takes to take a 90degrees turn'''
        # ideal: target_time = (theta/(pi/2)) / (speed [90degrees/s])

        t_start = time.time()
        
        joy_msg.axes[THETA] = 1 # >0.1
        self.pub_joy.publish(joy_msg)
        # do I need to publis every cycle, or should I do only once and wait for some time?
        #         
        while time.time() < t_start + target_time:
            pass # just wait for target_time          

        joy_msg.axes[2] = 0 # reset 
        self.pub_joy.publish(joy_msg)

        return theta

        pass 
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
    # waypoint_node.run()
    # waypoint_node.run_rotation_calibration()
    waypoint_node.run_straight_calibration()
    