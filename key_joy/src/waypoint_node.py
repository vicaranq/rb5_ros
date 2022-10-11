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
        return ( self.y_pos, self.x_pos, self.theta_pos)

    def get_deltas(self, curr_pos, target_pos):
        delta_x = target_pos[X] - curr_pos[X]
        delta_y = target_pos[Y] - curr_pos[Y]
        delta_theta = target_pos[THETA] - curr_pos[THETA]
        return (delta_x, delta_y, delta_theta)
    
    def get_joy_msg(self):
        joy_msg = Joy()
        joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        return  joy_msg

    def run_straight_calibration(self):
        joy_msg = Joy()
        joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        target_time = 2.0408 # time to get to 1m is: 100cm/49cm/s = 2.0408 s 
        # ideal: target_time = x / (speed [m/s])
        t_start = time.time()

        joy_msg.axes[X] = 1.2 # >0.1
        
        while time.time() < t_start + target_time:
            self.pub_joy.publish(joy_msg)
            # just wait for target_time          

        joy_msg.axes[X] = 0 # reset 
        self.pub_joy.publish(joy_msg)

        self.stop()

    def run_rotation_calibration(self):
        joy_msg = Joy()
        joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        target_time = 2.5   # [seconds to get to a 90 degrees angle]
        # ideal: target_time = rad / (speed [rad/s])
        t_start = time.time()
        joy_msg.axes[THETA] = 1 # >0.1
        while time.time() < t_start + target_time:
            self.pub_joy.publish(joy_msg)
            # just wait for target_time          

        joy_msg.axes[THETA] = 0 # reset 
        self.pub_joy.publish(joy_msg)

        self.stop()
    
    def run(self, target_position=(1, 1, 0) ):

        #testing msg
        # x, y, theta =  (1, 0, 0) good
        # x, y, theta =  (1, 1, 0)  good
        # x, y, theta =  (1, 1, 1.57)  
        # target_postion = (y, x, theta)

        joy_msg = self.get_joy_msg()
        delta_x, delta_y, delta_theta = self.get_deltas(self.get_current_pos(), target_position)
        
        print("Navigating from {} --> {}".format(self.get_current_pos(), target_position))
        print("delta_x: ", delta_x," | delta_y = ",delta_y, " | delta_theta: ", delta_theta)
        # y_curr, x_curr, theta_curr = self.get_current_pos()
        # move X axis
        if abs(delta_x) > 0.1:
            # if the robot is not at zero degrees, then rotate to make it zero
            self.turn(0,joy_msg)
            self.move_front(delta_x, joy_msg) # front in direction of x axis (world coordinate)
            time.sleep(1)
        # move Y axis
        if abs(delta_y) > 0.1:        
            self.move_sideways_no_slide(delta_y, joy_msg)
            time.sleep(1)
        # move angle
        if abs(delta_theta)  > 0.1:
            self.turn(delta_theta, joy_msg)
            time.sleep(1)
        self.stop()

    def move_sideways_no_slide(self, y, joy_msg):
        ''' function to move robot on the y-axis using rotation instead of sliding'''
        print("[move_sideways_no_slide] Movign sideways for {}m".format(y))
        # If moving to the left, first turn depending of sign of y then move for abs(y) meters to the front
        if y > 0:
            print("Turning 90deg")
            self.turn(math.pi/2 - self.theta_pos, joy_msg) # turn left 90deg
        elif y < 0:
            print("Turning -90deg")
            self.turn(-math.pi/2 - self.theta_pos, joy_msg) # turn right 90 deg
        print("Move front for {}m".format(abs(y)))            
        self.move_front(abs(y), joy_msg)
        self.stop()

    def move_front(self, x, joy_msg):
        '''
        Args:
        x -> int type represeting meters
        '''
        print("[move_front] Moving forward for {}m".format(x))
        time_per_m = 1.94   # [seconds to get to a meter]
        t_start = time.time()
        joy_msg.axes[X] = 1.2 if x >=0 else -1.2 # >0.1         
        while time.time() < t_start + time_per_m*abs(x):
            self.pub_joy.publish(joy_msg)
        joy_msg.axes[X] = 0 # reset 
        self.pub_joy.publish(joy_msg)
        #update
        self.x_pos += x
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
        #update 
        self.y_pos += y
        return y        
    def get_rads(self, theta):
        return self.theta_pos - theta

    def turn(self, theta, joy_msg):
        '''
        theta: angle in radiants to where we want to turn 
        '''
        #calibration_time = 2.5 # [sec/rad]time to get to pi/2
        time_per_rad = 2.5/ (math.pi/2)
        t_start = time.time()
        joy_msg.axes[THETA] = 1 # >0.1
        rads_to_turn = self.get_rads(theta)
        while time.time() < t_start + time_per_rad*rads_to_turn:
            self.pub_joy.publish(joy_msg)
            # just wait for target_time          
        joy_msg.axes[THETA] = 0 # reset 
        self.pub_joy.publish(joy_msg)
        self.stop()
        self.theta_pos = theta

    def stop(self):
        restore_terminal_settings(self.settings)

if __name__ == "__main__":
    waypoint_node = WaypointNode()
    rospy.init_node("waypoint")
    # 
    # waypoint_node.run_straight_calibration()
    # waypoint_node.run_rotation_calibration()

    '''
    0,0,0 -> 0,0,0
    -1,0,0 -> 1,0,0
    -1,1,1.57 -> 1,1,1.57
    -2,1,0 -> 2, 1, 0 
    -2,2,-1.57 -> 2, 2, -1.57 
    -1,1,-0.78 -> 1,1,-0.78
    0,0,0
    '''
    points = [(0,0,0),(1,0,0),(1,1,1.57),(2,1,0),(2,2,-1.57),(1,1,-0.78)]
    for p in points[:4]:
        waypoint_node.run(p)
    