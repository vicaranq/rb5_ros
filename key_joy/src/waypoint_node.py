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
        ''' curr_pos=(y,x,theta) , target_pos=(x,y,theta)'''
        delta_x = target_pos[0] - curr_pos[X]
        delta_y = target_pos[1] - curr_pos[Y]
        delta_theta = target_pos[2] - curr_pos[THETA]
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
        # target_time = 2.0408 # time to get to 1m is: 100cm/49cm/s = 2.0408 s 
        # target_time = 3 # 3s -> 111cm  then speed is 111cm/3s = 37 cm/s --> 100cm/37cm/s = 2.7027
        target_time = 2.7027
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
        delta_x, _, _ = self.get_deltas(self.get_current_pos(), target_position)
        print("Navigating from {} --> {}".format((self.x_pos,self.y_pos, self.theta_pos), target_position))
        print("delta_x: ", delta_x)
        # y_curr, x_curr, theta_curr = self.get_current_pos()
        # move X axis
        if abs(delta_x) > 0.1:
            # if the robot is not at zero degrees, then rotate to make it zero
            print("Turning to zero degrees...")
            self.turn(0,joy_msg)
            self.move_front(delta_x, joy_msg) # front in direction of x axis (world coordinate)
            time.sleep(1)
        # move Y axis
        _, delta_y, _ = self.get_deltas(self.get_current_pos(), target_position)
        print("delta_y: ", delta_y)
        if abs(delta_y) > 0.1:        
            self.move_sideways_no_slide(delta_y, joy_msg)
            time.sleep(1)
        _, _, delta_theta = self.get_deltas(self.get_current_pos(), target_position)
        print("delta_theta: ", delta_theta)
        # move angle
        if abs(delta_theta)  > 0.1:
            self.turn(delta_theta, joy_msg)
            time.sleep(1)
        print("State: ", (self.x_pos, self.y_pos, self.theta_pos))
        self.stop()

    def move_sideways_no_slide(self, y, joy_msg):
        ''' function to move robot on the y-axis using rotation instead of sliding'''
        print("[move_sideways_no_slide] Movign sideways for {}m".format(y))
        # If moving to the left, first turn depending of sign of y then move for abs(y) meters to the front
        if y > 0:
            print("Turning 90deg -> {}rads".format(math.pi/2 - self.theta_pos))
            self.turn(math.pi/2, joy_msg) # turn left 90deg
        elif y < 0:
            print("Turning -90deg")
            self.turn(-math.pi/2, joy_msg) # turn right 90 deg
        print("Move front for {}m".format(abs(y)))            
        self.move_front(abs(y), joy_msg, y_axis=True)

        self.stop()

    def move_front(self, d, joy_msg, y_axis=False):
        '''
        Args:
        d -> int type represeting meters
        '''
        print("[move_front] Moving forward for {}m".format(d))
        time_per_m = 2.0408   # [seconds to get to a meter]
        t_start = time.time()
        joy_msg.axes[X] = 1.2 if d >=0 else -1.2 # >0.1         
        while time.time() < t_start + time_per_m*abs(d):
            self.pub_joy.publish(joy_msg)
        joy_msg.axes[X] = 0 # reset 
        self.pub_joy.publish(joy_msg)
        #update
        if not y_axis:
            self.x_pos += d
        else:
            self.y_pos += d

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
        return theta - self.theta_pos

    def turn(self, theta, joy_msg):
        '''
        theta: angle in radiants to where we want to turn 
        '''
        #calibration_time = 2.5 # [sec/rad]time to get to pi/2
        time_per_rad = 2.5/ (math.pi/2)
        t_start = time.time()
        rads_to_turn = self.get_rads(theta)
        joy_msg.axes[THETA] = 1 if rads_to_turn >= 0 else -1# >0.1
        while time.time() < t_start + time_per_rad*abs(rads_to_turn):
            self.pub_joy.publish(joy_msg)
            # just wait for target_time          
        joy_msg.axes[THETA] = 0 # reset 
        self.pub_joy.publish(joy_msg)
        self.theta_pos = theta
        print("[turn] theta updated and turned {}rads".format(rads_to_turn))
        self.stop()

    def stop(self):
        restore_terminal_settings(self.settings)

if __name__ == "__main__":
    waypoint_node = WaypointNode()
    rospy.init_node("waypoint")
    # 
    waypoint_node.run_straight_calibration()
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
    # points = [(0,0,0),(1,0,0),(1,1,1.57),(2,1,0),(2,2,-1.57),(1,1,-0.78)]
    # for p in points[:-1]:
    #     waypoint_node.run(p)
    