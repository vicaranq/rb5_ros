#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import Joy
from tf2_msgs.msg import TFMessage
from key_parser import get_key, save_terminal_settings, restore_terminal_settings
import time, math
import numpy as np
X = 1
Y = 0 
THETA = 2

class FeedbackNode:
    def __init__(self):
        self.pub_joy = rospy.Publisher("/joy", Joy, queue_size=1)
        self.settings = save_terminal_settings()

        # Initialize position of robot 
        # (assuming -x to front, +y to the left, theta opening from -x towards y axis)
        # self.x_pos = 0
        # self.y_pos = 0
        self.x_w = 0
        self.y_w = 0
        self.theta_pos = 0

        self.tags = {}


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

    def get_translation(self, message):
        '''
        Returns tupple (x_T,y_T,z_T) from tag coordinates to TAG coordinates frame
        '''
        return (message.tranforms[0].transform.translation.x, \
                message.tranforms[0].transform.translation.y, \
                message.tranforms[0].transform.translation.z)

    def get_rotation(self, message):
        '''
        Returns tupple (x_T,y_T,z_T) from tag coordinates to TAG coordinates frame
        '''
        return (message.tranforms[0].transform.rotation.x, \
                message.tranforms[0].transform.rotation.y, \
                message.tranforms[0].transform.rotation.z, \
                message.tranforms[0].transform.rotation.w)
    
    def tag_information(self,  message):

        if message:
            # expecting message from /tf topic
            try: 
                # print("meesage:", message)
                tag_id = message.tranforms[0].transform.child_frame_id
                assert type(tag_id) == str, "Unexpected tag type"
                self.tags[tag_id]={"id": tag_id, \
                    "translation" : self.get_translation(message), \
                        "rotation" : self.get_rotation(message)}
            except:
                pass                

    def run(self, target_position_w, tag_id):
        '''
        Args:
        target_position_w -> Target position in world coordinates 
        tag_id -> unique identifier for  tag associaetd to the target position (1m away from actual target)
        
        '''
        #testing msg
        # x, y, theta =  (1, 0, 0) good
        # x, y, theta =  (1, 1, 0)  good
        # x, y, theta =  (1, 1, 1.57)  
        # target_postion = (y, x, theta)
        
        # Target in world coordinates
        x_target, y_target, alpha_target = target_position_w
        # Obtain Tag information
        rospy.Subscriber("/tf", TFMessage, self.tag_information)

        # tag_pos_T = self.tags[tag_id]# tag position information in world tag coordinate frame       
        
        t_start = time.time()
        while time.time() < t_start + 10:
            if self.tags:
                print("tag info: ", self.tags)                

        print("closing...")
        self.stop()
        #if first tag: NOTE: Depending of the tag, the tag coord frame maps differently to world one
        
        # X, Y, Z = (0,1,2)
        # tag_pos_x_w, tag_pos_y_w = (tag_pos_T['translation'][Z], -1*tag_pos_T['translation'][X]) # distance to x location in world coord.

        # dist_to_target_x_w = tag_pos_x_w - (x_target - self.x_w)

        # print(dist_to_target_x_w, end=" ")

        # initial_tag_reading_robot = self.tags[tag_id]
        # target_tag_reading_robot = initial_tag_reading_robot-diff




        '''
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
            self.turn(target_position[2], joy_msg)
            time.sleep(1)
        print("State: ", (self.x_pos, self.y_pos, self.theta_pos))
        self.stop()
        # update world coordinate for robot

        '''
        

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
        self.move_front(y, joy_msg, y_axis=True)

        self.stop()

    def move_front(self, d, joy_msg, y_axis=False):
        '''
        Args:
        d -> int type represeting meters
        '''
        print("[move_front] Moving forward for {}m".format(d))
        # time_per_m = 2.0408   # [seconds to get to a meter]
        
        #t_start = time.time()

        joy_msg.axes[X] = 1.2 if d >=0 or y_axis else -1.2 # >0.1         
        #while time.time() < t_start + time_per_m*abs(d):
        self.pub_joy.publish(joy_msg)
        time.sleep(0.5)
        joy_msg.axes[X] = 0 # reset 
        self.pub_joy.publish(joy_msg)
        #update
        if not y_axis:
            self.x_pos += d
        else:
            self.y_pos += d
       
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
        # while time.time() < t_start + time_per_rad*abs(rads_to_turn):
        self.pub_joy.publish(joy_msg)
        time.sleep(0.5)
            # just wait for target_time          
        joy_msg.axes[THETA] = 0 # reset 
        self.pub_joy.publish(joy_msg)
        # self.theta_pos = theta
        print("[turn] theta updated and turned {}rads".format(rads_to_turn))
        self.stop()

    def stop(self):
        restore_terminal_settings(self.settings)

def get_points_from_file(fname="waypoints.txt"):
    f = open(fname, "r")
    points = []
    for line in f:
        temp = line.strip().split(",")
        if len(temp) == 3: 
            points.append((float(temp[0])*-0.8, float(temp[1])*0.8, float(temp[2])))
    print("[file]{} points loaded".format(len(points)))
    return points

if __name__ == "__main__":
    feedback_node = FeedbackNode()
    rospy.init_node("feedback")
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
    # points = get_points_from_file()
    # print(points)
    # points = [(0,0,0),(1,0,0),(1,1,1.57),(2,1,0),(2,2,-1.57),(1,1,-0.78),(0,0,0)]

    points = [(1,0,0), (1,2,np.pi), (0,0,0)]    
    tags = ["marker_1",2,3] # tag ids associated to each position

    # for p,tag_id in (points, tags):
    p, tag_id = (points[0], tags[0])
    print("Starting navigation to target point: ", p, " tag: ", tag_id)        
    feedback_node.run(p, tag_id)
