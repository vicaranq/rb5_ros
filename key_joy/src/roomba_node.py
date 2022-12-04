#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import Joy
from tf2_msgs.msg import TFMessage
from key_parser import get_key, save_terminal_settings, restore_terminal_settings
import time, math
import numpy as np

from tf.transformations import *
from geometry_msgs.msg import Quaternion
import tf 
import copy

X = 1
Y = 0 
THETA = 2

GRID_UNIT = 0.05
THRESHOLD = 2*GRID_UNIT

MAP = np.zeros((int(2/0.05),int(2/0.05)))
# define obstacle
# for i in range(len(MAP)//2-3,len(MAP)//2+3):
#     for j in range(len(MAP)//2-3,len(MAP)//2+3):
#         MAP[i][j] = -1



# define start and end position
START = (2,2)


#result need to *0.05
def plan_path():
    path = []
    # assume 10cm boundary for safety
    map_augment = copy.deepcopy(MAP)
    for i in range(0,len(map_augment)):
        for j in range(0,len(map_augment)):
            map_augment[i][j] = 1
    for i in range(2,len(map_augment)-2):
        for j in range(2,len(map_augment)-2):
            map_augment[i][j] = 0
    
    #main idea: assume rectangular shape, and account for unvisited later
    # s = (START[0], START[1], 0)
    # path.append(s)
    # assume (2,2), facing up,
    curr_index_x = 2
    curr_index_y = 2
    for iteration in range(0,len(map_augment)//2):

        if 0 in map_augment:
            for ind in range(curr_index_x,len(map_augment)):
                if map_augment[curr_index_x][ind]==0:
                    map_augment[curr_index_x][ind]=1
                else:
                    curr_index_y = ind-1
                    path.append((curr_index_x, curr_index_y,math.pi/2))
                    break
            for ind in range(curr_index_x+1,len(map_augment)):
                if map_augment[ind][curr_index_y]==0:
                    map_augment[ind][curr_index_y]=1
                else:
                    curr_index_x = ind-1
                    path.append((curr_index_x, curr_index_y,math.pi))
                    break
            for ind in range(curr_index_x-1,0,-1):
                if map_augment[curr_index_x][ind]==0:
                    map_augment[curr_index_x][ind]=1
                else:
                    curr_index_y = ind+1
                    path.append((curr_index_x, curr_index_y,math.pi/2*3))
                    break        
            for ind in range(curr_index_x-1,0,-1):
                if map_augment[ind][curr_index_y]==0:
                    map_augment[ind][curr_index_y]=1
                else:
                    curr_index_x = ind+1
                    path.append((curr_index_x-1, curr_index_y,math.pi/4))
                    break
            curr_index_y = curr_index_y+1
            path.append((curr_index_x, curr_index_y,0))

        else:

            break
    left_spaces = np.where(map_augment==0)
    x_list = left_spaces[0]
    y_list = left_spaces[1]
    points = zip(x_list,y_list)
    for i in points:
        path.append((i[0],i[1],0))
    
    for point in range(len(path)):
        path[point] = (path[point][0]*0.05, path[point][1]*0.05, path[point][2])

    return path
    
class RoombaNode:
    def __init__(self):
        self.pub_joy = rospy.Publisher("/joy", Joy, queue_size=1)
        self.settings = save_terminal_settings()

        # Initialize position of robot 
        # (assuming -x to front, +y to the left, theta opening from -x towards y axis
        self.x_w = 0.1
        self.y_w = 0.1
        self.theta_w = 0

        self.tags = [np.array([0,2,1]), np.array([.75,2,1]), np.array([1.45,1.65,1]), np.array([1.45,0.65,1]), np.array([0.8, 0,1])]
        self.current_seen_tags = {}


    def get_current_pos(self):
        return ( self.x_w, self.y_w)

    def get_deltas(self, curr_pos, target_pos):
        ''' curr_pos=(x,y) , target_pos=(x,y)'''
        delta_x = target_pos[0] - curr_pos[0]
        delta_y = target_pos[1] - curr_pos[1]
        # delta_theta = target_pos[2] - curr_pos[2]
        return (delta_x, delta_y)
    
    def get_joy_msg(self):
        joy_msg = Joy()
        joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        return  joy_msg

    def get_translation(self, message):
        '''
        Returns tupple (x_T,y_T,z_T) from tag coordinates to TAG coordinates frame
        '''
        return (message.transforms[0].transform.translation.x, \
                message.transforms[0].transform.translation.y, \
                message.transforms[0].transform.translation.z)

    def get_rotation(self, message):
        '''
        Returns tupple (x_T,y_T,z_T) from tag coordinates to TAG coordinates frame
        '''
        return (message.transforms[0].transform.rotation.x, \
                message.transforms[0].transform.rotation.y, \
                message.transforms[0].transform.rotation.z, \
                message.transforms[0].transform.rotation.w)
    
    def tag_information(self,  message):

        if message:
            # expecting message from /tf topic
            try: 
                # print("message:\n", message)
                # print("message_type:\n", type(message))
                # print("message.transforms_type:\n", type(message.transforms))

                # print('msg.transforms[0]', message.transforms[0])
                # print('msg.transforms[0].transform', message.transforms[0].transform)
                # print('x', message.transforms[0].transform.translation.x)

                tag_id = message.transforms[0].child_frame_id
                assert type(tag_id) == str, "Unexpected tag type"
                # self.tags[tag_id]={"id": tag_id, \
                #     "translation" : self.get_translation(message), \
                #     "rotation" : self.get_rotation(message)}
                # print("tags updated!")

                # self.current_seen_tags will act similarly as self.tags, however it clears every iteration (step in the experiment e.g. every 0.1m or turn)
                # self.current_seen_tags = (self.get_translation(message)[2] , \
                #                                   -1.0*self.get_translation(message)[0])

                self.current_seen_tags[tag_id] = {"x" : self.get_translation(message)[2] , \
                                                  "y":self.get_translation(message)[0], \
                                                  "id": tag_id, \
                                                  "translation" : self.get_translation(message), \
                                                  "rotation" : self.get_rotation(message)
                                                 }                                                    
            except:
                print("something fail")
                print(tag_id)
                raise
                pass   

    def stop_robot(self):
        # reset 
        joy_msg = self.get_joy_msg()        
        joy_msg.axes[X] = 0 
        joy_msg.axes[Y] = 0 
        joy_msg.axes[THETA] = 0 
        self.pub_joy.publish(joy_msg)

    def readjust_angle(self, tag_pos_y_r, d_x):
        if abs(tag_pos_y_r) > 0.005 and abs(tag_pos_y_r/d_x) <= 1 and d_x > 0.2: # if more than 5cm, and it's a valid value to asin(), and d is not so small, then readjust angle
            # stop before turning
            self.stop_robot()

            #get heuristic angle
            theta = -1.0*math.asin(tag_pos_y_r/d_x) * 0.7 # must be -1 no? 
            # theta = math.atan(tag_pos_y_r/d_x) # must be -1 no? 
            print("adjusting by: {} deg (tag_pos_y_r: {} and d_x: {})".format(theta*180/math.pi, tag_pos_y_r, d_x))

            # if we are facing to +x then it is theta (tag #1 )

            # if we are facing to +y then it is theta + 90 (tag #2 )
            joy_msg = self.get_joy_msg()
            self.turn(self.theta_w+theta, joy_msg) # turn wihtout updating theta of robot, update once reached the target

    def get_w_cord_for_tag(self, tag_pos_T):
        X, Y, Z = (0,1,2)
        # tag_pos_x_w, tag_pos_y_r = (tag_pos_T['translation'][Z], -1*tag_pos_T['translation'][X]) # distance to x location in world coord.
        # NOTE: change this depending on the tag! 
        return (tag_pos_T['translation'][Z], tag_pos_T['translation'][X]) # distance to x location in world coord.

    def move_sideways_no_slide(self, y):
        ''' function to move robot on the y-axis world frame using rotation instead of sliding'''
        print("[move_sideways_no_slide] Movign sideways for {}m".format(y))
        # If moving to the left, first turn depending of sign of y then move for abs(y) meters to the front
        if y > 0:
            print("Turning 90deg")
            self.turn(math.pi/2) # turn left 90deg
            #self.readjust_angle_with_quaternions(tag_id) 
        elif y < 0:
            print("Turning -90deg")
            self.turn(-math.pi/2) # turn right 90 deg
            #self.readjust_angle_with_quaternions(tag_id) 
        time.sleep(1)
        print("Move front for {}m".format(abs(y)))            
        self.move_front(y, y_axis=True)
        self.stop()
    
    def reduce_speed(self, d, speed):
        d_ = abs(d)
        if d_ <= 0.2:
            # REDUCING TO .5 WAS TOO MUCH
            return speed*0.75
        return speed            

    def readjust_angle_with_quaternions(self, tag_id):
        # make it work for tag 2 first
        print("Readusting with quaternions....")

        # find quaternion of reference
        # Q for landmark 4 --> 'marker_1'
        # tagLM4_q = [-0.05788680893984566, 0.03995771047955379, 0.6919213818811771, 0.718538307969475]
        # Q for landmark 8 --> 'marker_7'
        tagLM8_q = [-0.11650380279464587, 0.03710365722695598, 0.992326616226908, -0.018386660447882553]
        
        tag_q_dict = { 'marker_7': tagLM8_q} 

        if tag_id in tag_q_dict:
        #assert tag_id in tag_q_dict, "Unexpected marker in quatrernion dict"

            q2 = tag_q_dict[tag_id]
            print("Using q2: ", q2)
            q1 = list(self.tags[tag_id]['rotation'])
            q1_inv = q1
            q1_inv[3] = -q1_inv[3] 
            qr = tf.transformations.quaternion_multiply(q2, q1_inv)
            (roll, pitch, yaw) = euler_from_quaternion (qr) # from tf.transformations       
            
            print("Pitch value: ", pitch)
            # readjust angle only if pitch is greater than 0.01 
            if abs(pitch) > 0.01 and abs(pitch) < math.pi/2.0: # ~ off by 2deg is fine, and also offset angle shouldn't be greater than 90deg
                heuristic_pitch = pitch #*0.8
                self.turn(heuristic_pitch + self.theta_w) # readjusting to angle coordinates +CW and -CCC
                time.sleep(0.2)   


    def turn_10(self, left=False):
        joy_msg = self.get_joy_msg()
        # time_per_rad = 2.3/ (math.pi/2)
        #time_per_rad = 2.8/ (math.pi/2)
        time_per_rad = 3/ (math.pi/2/9)

        t_start = time.time()
        
        if left:
            joy_msg.axes[THETA] = 0.7
            self.theta_w+=math.pi/2/9
        else:    
            joy_msg.axes[THETA] = -0.7
            self.theta_w-=math.pi/2/9

        while time.time() < t_start + time_per_rad*np.pi/2:
        # while time.time() < t_start + time_per_rad*angle:
            self.pub_joy.publish(joy_msg)
            # just wait for target_time          
        joy_msg.axes[THETA] = 0 # reset 
        self.pub_joy.publish(joy_msg)
        self.current_seen_tags = {}

    def turn(self, theta, scale = False):
        '''
        theta: angle in radiants to where we want to turn in world coordinates! If theta = 0, then turn to where +x axis of the world/map coordinate is
        '''
        joy_msg = self.get_joy_msg()


        t_start = time.time()
        rads_to_turn = self.get_rads(theta)


        # From calibration tests:
        time_per_rad = 2.4/ (math.pi/2) if rads_to_turn >=0 else 2.4/ (math.pi/2)

        # rads_to_turn has to be within 2pi range
        # sign = -1 if rads_to_turn >=0 else 1
        # rads_to_turn = abs(rads_to_turn) % (2*math.pi)
        # rads_to_turn = sign*rads_to_turn

        # joy_msg.axes[THETA] = 1.1 if rads_to_turn >= 0 else -1.1# >0.1
        joy_msg.axes[THETA] = -0.9 if rads_to_turn >= 0 else 0.9# >0.1
        # if scale:
        #     # used for angle readdjustment
        #     joy_msg.axes[THETA] = 0.5 if rads_to_turn >= 0 else -0.5# >0.1

        while time.time() < t_start + time_per_rad*abs(rads_to_turn):
            self.pub_joy.publish(joy_msg)
            # just wait for target_time          
        joy_msg.axes[THETA] = 0 # reset 
        self.pub_joy.publish(joy_msg)
        
        # theta_w has to be within 2pi range   
        sign = 1 if theta >=0 else -1    
        theta_mod =  abs(theta) %  (2*math.pi)
        self.theta_w =  sign*theta_mod
        print("[turn] theta updated to {} and turned {}rads".format(self.theta_w, rads_to_turn))
        self.stop()

    def check_danger_zone2(self, moving_on_y_flag):

        for tag_i in self.current_seen_tags:
            tag_pos_x_r, tag_pos_y_r  = self.get_w_cord_for_tag(self.current_seen_tags[tag_i]) 
            if abs(tag_pos_x_r) <= 0.1 and abs(tag_pos_y_r) <= 0.1:
                print("{} in Danger Zone!".format(tag_i))
                print("Stopping vehicle!")
                joy_msg = self.get_joy_msg()
                joy_msg.axes[X] = 0 # reset 
                self.pub_joy.publish(joy_msg)

                # if object to the left, then move to the right
                # Y positive means that object is to the right (given robot y-axis points to the left)
                if tag_pos_y_r > 0: 
                    # move to the left
                    print("!!! MOVE TO THE LEFT !!!")
                    self.turn_10(left=False)


                else:
                    print("!!! MOVE TO THE RIGHT !!!")
                    # move to the right 
                    self.turn_10(left=True)

    def move_with_tag(self, d, tag_id, y_axis=False, moving_diag=False):
        
        joy_msg = self.get_joy_msg()
        tag_pos_x_r, tag_pos_y_r  = self.get_w_cord_for_tag(self.tags[tag_id])
        target_pos_x = tag_pos_x_r - d
        print("[move_front] Moving forward for {}m".format(d))

        #joy_msg.axes[X] = 1.2 if d >=0 or y_axis else -1.2 # >0.1   
        joy_msg.axes[X] = 1.0 if d >=0 or y_axis else -1.0 # >0.1  

        if moving_diag:
            print("Moving slower in diagonal")
            joy_msg.axes[X] = 0.8 if d >=0 else -0.8 # >0.1         

        # if d is within 20 cm, start reducing the speed
        #joy_msg.axes[X] = self.reduce_speed(d, joy_msg.axes[X])

        while tag_pos_x_r-target_pos_x > 0.1: # while distance from robot to label is greater than 10cm            
            print("d: ", tag_pos_x_r-target_pos_x)
            ''' ====================  ADJUST ANGLE ===================='''
            #print("Readjusting angle with tag_id:  ", tag_id)
            #self.readjust_angle_with_quaternions(tag_id)                                          
            ''' ====================  MOVE FORWARD ===================='''    
            self.pub_joy.publish(joy_msg)  # Start moving                        
            ## MOVE INTERVALS OF 0.1s, CHECK DANGER ZONE, RESET SEEN TAGS
            self.current_seen_tags = {}
            time.sleep(0.2) # self.current_seen_tags must be populated during this movement
            
            self.check_danger_zone2(y_axis) 
            '''
            self.current_seen_tags = {}                         
            time.sleep(0.1)
            self.check_danger_zone2(y_axis) 
            self.current_seen_tags = {}
            time.sleep(0.1)
            self.check_danger_zone2(y_axis) 
            self.current_seen_tags = {}
            time.sleep(0.1)
            self.check_danger_zone2(y_axis) 
            self.current_seen_tags = {}
            '''
                                    
            tag_pos_x_r, tag_pos_y_r  = self.get_w_cord_for_tag(self.tags[tag_id])  # update tag_pos_x_r
        # stop robot
        joy_msg.axes[X] = 0 # reset 
        self.pub_joy.publish(joy_msg) # todo: maybe decrease time of samplig on d to get to target so it doesnt hit target
        print("Arrived!! d: ", tag_pos_x_r-target_pos_x)


    def move_front_or_back(self, target_position_w):
        '''
        Args:
        target_position_w -> (x,y) target on Map frame to where we want to move 
        '''
        def get_distance(target_position_w):            
            ''' Assuming no diag movements
            target_position_w: Expecting (x,y) in map coordinate
            '''
            delta_x, delta_y = self.get_deltas(self.get_current_pos(), target_position_w)
            y_axis=False
            if abs(delta_x) > 0.01:
                distance = delta_x
            elif abs(delta_y) > 0.01:        
                distance = delta_y
                y_axis=True
            else:
                raise "delta_x and delta_y are less than 1cm: {} , {}".format(delta_x, delta_y)               
            
            return distance, y_axis


        distance, y_axis = get_distance(target_position_w)
        if distance > 0:
            print("--------------------------")
            print("Moving forward")
        else:
            print("--------------------------")
            print("Moving backwards")
        self.move_front_no_tag(distance)

        #update
        if y_axis:
            self.y_w += distance
        else:
            self.x_w += distance
       
    def move_front_no_tag(self, d):
        '''
        Args:
        d -> int type represeting meters
        '''
        joy_msg = self.get_joy_msg()
        print("[move_front] Moving forward for {}m".format(d))
        #time_per_m = 2.0408   # [seconds to get to a meter] ---> TODO: Calibrate velocity moving front and moving backwards
        time_per_m = 5.8
        t_start = time.time()

        joy_msg.axes[X] = 0.6 if d> 0 else -0.6

        while time.time() < t_start + time_per_m*abs(d):
            self.pub_joy.publish(joy_msg)
            
            ''' CHECK DANGER ZONE'''
            self.current_seen_tags = {}
            time.sleep(0.1) # self.current_seen_tags must be populated during this movement
            if self.check_danger_zone():
                break

        joy_msg.axes[X] = 0 # reset 
        self.pub_joy.publish(joy_msg)
         

    def get_rads(self, theta):
        return theta - self.theta_w

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
    
    def run_rotation_calibration(self):
        joy_msg = Joy()
        joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        #target_time = 2.1   # [seconds to get to a 90 degrees angle]
        #target_time = 2.3   # [seconds to get to a 90 degrees angle]
        target_time = 2.1

        ''' 
        NOTE:
        On carpet...
        
        2.3s at 0.9 works great for positive turns
        2.1s at -0.9 for negative turns
        
        '''

        # ideal: target_time = rad / (speed [rad/s])
        t_start = time.time()
        joy_msg.axes[THETA] = -0.9 #1 # >0.1
        while time.time() < t_start + target_time:
            self.pub_joy.publish(joy_msg)
            # just wait for target_time          

        joy_msg.axes[THETA] = 0 # reset 
        self.pub_joy.publish(joy_msg)

        self.stop()

    def turn_90(self, left=True):
        joy_msg = self.get_joy_msg()
        # time_per_rad = 2.3/ (math.pi/2)
        #time_per_rad = 2.8/ (math.pi/2)
        time_per_rad = 2.3/ (math.pi/2)

        t_start = time.time()
        
        if left:
            joy_msg.axes[THETA] = 0.8
        else:    
            joy_msg.axes[THETA] = -0.8

        while time.time() < t_start + time_per_rad*np.pi/2:
        # while time.time() < t_start + time_per_rad*angle:
            self.pub_joy.publish(joy_msg)
            # just wait for target_time          
        joy_msg.axes[THETA] = 0 # reset 
        self.pub_joy.publish(joy_msg)
        self.current_seen_tags = {}

        time.sleep(1)


    def check_danger_zone(self):
        '''
        This function checks each tag seen so far in self.current_seen_tags, if any of the tags are too close (within THRESOLD), then return True.
        '''

        for tag_i in self.current_seen_tags:
            tag_pos_x_r, tag_pos_y_r  = self.get_w_cord_for_tag(self.current_seen_tags[tag_i]) 
            if abs(tag_pos_x_r) <= THRESHOLD and abs(tag_pos_y_r) <= THRESHOLD:
                print("{} in Danger Zone!".format(tag_i))
                return True
            
        return False

            
    ''' NEW FUNCTIONS '''

    def match_tag(self, input_tag):
        '''
        input_tag: coordinate of tag in Map frame 
        '''
        distance = 99999
        index = -1
        for i in range(len(self.tags)):
            curr_distance = (self.tags[i][0]-input_tag[0])**2 + (self.tags[i][1]-input_tag[1])**2
            if curr_distance<distance:
                distance=curr_distance
                index=i
        return self.tags[index]

    def get_H(self):
        ''' Get Homography matrix that maps 2D homogeneous coordinates in Robot frame to Map frame '''

        H = [   [1.0*np.sin(self.theta_w), 1.0*np.cos(self.theta_w) , self.x_w], 
                [1.0*np.cos(self.theta_w),  1.0*np.sin(self.theta_w)     , self.y_w], 
                [0                   ,  0                        , 1], 
            ]       
        return H

    def transform_from_R_to_M(self, robot_coord):
        ''' Transform from Robot frame to Map frame'
        robot_coord: Expected to be a 2D homogeneous coordinate in  Robot frame (np array of shape (3,1))
        '''
        assert robot_coord.shape == (3,1), " Expecting shape of (3,1) "
        H = self.get_H()
        map_coord = np.dot(H,robot_coord)
        map_coord = map_coord/map_coord[2,0]
        
        assert map_coord.shape == (3,1) and map_coord[2,0] == 1

        return map_coord

    def adjust_xy(self, LM_coord_reading, LM_ground_truth):
        '''
        Args:
            LM_coord_reading: np array of shape (3,1)  in map frame
            LM_ground_truth:  np array of shape (3,)  in map frame
        Adjusts X and Y coordinates of the robot (self.x_w and self.y_w) based on the error from the tag reading and groundtruth         
        '''        
        assert LM_coord_reading.shape == (3,1) and  LM_ground_truth.shape == (3,), "Wrong Shape"
        if LM_coord_reading[2,0] != 1:
            LM_coord_reading = LM_coord_reading/LM_coord_reading[2,0] 
           
        ''' Assuming +X axis points upwards and +Y points to the right ''' 
        delta_x =  LM_coord_reading[0,0] - LM_ground_truth[0]
        delta_y =  LM_coord_reading[1,0] - LM_ground_truth[1]

        
        self.x_w -= delta_x
        self.y_w -= delta_y

        #self.mark_map()

    def mark_map(self):
        '''' To be developed .... '''
        MAP[self.x_w, self.y_w] = 1

    def rotate_and_adjust(self, target_position_w):

        delta_x, delta_y = self.get_deltas(self.get_current_pos(), target_position_w)        
        # y_axis=True
        if delta_y > 0:
            self.turn_90()
            self.move_front_no_tag(delta_y)
            self.turn_90(self, left=True)
        else:
            self.turn_90(self, left=True)
            self.move_front_no_tag(delta_y)
            self.turn_90()
        #update        
        self.y_w += delta_y
        


    def run(self, next_action):
        # previous arguments: target_position_w, tag_id, robot_pos = (0.0,0.0), angle_for_short=0)
        '''
        Args:
        target_position_w -> Target position in world coordinates ("F", (37,2)) --> First postion is F, B, or R. Second position is point to where we are moving (x,y). 
        tag_id -> unique identifier for tag associaetd to the target position 
        
        '''              

        print("Robot's World Position: ", self.get_current_pos())
        print("Target Position: ", target_position_w)

        rospy.Subscriber("/tf", TFMessage, self.tag_information)
        time.sleep(2)



        state, target_position_w = next_action

        print("Navigating from {} --> {}".format((self.x_w,self.y_w, self.theta_w), target_position_w))
        print("--------------------------")
        print("--------------------------")
        print(" Next Action : ", next_action )
        if state == "F" or state == "B":
            ''' Move forward '''
            print("--------------------------")
            print("Moving forward")
            self.move_front_or_back(target_position_w)

        elif state == "R":
            print("--------------------------")
            print("Rotating")
            self.rotate_and_adjust(target_position_w)

        print("State: ", (self.x_w, self.y_w, self.theta_w))
        self.stop()

    def run_bakup(self, target_position_w):
        # previous arguments: target_position_w, tag_id, robot_pos = (0.0,0.0), angle_for_short=0)
        '''
        Args:
        target_position_w -> Target position in world coordinates (x,y)
        tag_id -> unique identifier for tag associaetd to the target position 
        
        '''              

        print("Robot's World Position: ", self.get_current_pos())
        print("Target Position: ", target_position_w)

        rospy.Subscriber("/tf", TFMessage, self.tag_information)
        time.sleep(2)



        # x_tag = self.transform_from_R_to_M( self.current_seen_tags )
        # x_tag_GT = match_tag(x_tag)
        # self.adjust_xy(x_tag, x_tag_GT)

        # do a prior turn
        # self.turn(angle_for_short)
        
        delta_x, delta_y = self.get_deltas(self.get_current_pos(), target_position_w)
        print("Navigating from {} --> {}".format((self.x_w,self.y_w, self.theta_w), target_position_w))
        print("delta_x: ", delta_x, "delta_y: ", delta_y)

        if abs(delta_x) > 0.1 and abs(delta_y) > 0.1:
            ''' ----------------  MOVE DIAG ----------------  '''            
            print("Move Diag!")
            print("Move on x: {} and move on y: {}".format(delta_x, delta_y))

            # first find tag associated to target

            # Now we see tag
            # assming robot will readjust angle before moving forward, the distance to move forward ideally is: 
            # self.readjust_angle_with_quaternions(tag_id)
            time.sleep(1)
            d = math.sqrt(delta_x**2 + delta_y**2)
            print("Distance to travel: ", d)            
            self.turn(self.theta_w - np.arctan2(delta_x, delta_y ))
            if delta_x < 0 and delta_y < 0 and self.theta_w < np.pi:
                print("moving backwards!!!")
                self.move_front(-1.0*d, moving_diag=True, diag_update=(delta_x, delta_y))
            else:
                self.move_front(d, moving_diag=True, diag_update=(delta_x, delta_y)) # front in direction of x axis (world coordinate)
            time.sleep(1)                   
        elif abs(delta_x) > 0.1:
            ''' ----------------  MOVE ON X AXIS ----------------  ''' 
            # if the robot is not at zero degrees, then rotate to make it zero
            print("Turning to zero degrees...")
            self.turn(0)
            self.move_front(delta_x) # front in direction of x axis (world coordinate)
            time.sleep(1)            
            # _, delta_y, _ = self.get_deltas(self.get_current_pos(), target_position_w) #  UPDATED VALUE AFTER MOVING ON X    
        elif abs(delta_y) > 0.1:        
            # move Y axis
            ''' ----------------  MOVE ON Y AXIS ----------------  '''        
            print("delta_y: ", delta_y)
            self.move_sideways_no_slide(delta_y)
            time.sleep(1)
                        
        #_, _, delta_theta = self.get_deltas(self.get_current_pos(), target_position_w) # UPDATED VALUE AFTER MOVING ON X AND Y                                 
        # move angle
        # if abs(delta_theta)  > 0.1:
        #     print("delta_theta: ", delta_theta)
        #     ''' MOVE ON THETA '''                    
        #     self.turn(target_position_w[2])
        #     # We can use pitch angle at this point to readjust turn 
        #     # Robot should be 90deg with respect the coordinate frame of tag 2
        #     time.sleep(1)
        ''' ----------------------------------------------------------------'''
        print("State: ", (self.x_w, self.y_w, self.theta_w))
        self.stop()


    def print_TAG_info(self,  tag_id):
        print("Robot's World Position: ", self.get_current_pos())

        # Obtain Tag information
        rospy.Subscriber("/tf", TFMessage, self.tag_information)
        
        t_start = time.time()
        t_experiment = 5 # [s]
        while time.time() < t_start + t_experiment:
            if tag_id in self.tags:
                print("Tag info: \n",self.tags[tag_id])
                time.sleep(1)
            elif self.tags:
                print("Different Tags observed: ", list(self.tags.keys()) )

    def print_rot_ang_from_tag(self,  tag_id):
        from geometry_msgs.msg import Quaternion
        import tf 
        
        # From experiment placing the robot right in front of the tag
        tag2_q = [-0.07681572469557221, 0.030113621272255503, -0.010630514604218507, 0.9965337457470342]
        tag1_q = [-0.031684626256729034, 0.05936072671254348, 0.6980881821665746, 0.7128430952899086]
        tag3_1 = [-0.19803038269123838, -0.019045190574983963, -0.7039578667645559, 0.681809775573378]
        tag_q_dict = {'marker_1': tag1_q, 'marker_4':tag2_q, 'marker_2':tag3_1}
        # tag2_q = Quaternion(tag2_quat_tf[0], tag2_quat_tf[1], tag2_quat_tf[2], tag2_quat_tf[3])
        print("tag2_q: ", tag2_q, type(tag2_q))
        
        time.sleep(0.1)
                
        assert tag_id in tag_q_dict, 'unexpected tag_id for quaternions'
        q2 = tag_q_dict[tag_id]

        # Obtain Tag information
        rospy.Subscriber("/tf", TFMessage, self.tag_information)
        
        t_start = time.time()
        t_experiment = 10 # [s]
        while time.time() < t_start + t_experiment:
            if tag_id in self.tags:
                # print("Tag info: \n",self.tags[tag_id])
                # print("tag2_q: ", tag2_q,  type(tag2_q))

                '''
                Say you have two quaternions from the same frame, q_1 and q_2. You want to find the relative rotation, q_r, to go from q_1 to q_2:
                q_2 = q_r*q_1
                You can solve for q_r similarly to solving a matrix equation. Invert q_1 and right-multiply both sides. Again, the order of multiplication is important:
                q_r = q_2*q_1_inverse
                '''
                q1 = list(self.tags[tag_id]['rotation'])
                q1_inv = q1
                q1_inv[3] = -q1_inv[3] 
                qr = tf.transformations.quaternion_multiply(q2, q1_inv)
                (roll, pitch, yaw) = euler_from_quaternion (qr) # from tf.transformations
        
                print("(roll, pitch, yaw): ", (roll, pitch, yaw))
                time.sleep(1)

            elif self.tags:
                print("Different Tags observed: ", list(self.tags.keys()) )   
         

if __name__ == "__main__":
    roomba_node = RoombaNode()
    rospy.init_node("roomba")

    # points = get_points_from_file()
    # print(points)
    # points = [(0,0,0),(1,0,0),(1,1,1.57),(2,1,0),(2,2,-1.57),(1,1,-0.78),(0,0,0)]
    ''' Calibrate'''
    # roomba_node.run_rotation_calibration()
    ''' -----'''
    # roomba_node.move_front_no_tag(1)
    # roomba_node.move_front_no_tag(-1)

    roomba_node.turn_90()

    # midpoint = mapping_shortest_dist()
    # points = [midpoint[:2], (1.0, 1.0)]
    # #points = [(1.5, 0.0), (1.5, 1.5)]     # no angle required to get to these positions
    # '''
    # marker_1: LM4 in the map
    # marker_7: LM8 in the map
    # '''
    # tags = ["marker_1","marker_7"] # tag ids associated to each position

    # '''
    # Getting Tag info
    # '''
    # # feedback_node.print_rot_ang_from_tag(tags[2])
    # # roomba_node.print_TAG_info( tags[1])
    # '''
    # Running Experiment
    # '''
    # # print("Starting navigation to target point: ", p, " tag: ", tag_id)        
    # # roomba_node.run(p, tag_id, robot_pos= (0.7,1.4,np.pi) )
    
    # '''
    # Try this next    
    
    # for p in plan_path()[:4]:        
    #     print("======================================================================")
    #     print("Starting navigation to target point: ", p)               
    #     roomba_node.run(p)
    
    # test_points = [("F", (37,2)), ("B", (2,2)), ("R", (2,4)),\
    #                 ("F", (37,4)), ("B", (2,4)), ("R", (2,6)),\
    #                 ("F", (37,6)), ("B", (2,6)), ("R", (2,8)) 
    #                 ]
    # for p in test_points:        
    #     print("======================================================================")
    #     print("Starting navigation to target point: ", p)               
    #     roomba_node.run(p)

    # roomba_node.run()
