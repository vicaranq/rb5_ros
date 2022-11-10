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
import re

X = 1
Y = 0 
THETA = 2

class KalmanNode:
    def __init__(self):
        self.pub_joy = rospy.Publisher("/joy", Joy, queue_size=1)
        self.settings = save_terminal_settings()

        # Initialize position of robot 
        # (assuming -x to front, +y to the left, theta opening from -x towards y axis)
        self.x_w = 0
        self.y_w = 0
        self.theta_w = 0

        # current reading of tags
        self.tags = {} #np.zeros((30,1))
        # system state
        self.state = np.zeros((3,1))
        self.system_matrix_F = np.identity(33) 
        # it is really G*di, but since G is Identity matrix we put them together
        self.control_matrix_G = np.zeros((3,1)) # Victor: Has to change according to the state size 
        
        # calculated every time
        self.H = np.zeros((30,33)) 
        # covariance matrix, initialized to 0.01
        self.P = np.identity(3)/100 # Victor: This might be too low (this numbers kind of assumes a system that is around 0.6deg and 1cm accurate, may affect the update)
        self.P[2:2] = 0.0003
        # noise at 0.01^2
        self.R = np.identity(30)/10000

        self.cache_P = [] # update covarience cache every time step
        self.cache_states = [] # update state cache every time step

        self.tagId_to_idx = []
        self.seen_tags = set() # Set containing tags from tagId_to_idx

        # self.prev_seen_tags = {} # dictcontaining previously seen tag and its 'x', 'y', and 'theta' in the map frame
        self.current_seen_tags = {} # dictcontaining previously seen tag and its 'x', 'y', and 'theta' in the map frame


    def get_current_pos(self):
        return ( self.x_w, self.y_w, self.theta_w)

    def get_deltas(self, curr_pos, target_pos):
        ''' curr_pos=(y,x,theta) , target_pos=(x,y,theta)'''
        delta_x = target_pos[0] - curr_pos[0]
        delta_y = target_pos[1] - curr_pos[1]
        delta_theta = target_pos[2] - curr_pos[2]
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
        # self.tags = np.zeros((30,1))
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
                num = int(re.findall(r'\d+', tag_id)[0])
                num = num-1
                assert type(num) == int, "Unexpected extracted tag id type"
                # assert num <= 9, " unexpected tag number"
                qr = self.get_rotation(message)
                (roll, pitch, yaw) = euler_from_quaternion(qr) 

                # self.tags will have the HISTORICAL information of all targs seen before 
                self.tags[tag_id] = {
                                    "x" : self.get_translation(message)[2] , 
                                    "y":self.get_translation(message)[0], 
                                    "theta": pitch, 
                                    "pos": np.array([[self.get_translation(message)[2]], 
                                                     [self.get_translation(message)[0]],                                                                                                                                 
                                                     [pitch ]
                                                    ])
                                    }
                # self.current_seen_tags will act similarly as self.tags, however it cleared every iteration (step in the experiment e.g. every 0.1m or turn)
                self.current_seen_tags[tag_id] = {"x" : self.get_translation(message)[2] , \
                                        "y":self.get_translation(message)[0], \
                                            "theta": pitch}
                # # x and y reading from tag
                # self.tags[num*3]=self.get_translation(message)[2] # assuming z-axis from tag aligns with x-axis from robot
                # self.tags[num*3+1]=self.get_translation(message)[0] # assuming x-axis from tag aligns with y-axis from robot
                # # pose angle of tag
                # # note this angle is 1/2cos(theta), where theta is the pose rotation of tag
                # self.tags[num*3+2]=self.get_rotation(message)[0] # Victor: angle of rotation from x-axis in tag?
                # print("tags updated!")
            except:
                print("something fail")
                raise
                  

    def stop_robot(self):
        # reset 
        joy_msg = self.get_joy_msg()        
        joy_msg.axes[X] = 0 
        joy_msg.axes[Y] = 0 
        joy_msg.axes[THETA] = 0 
        self.pub_joy.publish(joy_msg)

      

    def get_w_cord_for_tag(self, tag_pos_T):
        X, Y, Z = (0,1,2)
        # tag_pos_x_w, tag_pos_y_r = (tag_pos_T['translation'][Z], -1*tag_pos_T['translation'][X]) # distance to x location in world coord.
        # NOTE: change this depending on the tag! 
        return (tag_pos_T['translation'][Z], tag_pos_T['translation'][X]) # distance to x location in world coord.

    def move_sideways_no_slide(self, y, tag_id, joy_msg):
        ''' function to move robot on the y-axis using rotation instead of sliding'''
        print("[move_sideways_no_slide] Movign sideways for {}m".format(y))
        # If moving to the left, first turn depending of sign of y then move for abs(y) meters to the front
        if y > 0:
            print("Turning 90deg -> {}rads".format(math.pi/2 - self.theta_w))
            self.turn_v2(math.pi/2, joy_msg) # turn left 90deg
        elif y < 0:
            print("Turning -90deg")
            self.turn_v2(-math.pi/2, joy_msg) # turn right 90 deg
        time.sleep(1)
        print("Move front for {}m".format(abs(y)))            
        self.move_front_old(y, tag_id, y_axis=True)

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
        tag2_q = [-0.07681572469557221, 0.030113621272255503, -0.010630514604218507, 0.9965337457470342]
        tag1_q = [-0.031684626256729034, 0.05936072671254348, 0.6980881821665746, 0.7128430952899086]
        #tag3_1 = [-0.19803038269123838, -0.019045190574983963, -0.7039578667645559, 0.681809775573378] # facing straight
        # tag3_1 = [0.012554685868491552, -0.15974074559935636, -0.7087651669450613, 0.68700597681784740]# facing diagonal at tag 3
        tag3_1 = [-0.0766701086000425, -0.09950813695988033, -0.7134872517193238, 0.6893154334265537] # mixed of above two
        tag_q_dict = {'marker_1': tag1_q, 'marker_4':tag2_q, 'marker_2':tag3_1}
        assert tag_id in tag_q_dict, "Unexpected marker in quatrernion dict"

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
            joy_msg = self.get_joy_msg()
            self.turn_v2(pitch + self.theta_w, joy_msg) # readjusting to angle coordinates +CW and -CCC
            time.sleep(0.2)   
        return pitch


    def turn_v2(self, theta, joy_msg, scale = False):
        '''
        theta: angle in radiants to where we want to turn 
        '''
        # From calibration tests:
        time_per_rad = 2.3/ (math.pi/2) if theta >=0 else 2.1/ (math.pi/2)

        t_start = time.time()
        rads_to_turn = self.get_rads(theta)
        # rads_to_turn has to be within 2pi range
        # sign = -1 if rads_to_turn >=0 else 1
        # rads_to_turn = abs(rads_to_turn) % (2*math.pi)
        # rads_to_turn = sign*rads_to_turn

        # joy_msg.axes[THETA] = 1.1 if rads_to_turn >= 0 else -1.1# >0.1
        joy_msg.axes[THETA] = 0.9 if rads_to_turn >= 0 else -0.9# >0.1
        # if scale:
        #     # used for angle readdjustment
        #     joy_msg.axes[THETA] = 0.5 if rads_to_turn >= 0 else -0.5# >0.1

        while time.time() < t_start + time_per_rad*abs(rads_to_turn):
            self.pub_joy.publish(joy_msg)
            # just wait for target_time          
        joy_msg.axes[THETA] = 0 # reset 
        self.pub_joy.publish(joy_msg)
        
        # theta_w has to be within 2pi range   
        sign = -1 if theta >=0 else 1    
        theta_mod =  abs(theta) %  (2*math.pi)
        self.theta_w =  sign*theta_mod
        print("[turn] theta updated to {} and turned {}rads".format(self.theta_w, rads_to_turn))
        self.stop()

    def move_with_tag(self, d, tag_id, y_axis=False, moving_diag=False):
        
        
        joy_msg = self.get_joy_msg()

        tag_pos_x_r, tag_pos_y_r  = self.get_w_cord_for_tag(self.tags[tag_id])

        target_pos_x = tag_pos_x_r - d

        
        print("[move_front] Moving forward for {}m".format(d))
        time_per_m = 2.0408   # [seconds to get to a meter] on carpet
        # time_per_m = 2.7027   # [seconds to get to a meter] on ceramic 
        t_start = time.time()

        #joy_msg.axes[X] = 1.2 if d >=0 or y_axis else -1.2 # >0.1   
        joy_msg.axes[X] = 1.0 if d >=0 or y_axis else -1.0 # >0.1  
        if moving_diag:
            print("Moving slower in diagonal")
            joy_msg.axes[X] = 0.8 if d >=0 else -0.8 # >0.1         

        # if d is within 20 cm, start reducing the speed
        #joy_msg.axes[X] = self.reduce_speed(d, joy_msg.axes[X])

        #while time.time() < t_start + time_per_m*abs(d):
        temp_dist = tag_pos_y_r
        while tag_pos_x_r-target_pos_x > 0.1:
            time.sleep(0.2) # wait to populate tag dict
            tag_pos_x_r, tag_pos_y_r  = self.get_w_cord_for_tag(self.tags[tag_id])
            print("d: ", tag_pos_x_r-target_pos_x)
            ''' ====================  ADJUST ANGLE ===================='''
            if tag_id in { "marker_4", "marker_1", "marker_2"}:
                print("Found ", tag_id)
                pitch = self.readjust_angle_with_quaternions(tag_id) 
            elif abs(temp_dist - tag_pos_y_r) > 0.05 and tag_pos_x_r-target_pos_x > 0.2:                        
                print("Didn't found marker_4")
                # self.readjust_angle(tag_pos_y_r, tag_pos_x_r)             
            # time.sleep(0.5)
            ''' ====================  MOVE FORWARD ===================='''    
            self.pub_joy.publish(joy_msg)            
            time.sleep(0.2)                                

        print("Arrived!! d: ", tag_pos_x_r-target_pos_x)
        joy_msg.axes[X] = 0 # reset 
        self.pub_joy.publish(joy_msg) # todo: maybe decrease time of samplig on d to get to target so it doesnt hit target


    def move_front_old(self, d,tag_id, y_axis=False, moving_diag=False, diag_update=(0,0)):
        '''
        Args:
        d -> float type represeting meters
        '''
        # time.sleep(1) 
        joy_msg = self.get_joy_msg()
        if tag_id in self.tags:
            time.sleep(1) 
            ''' NOTE: Encapsulate this code into function to use in the next else statement'''
            self.move_with_tag( d, tag_id, y_axis=False, moving_diag=moving_diag)

        else: 
            time_per_m = 2.7027   # [seconds to get to a meter]
            t_start = time.time()
            print("[move_front - no tag info] Moving forward for {}m".format(d))
            joy_msg.axes[X] = 1.2 if d >=0 or y_axis else -1.3 # >0.1   
            if moving_diag:
                joy_msg.axes[X] = 1.0 if d >=0 or y_axis else -1.1 # >0.1   

            while time.time() < t_start + time_per_m*abs(d):
                self.pub_joy.publish(joy_msg)
                ''' IF WE SEE THE TAG 3 AND WE ARE MOVING ON Y, then use the approach in the if statement above'''
                # time.sleep(1)
                # if tag_id == 'marker_2' and y_axis:
                #   self.move_with_tag( d, y_axis=False)
            joy_msg.axes[X] = 0 # reset 
            self.pub_joy.publish(joy_msg)
            time.sleep(1)  

        #update
        if moving_diag:
            assert diag_update != (0,0), " Unexpected diag update"
            self.x_w, self.y_w = diag_update
        elif y_axis:
            self.y_w += d
        else:
            self.x_w += d

    def move_front_new(self, d, y_axis=False):
        '''
        Args:
        d -> int type represeting meters
        '''
        joy_msg = self.get_joy_msg()
        print("[move_front] Moving forward for {}m".format(d))
        # time_per_m = 2.0408   # [seconds to get to a meter]
        time_per_m = 5.4

        t_start = time.time()

        #joy_msg.axes[X] = 1.2 if d >=0 or y_axis else -1.2 # >0.1         
        joy_msg.axes[X] = 0.6 if d >=0 or y_axis else -0.6 # >0.1         
        while time.time() < t_start + time_per_m*abs(d):
            self.pub_joy.publish(joy_msg)

        joy_msg.axes[X] = 0 # reset 
        self.pub_joy.publish(joy_msg)
       
    def get_rads(self, theta):
        return theta - self.theta_w

    def turn(self, theta, joy_msg):
        '''
        theta: angle in radiants to where we want to turn 
        '''
        #calibration_time = 2.5 # [sec/rad]time to get to pi/2
        time_per_rad = 2.6/ (math.pi/2)
        t_start = time.time()
        rads_to_turn = self.get_rads(theta)
        joy_msg.axes[THETA] = 1 if rads_to_turn >= 0 else -1# >0.1
        # while time.time() < t_start + time_per_rad*abs(rads_to_turn):
        self.pub_joy.publish(joy_msg)
        time.sleep(0.5)
            # just wait for target_time          
        joy_msg.axes[THETA] = 0 # reset 
        self.pub_joy.publish(joy_msg)
        # self.theta_w = theta
        print("[turn] theta updated and turned {}rads".format(rads_to_turn))
        self.stop()

    def stop(self):
        restore_terminal_settings(self.settings)
  
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
      

    def turn_90(self):
        joy_msg = self.get_joy_msg()
        # time_per_rad = 2.3/ (math.pi/2)
        time_per_rad = 2.8/ (math.pi/2)

        t_start = time.time()
        # joy_msg.axes[THETA] = 0.8
        joy_msg.axes[THETA] = -0.8
        while time.time() < t_start + time_per_rad*np.pi/2:
            self.pub_joy.publish(joy_msg)
            # just wait for target_time          
        joy_msg.axes[THETA] = 0 # reset 
        self.pub_joy.publish(joy_msg)
        self.current_seen_tags = {}
        time.sleep(1)

    def turn_45(self, joy_msg):
        time_per_rad = 2.3/ (math.pi/2)
        t_start = time.time()
        joy_msg.axes[THETA] = 0.9
        while time.time() < t_start + time_per_rad*np.pi/4:
            self.pub_joy.publish(joy_msg)
            # just wait for target_time          
        joy_msg.axes[THETA] = 0 # reset 
        self.pub_joy.publish(joy_msg)

    def save_data(self):
        self.cache_P.append(self.P)
        self.cache_states.append(self.state)

    def write_saved_data(self):

        # save states -> hardcoding 33 dims so -> 33xsamples
        np.savetxt("states_cache.csv", 
           np.array(self.cache_states),
           delimiter =", "
           )
        # save covariances samplesx33x33
        covs_cache = np.array(self.cache_states)
        covsReshaped = covs_cache.reshape(covs_cache.shape[0], -1) 
        np.savetxt("covs_cache.csv", 
           covsReshaped,
           delimiter =", "
           )
        with open('cov_original_shape.txt', 'w') as f:
            f.write(str(covs_cache.shape))
        # NOTE: 
        # To load: 
        # loadedArr = np.loadtxt(filename)
        # loadedOriginal = loadedArr.reshape(x, y, z) where x,y,z come from cov_original_shape.txt
    def update_H(self, num):
        self.H = np.zeros((num*3,num*3+3))
        transformation_matrix = np.zeros((3,3))
        transformation_matrix[0][0] = np.cos(self.state[2,0])
        transformation_matrix[0][1] = 1.0*np.sin(self.state[2,0])  
        transformation_matrix[1][0] = -1.0*np.sin(self.state[2,0]) 
        transformation_matrix[1][1] = np.cos(self.state[2,0])
        transformation_matrix[2][2] = 1
        for k in range(0,num*3,3):
            self.H[k:k+3,0:3] = -1.0*transformation_matrix
        
        # fill transformation matrix for tags -> filling diagonally
        for k in range(0,num*3,3):
            self.H[k:k+3,k+3:k+6] = transformation_matrix

    def update_G(self, i ):
        self.control_matrix_G = np.zeros((33,1))
        if i==0:
            self.control_matrix_G[0] = 0.1
        elif i==1:
            self.control_matrix_G[1] = 0.1
        elif i==2:
            self.control_matrix_G[0] = -0.1
        elif i==3:
            self.control_matrix_G[1] = -0.1  

    def get_idxs_of_seen_tags(self):
        
        markers = self.current_seen_tags.keys()
        result = []
        unseen = []
        for marker in markers:
            if marker in self.seen_tags:
                for i in range(len(self.tagId_to_idx)):
                    if self.tagId_to_idx[i] == marker:
                        result.append(i)
            else:
                unseen.append(marker)

        return result, unseen

    def get_sliced_cov(self, num, idxs_seen ):  

        local_P = np.zeros((num*3+3, num*3+3))
        # fill first 3x3 with Robot state
        local_P[:3,:3]=self.P[:3,:3] 
        # Get the sliced diagonal from self.P corresponding to the local_P based on the seen indices (e.g. tag with index 0 would mean the first tag we ever saw)
        for idx in range(len(idxs_seen)):
            assert idxs_seen[idx] >= 0 and idxs_seen[idx] < len(self.P), " Unexpected index: {}, for P shape: {}".format(idxs_seen[idx], self.P.shape )
            local_P[idx*3+3:idx*3+6, idx*3+3:idx*3+6] = self.P[idxs_seen[idx]*3+3:idxs_seen[idx]*3+6, idxs_seen[idx]*3+3:idxs_seen[idx]*3+6]
        return  local_P   

    def get_sliced_state(self, num, idxs_seen):        
        local_state = np.zeros((num*3+3,1)) # num of seen tags (x,y,theta) plus robot's (x,y,theta)
        local_state[:3] = self.state[:3] # robot
       
        for idx in range(len(idxs_seen)):
            assert idxs_seen[idx] >= 0 and idxs_seen[idx] < len(self.state), " Unexpected index: {}, for state length: {}".format(idxs_seen[idx], len(self.state) )
            assert idx*3+6 <= local_state.shape[0], "Unexpected idx on local_state: {} for local_state shape:{}".format(idx*3+6, local_state.shape)
            #print("---")
            #print("idxs_seen[idx]*3+3: ", idxs_seen[idx]*3+3)
            #print(self.state[idxs_seen[idx]*3+3:idxs_seen[idx]*3+6])
            #print("self.state: ", self.state.shape)
            local_state[idx*3+3:idx*3+6] = self.state[idxs_seen[idx]*3+3:idxs_seen[idx]*3+6]
        return local_state

    def update_state(self, updated_local_state, idxs_seen):
        self.state[:3] = updated_local_state[:3]
        for idx in range(len(idxs_seen)):
            self.state[idxs_seen[idx]*3+3:idxs_seen[idx]*3+6] = updated_local_state[idx*3+3:idx*3+6]

    def update_global_P(self, local_P, idxs_seen):
        for idx in range(len(idxs_seen)):
            self.P[idxs_seen[idx]*3+3:idxs_seen[idx]*3+6, idxs_seen[idx]*3+3:idxs_seen[idx]*3+6] = local_P[idx*3+3:idx*3+6, idx*3+3:idx*3+6]

    def get_f(self, idxs_seen):
        ''' gets f vector with measurements from current seen tags (x,y,theta,x,y,theta,...)'''
        res = 1
        print('---')
        for i in idxs_seen:
            marker = self.tagId_to_idx[i]            
            print("Finding info for marker: ", marker)
            marker_pos = self.tags[marker]["pos"] # (3,1)
            print("Marker's pos", marker_pos)
            # print("res:", res)
            if type(res) == int:
                res = marker_pos
            else: 
                res = np.concatenate((res,marker_pos))
        # print('---')
        return res


    def run(self, robot_pos = (0.0,0.0,0.0)):
        '''
        Args:
        target_position_w -> Target position in world coordinates 
        tag_id -> unique identifier for  tag associaetd to the target position (1m away from actual target)
        
        '''              
        if robot_pos != (0.0,0.0,0.0):
            self.x_w, self.y_w, self.theta_w = robot_pos
        print("Robot's World Position: ", self.get_current_pos())
        # print("Target Position: ", target_position_w)

        rospy.Subscriber("/tf", TFMessage, self.tag_information)
        time.sleep(3)

        # NOTE: Move front 0.1m 10 times, at each step predict and update using Kalman's filter, then turn 90deg and do the same 
        for i in range(2):
            print('================================= i: {} ========================'.format(i))
            for _ in range(10): #10
                # move forward 0.1m
                self.move_front_new(0.1) # front in direction of x axis (world coordinate)
                time.sleep(1)
                '''
                Kalman update for distance
                '''
                exit_early = False
                # first update state
                self.control_matrix_G = np.zeros((3,1))
                if i==0:
                    self.control_matrix_G[0] = 0.1
                elif i==1:
                    self.control_matrix_G[1] = 0.1
                    exit_early = True
                elif i==2:
                    self.control_matrix_G[0] = -0.1
                elif i==3:
                    self.control_matrix_G[1] = -0.1
                # self.update_G(i)

                # Update robot state, future label states are expected not to move so G is 0 for tags
                self.state[:3] = self.state[:3] + self.control_matrix_G[:3]
                # self.control_matrix_G = np.zeros((33,1))
                
                # if we see tags since the last iteration                
                if self.current_seen_tags:
                    
                    idxs_seen, unseen_IDs = self.get_idxs_of_seen_tags()                    
                    if idxs_seen:
                        print(" -------- SEEN IDs ----------")
                        ''' UPDATE '''
                        # calculate H
                        print("idxs_seen: ",idxs_seen)
                        num = len(idxs_seen)        # number of tags to update                        
                        self.update_H(num)
                        print("self.H: ")
                        print(self.H)

                        # construct covariance matrix P for this update
                        local_P = self.get_sliced_cov(num, idxs_seen)
                        print("self.P:")
                        print(self.P)
                        print("local_P:")
                        print(local_P)


                        # local state construction
                        local_state = self.get_sliced_state(num, idxs_seen)

                        print("Sliced local_state: ")
                        print(local_state)
                        print(type(local_state))

                        # calculate Kalman filter gain
                        R = np.identity(num*3)/10000

                        S = np.dot(np.dot(self.H,local_P) , np.transpose(self.H) ) + R
                        K = np.dot( np.dot(local_P, np.transpose(self.H)) ,  np.linalg.inv(S) )

                        f = self.get_f(idxs_seen)
                        print("f:")
                        print(f)
                        # update state
                        local_state = local_state + np.dot(K, (f - np.dot(self.H, local_state) ))

                        print("updated local_state: ")
                        print(local_state)
                        print(type(local_state))

                        self.update_state(local_state, idxs_seen)                                    
                        
                        # update covariance
                        local_P = np.dot( (np.identity(num*3+3)- np.dot(K,self.H) ) , local_P )
                        
                        #print("local_P:")
                        #print(local_P)

                        self.update_global_P(local_P, idxs_seen)

                        print("self.P: ")
                        print(self.P)

                    if unseen_IDs:
                        # new tags found and they are not in the state
                        print(" -------- UNSEEN IDs ----------")

                        for marker in unseen_IDs:

                            x_r = self.tags[marker]['x']
                            y_r = self.tags[marker]['y']
                            theta_r = self.tags[marker]['theta']
                            # print("theta_map: ", theta_map , type(theta_map))
                            theta_map = self.state[2,0] # from robot
                            print("theta_map: ", theta_map , type(theta_map))
                            H_ = np.array( [
                                [np.cos(theta_map), -1*np.sin(theta_map), self.state[0] ],
                                [np.sin(theta_map), np.cos(theta_map)   , self.state[1] ],
                                [0                , 0                   , 1]
                            ])

                            XY_map = np.dot(H_, np.array([ [x_r],[y_r],[1]]))
                            # NOTE: XY_map[0] gives an array like array([ 0.99263714])
                            S_to_add = np.array( [ XY_map[0][0]  , XY_map[1][0], theta_r + theta_map ] )
                            S_to_add = S_to_add[:,np.newaxis]
                            
                            print('New state to add for Tag {}:'.format(marker))
                            print(S_to_add)
                            print('Tag info from sensor:')
                            print(self.tags[marker])
                            
                            # Extend State
                            self.state = np.concatenate( (self.state, S_to_add) )
                            # Extend Covariance Matrix
                            temp_P = np.identity(len(self.P)+3)/100
                            temp_P[-1,-1] = 0.00001
                            temp_P[:len(self.P), :len(self.P)] = self.P
                            self.P = temp_P
                            
                            # Add marker to seen markers and to tagsId map
                            self.tagId_to_idx.append(marker)
                            self.seen_tags.add(marker)

                    # RESET CURRENT SEEN TAGS
                    self.current_seen_tags = {} # NOTE: It will populate at the backend by the self.tag_information which reads info from TF tags
                
                print("Robot state:")
                for i in range(0,len(self.state),3):

                    tag_idx = i // 3 
                    if tag_idx == 0:
                        # robot
                        marker_name = 'ROBOT'
                    else:
                        assert tag_idx-1 >= 0 and tag_idx-1 < len(self.tagId_to_idx), "Wrong tag_idx! "
                        marker_name = self.tagId_to_idx[tag_idx-1]
                    
                    print(marker_name , " -->  ({},{},{})".format(self.state[i][0], self.state[i+1],self.state[i+2] ) )

                if exit_early:
                    break
                # Save State and Covariance Data
                self.save_data()
            if exit_early:
                break

            self.turn_90()
            time.sleep(1)
            print("Updating theta to: ")
            print(self.state[2]+np.pi/2)
            self.state[2] = self.state[2]+np.pi/2 # Victor: mod 2pi if we do more loops in other experiments 
        '''
        for i in range(len(self.state)):
            print(i , " --> ", self.state[i])
        '''

        # right saved states and covariances to file
        #self.write_saved_data()
        '''
        8 point motion
        '''
        '''
        for i in range(8):
            for j in range(10):
                # move forward 0.1m
                self.move_front_new(0.05) # front in direction of x axis (world coordinate)
                time.sleep(1)

                # first update state
                if i==0:
                    self.control_matrix_G[0] = 0.05
                    self.state = self.state+self.control_matrix_G
                    self.control_matrix_G = np.zeros((33,1))
                elif i==1:
                    self.control_matrix_G[0] = 0.5/np.sqrt(2)
                    self.control_matrix_G[1] = 0.5/np.sqrt(2)
                    self.state = self.state+self.control_matrix_G
                    self.control_matrix_G = np.zeros((33,1))
                elif i==2:
                    self.control_matrix_G[1] = 0.05
                    self.state = self.state+self.control_matrix_G
                    self.control_matrix_G = np.zeros((33,1))
                elif i==3:
                    self.control_matrix_G[0] = -0.05/np.sqrt(2)
                    self.control_matrix_G[1] = 0.05/np.sqrt(2)
                    self.state = self.state+self.control_matrix_G
                    self.control_matrix_G = np.zeros((33,1))
                elif i==4:
                    self.control_matrix_G[0] = -0.05
                    self.state = self.state+self.control_matrix_G
                    self.control_matrix_G = np.zeros((33,1))  
                elif i==5:
                    self.control_matrix_G[0] = -0.05/np.sqrt(2)
                    self.control_matrix_G[1] = -0.05/np.sqrt(2)
                    self.state = self.state+self.control_matrix_G
                    self.control_matrix_G = np.zeros((33,1))      
                elif i==6:
                    self.control_matrix_G[1] = -0.05/np.sqrt(2)
                    self.state = self.state+self.control_matrix_G
                    self.control_matrix_G = np.zeros((33,1))       
                elif i==7:
                    self.control_matrix_G[0] = 0.05/np.sqrt(2)
                    self.control_matrix_G[1] = -0.05/np.sqrt(2)
                    self.state = self.state+self.control_matrix_G
                    self.control_matrix_G = np.zeros((33,1))       

                # calculate H
                transformation_matrix = np.zeros((3,3))
                transformation_matrix[0][0] = np.cos(self.state[2])
                transformation_matrix[0][1] = -1.0*np.sin(self.state[2])
                transformation_matrix[1][0] = np.sin(self.state[2])
                transformation_matrix[1][1] = np.cos(self.state[2])
                transformation_matrix[2][2] = 1
                for k in range(0,30,3):
                    self.H[k:k+3,0:3] = -1.0*transformation_matrix
                
                # fill transformation matrix for tags
                for k in range(3,30,3):
                    self.H[k:k+3,k+3:k+6] = transformation_matrix

                # calculate Kalman filter gain
                K = self.P@np.transpose(self.H)@(self.H@self.P@np.transpose(self.H) + self.R)
                # update state
                self.state = self.state + K@(self.tags-self.H@self.state)
                # update covariance
                self.P = (np.identity(33)-K@self.H)@self.P

            self.turn_45(joy_msg)
            time.sleep(1)
            self.theta_w = self.theta_w+np.pi/4
        '''

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

         

if __name__ == "__main__":
    kalman_node = KalmanNode()
    rospy.init_node("kalman")

    # points = get_points_from_file()
    # print(points)
    # points = [(0,0,0),(1,0,0),(1,1,1.57),(2,1,0),(2,2,-1.57),(1,1,-0.78),(0,0,0)]
    ''' Calibrate'''
    # feedback_node.run_rotation_calibration()
    ''' -----'''
    # points = [(1.0,0.0,np.pi/2.0), (1.0,1.0,np.pi), (0.0,1.0,np.pi*1.5),(0.0,1.0,0.0) ]    
    #tags = ["marker_1","marker_4","marker_2"] # tag ids associated to each position
    #tags = ["marker_1","marker_4","marker_2"] # tag ids associated to each position
    #p, tag_id = (points[2], tags[2])
    '''
    Getting Tag info
    '''
    # feedback_node.print_rot_ang_from_tag(tags[2])
    # feedback_node.print_TAG_info( tags[2])
    '''
    Running Experiment
    '''
    # print("Starting navigation to target point: ", p, " tag: ", tag_id)        
    # feedback_node.run(p, tag_id, robot_pos= (0.7,1.4,np.pi) )
    
    '''
    Try this next    
    
    for p in points[:]:        
        print("======================================================================")
        print("Starting navigation to target point: ", p)        
        feedback_node.run(p)
    '''
    # kalman_node.run()
    # kalman_node.move_front_new(1)
    kalman_node.turn_90()

