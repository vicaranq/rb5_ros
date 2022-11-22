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


X = 1
Y = 0 
THETA = 2

class PlanningNode:
    def __init__(self):
        self.pub_joy = rospy.Publisher("/joy", Joy, queue_size=1)
        self.settings = save_terminal_settings()

        # Initialize position of robot 
        # (assuming -x to front, +y to the left, theta opening from -x towards y axis)
        # self.x_pos = 0
        # self.y_pos = 0
        self.x_w = 0
        self.y_w = 0
        self.theta_w = 0
        self.grid_unit = 0.05 # 5x5cm grid size
        self.tags = {}
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
                self.tags[tag_id]={"id": tag_id, \
                    "translation" : self.get_translation(message), \
                        "rotation" : self.get_rotation(message)}
                # print("tags updated!")

                # self.current_seen_tags will act similarly as self.tags, however it clears every iteration (step in the experiment e.g. every 0.1m or turn)
                self.current_seen_tags[tag_id] = {"x" : self.get_translation(message)[2] , \
                                                  "y":self.get_translation(message)[0], \
                                                  "id": tag_id, \
                                                  "translation" : self.get_translation(message), \
                                                  "rotation" : self.get_rotation(message)
                                                 }                
            except:
                print("something fail")
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

    def move_sideways_no_slide(self, y, tag_id):
        ''' function to move robot on the y-axis world frame using rotation instead of sliding'''
        print("[move_sideways_no_slide] Movign sideways for {}m".format(y))
        # If moving to the left, first turn depending of sign of y then move for abs(y) meters to the front
        if y > 0:
            print("Turning 90deg")
            self.turn(math.pi/2) # turn left 90deg
        elif y < 0:
            print("Turning -90deg")
            self.turn(-math.pi/2) # turn right 90 deg
        time.sleep(1)
        print("Move front for {}m".format(abs(y)))            
        self.move_front(y, tag_id, y_axis=True)
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
        tagLM4_q = [-0.05788680893984566, 0.03995771047955379, 0.6919213818811771, 0.718538307969475]
        # Q for landmark 8 --> 'marker_7'
        tagLM8_q = [0.022206495547235545, 0.11227388669211188, 0.9933441174357323, -0.01299654871046849]
        
        tag_q_dict = {'marker_1': tagLM4_q, 'marker_7':tagLM8_q} 
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
            heuristic_pitch = pitch*0.8
            self.turn(heuristic_pitch + self.theta_w) # readjusting to angle coordinates +CW and -CCC
            time.sleep(0.2)   


    def turn(self, theta, scale = False):
        '''
        theta: angle in radiants to where we want to turn in world coordinates! If theta = 0, then turn to where +x axis of the world/map coordinate is
        '''
        joy_msg = self.get_joy_msg()


        t_start = time.time()
        rads_to_turn = self.get_rads(theta)


        # From calibration tests:
        time_per_rad = 2.3/ (math.pi/2) if rads_to_turn >=0 else 2.1/ (math.pi/2)

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
            print("Readjusting angle with tag_id:  ", tag_id)
            self.readjust_angle_with_quaternions(tag_id)                                          
            ''' ====================  MOVE FORWARD ===================='''    
            self.pub_joy.publish(joy_msg)  # Start moving                        
            ## MOVE INTERVALS OF 0.1s, CHECK DANGER ZONE, RESET SEEN TAGS
            self.current_seen_tags = {}
            time.sleep(0.1) # self.current_seen_tags must be populated during this movement
            self.check_danger_zone(y_axis) 
            # self.current_seen_tags = {}                         
            # time.sleep(0.1)
            # self.check_danger_zone(y_axis) 
            # self.current_seen_tags = {}
            # time.sleep(0.1)
            # self.check_danger_zone(y_axis) 
            # self.current_seen_tags = {}
            # time.sleep(0.1)
            # self.check_danger_zone(y_axis) 
            # self.current_seen_tags = {}
                                  
            tag_pos_x_r, tag_pos_y_r  = self.get_w_cord_for_tag(self.tags[tag_id])  # update tag_pos_x_r
        # stop robot
        joy_msg.axes[X] = 0 # reset 
        self.pub_joy.publish(joy_msg) # todo: maybe decrease time of samplig on d to get to target so it doesnt hit target
        print("Arrived!! d: ", tag_pos_x_r-target_pos_x)


    def move_front(self, d, tag_id, y_axis=False, moving_diag=False, diag_update=(0,0)):
        '''
        Args:
        d -> float type represeting meters
        '''
        if tag_id in self.tags:
            #time.sleep(1) 
            self.move_with_tag( d, tag_id, y_axis=False, moving_diag=moving_diag)

        else: 
            raise "Expected tag id: {} not found in self.tags: {} ".format(tag_id, self.tags)

        #update
        if moving_diag:
            assert diag_update != (0,0), " Unexpected diag update"
            self.x_w, self.y_w = diag_update
        elif y_axis:
            self.y_w += d
        else:
            self.x_w += d
       
    def move_front_no_tag(self, d, y_axis = False):
        '''
        Args:
        d -> int type represeting meters
        '''
        joy_msg = self.get_joy_msg()
        print("[move_front] Moving forward for {}m".format(d))
        time_per_m = 2.0408   # [seconds to get to a meter]
        
        t_start = time.time()

        joy_msg.axes[X] = 0.8
        while time.time() < t_start + time_per_m*abs(d):
            self.pub_joy.publish(joy_msg)
        #time.sleep(0.5)
        joy_msg.axes[X] = 0 # reset 
        self.pub_joy.publish(joy_msg)
        #update
        if not y_axis:
            self.x_pos += d
        else:
            self.y_pos += d        

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
        time_per_rad = 3/ (math.pi/2)

        t_start = time.time()
        
        if left:
            joy_msg.axes[THETA] = 0.7
        else:    
            joy_msg.axes[THETA] = -0.7

        while time.time() < t_start + time_per_rad*np.pi/2:
        # while time.time() < t_start + time_per_rad*angle:
            self.pub_joy.publish(joy_msg)
            # just wait for target_time          
        joy_msg.axes[THETA] = 0 # reset 
        self.pub_joy.publish(joy_msg)
        self.current_seen_tags = {}

        time.sleep(1)


    def check_danger_zone(self, moving_on_y_flag):
        '''
        This function checks each tag seen so far in self.tags, if any of the tags are too close, move away from it.
        Possible issues, what if tag is close and then not seen anymore? tag will not be updated

        moving_on_y_flag : True if the robot is aligned with y-axis, otherwise False indicating that is moving on x-axis when check if performed

        '''

        THRESHOLD = 5*self.grid_unit # meters

        for tag_i in self.current_seen_tags:
            tag_pos_x_r, tag_pos_y_r  = self.get_w_cord_for_tag(self.current_seen_tags[tag_i]) 
            if abs(tag_pos_x_r) <= THRESHOLD and abs(tag_pos_y_r) <= THRESHOLD:
                print("!!!! {} in Danger Zone!!!!!!".format(tag_i))
                print("Stopping vehicle!")
                joy_msg = self.get_joy_msg()
                joy_msg.axes[X] = 0 # reset 
                self.pub_joy.publish(joy_msg)
                # if object to the left, then move to the right
                # Y positive means that object is to the right (given robot y-axis points to the left)
                if tag_pos_y_r > 0: 
                    # move to the left
                    print("!!! MOVE TO THE LEFT !!!")
                    self.turn_90(left=True)
                    # move front a bit # but how to know if it's moving on x or y axis on world coordinate?
                    d = 0.4 - tag_pos_y_r + 0.05 # Move away to avoid obstacle plus 5cm to be safe
                    self.move_front_no_tag(d , not moving_on_y_flag)
                    self.turn_90()
                else:
                    print("!!! MOVE TO THE RIGHT !!!")
                    # move to the right 
                    self.turn_90()
                    # move front a bit 
                    d = 0.4 - abs(tag_pos_y_r) + 0.05 # Move away to avoid obstacle plus 5cm to be safe
                    self.move_front_no_tag(d , not moving_on_y_flag)
                    self.turn_90(left=True)

            





    def run(self, target_position_w, tag_id, robot_pos = (0.0,0.0)):
        '''
        Args:
        target_position_w -> Target position in world coordinates (x,y)
        tag_id -> unique identifier for tag associaetd to the target position 
        
        '''              
        if robot_pos != (0.0,0.0):
            self.x_w, self.y_w = robot_pos

        print("Robot's World Position: ", self.get_current_pos())
        print("Target Position: ", target_position_w)

        rospy.Subscriber("/tf", TFMessage, self.tag_information)
        time.sleep(2)

        delta_x, delta_y = self.get_deltas(self.get_current_pos(), target_position_w)
        print("Navigating from {} --> {}".format((self.x_w,self.y_w, self.theta_w), target_position_w))
        print("delta_x: ", delta_x, "delta_y: ", delta_y)
 
        if abs(delta_x) > 0.1 and abs(delta_y) > 0.1:
            ''' ----------------  MOVE DIAG ----------------  '''            
            print("Move Diag!")
            print("Move on x: {} and move on y: {}".format(delta_x, delta_y))

            # first find tag associated to target
            time.sleep(0.2)
            while tag_id not in self.tags:
                # rotate until findind the tag
                print("Looking for tag: ", tag_id )
                self.turn(self.theta_w + 10*(math.pi/180))
                time.sleep(0.2)
            # Now we see tag
            # assming robot will readjust angle before moving forward, the distance to move forward ideally is: 
            print("Found it! ")
            d = math.sqrt(delta_x**2 + delta_y**2)
            print("Distance to travel: ", d)            
            self.move_front(d, tag_id, moving_diag=True, diag_update=(delta_x, delta_y)) # front in direction of x axis (world coordinate)
            time.sleep(1)                   
        elif abs(delta_x) > 0.1:
            ''' ----------------  MOVE ON X AXIS ----------------  ''' 
            # if the robot is not at zero degrees, then rotate to make it zero
            print("Turning to zero degrees...")
            self.turn(0)
            self.move_front(delta_x, tag_id) # front in direction of x axis (world coordinate)
            time.sleep(1)            
            # _, delta_y, _ = self.get_deltas(self.get_current_pos(), target_position_w) #  UPDATED VALUE AFTER MOVING ON X    
        elif abs(delta_y) > 0.1:        
            # move Y axis
            ''' ----------------  MOVE ON Y AXIS ----------------  '''        
            print("delta_y: ", delta_y)
            self.move_sideways_no_slide(delta_y, tag_id)
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
    planning_node = PlanningNode()
    rospy.init_node("planning")

    # points = get_points_from_file()
    # print(points)
    # points = [(0,0,0),(1,0,0),(1,1,1.57),(2,1,0),(2,2,-1.57),(1,1,-0.78),(0,0,0)]
    ''' Calibrate'''
    # feedback_node.run_rotation_calibration()
    ''' -----'''
    points = [(1.0, 0.0), (1.0, 1.0)]     # no angle required to get to these positions
    '''
    marker_1: LM4 in the map
    marker_4: LM8 in the map
    '''
    tags = ["marker_1","marker_7"] # tag ids associated to each position

    # p, tag_id = (points[2], tags[2])
    '''
    Getting Tag info
    '''
    # feedback_node.print_rot_ang_from_tag(tags[2])
    #planning_node.print_TAG_info( "marker_7")
    '''
    Running Experiment
    '''
    # print("Starting navigation to target point: ", p, " tag: ", tag_id)        
    # planning_node.run(p, tag_id, robot_pos= (0.7,1.4,np.pi) )
    
    '''
    Try this next    
    '''
    for p,tag_id in zip(points[:], tags[:]):        
        print("======================================================================")
        print("Starting navigation to target point: ", p, " tag: ", tag_id)        
        planning_node.run(p, tag_id)

