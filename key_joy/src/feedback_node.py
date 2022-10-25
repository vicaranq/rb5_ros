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
        self.theta_w = 0

        self.tags = {}


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
            self.turn_v2(self.theta_w+theta, joy_msg) # turn wihtout updating theta of robot, update once reached the target

    def turn_old(self, theta, update=True):
        '''
        theta: angle in radiants to where we want to turn in world frame
        '''
        joy_msg = self.get_joy_msg()
        #calibration_time = 2.5 # [sec/rad]time to get to pi/2 NOTE: Verify 2.5 for
        #time_per_rad = 2.5/ (math.pi/2)
        time_per_rad = 1.5/ (math.pi/2)
        t_start = time.time()
        rads_to_turn = self.get_rads(theta)
        joy_msg.axes[THETA] = 1 if rads_to_turn >= 0 else -1# >0.1
        while time.time() < t_start + time_per_rad*abs(rads_to_turn):
            self.pub_joy.publish(joy_msg)
            # just wait for target_time          
        joy_msg.axes[THETA] = 0 # reset 
        self.pub_joy.publish(joy_msg)
        if update:
            self.theta_w = theta
            print("[turn] theta updated and turned {}rads [{}deg]".format(rads_to_turn, rads_to_turn*180/math.pi))
        else:
            print("[turn] turned {}rads".format(rads_to_turn))
        # self.stop()            

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

    def turn_v2(self, theta, joy_msg):
        '''
        theta: angle in radiants to where we want to turn 
        '''
        #calibration_time = 2.5 # [sec/rad]time to get to pi/2
        time_per_rad = 2.3/ (math.pi/2)
        t_start = time.time()
        rads_to_turn = self.get_rads(theta)
        joy_msg.axes[THETA] = 1 if rads_to_turn >= 0 else -1# >0.1
        while time.time() < t_start + time_per_rad*abs(rads_to_turn):
            self.pub_joy.publish(joy_msg)
            # just wait for target_time          
        joy_msg.axes[THETA] = 0 # reset 
        self.pub_joy.publish(joy_msg)
        self.theta_w = theta
        print("[turn] theta updated and turned {}rads".format(rads_to_turn))
        self.stop()

    def move_front_old(self, d,tag_id, y_axis=False):
        '''
        Args:
        d -> float type represeting meters
        '''
        joy_msg = self.get_joy_msg()
        if tag_id in self.tags:
            tag_pos_x_r, tag_pos_y_r  = self.get_w_cord_for_tag(self.tags[tag_id])

            target_pos_x = tag_pos_x_r - d

            
            print("[move_front] Moving forward for {}m".format(d))
            time_per_m = 2.0408   # [seconds to get to a meter] on carpet
            # time_per_m = 2.7027   # [seconds to get to a meter] on ceramic 
            t_start = time.time()

            #joy_msg.axes[X] = 1.2 if d >=0 or y_axis else -1.2 # >0.1   
            joy_msg.axes[X] = 1.0 if d >=0 or y_axis else -1.0 # >0.1   

            # if d is within 20 cm, start reducing the speed
            #joy_msg.axes[X] = self.reduce_speed(d, joy_msg.axes[X])

            #while time.time() < t_start + time_per_m*abs(d):
            temp_dist = tag_pos_y_r
            while tag_pos_x_r-target_pos_x > 0.1:
                tag_pos_x_r, tag_pos_y_r  = self.get_w_cord_for_tag(self.tags[tag_id])
                if abs(temp_dist - tag_pos_y_r) > 0.05 and tag_pos_x_r-target_pos_x > 0.2:
                    self.readjust_angle(tag_pos_y_r, tag_pos_x_r) 
                # time.sleep(0.5)    
                self.pub_joy.publish(joy_msg)            
                time.sleep(0.3)                                
                
            if abs(tag_pos_x_r-target_pos_x) < 0.1:
                print("Arrived!!")
            joy_msg.axes[X] = 0 # reset 
            self.pub_joy.publish(joy_msg)
        else: 
            time_per_m = 2.7027   # [seconds to get to a meter]
            t_start = time.time()
            print("[move_front - no tag info] Moving forward for {}m".format(d))
            joy_msg.axes[X] = 1.2 if d >=0 or y_axis else -1.2 # >0.1         
            while time.time() < t_start + time_per_m*abs(d):
                self.pub_joy.publish(joy_msg)
            joy_msg.axes[X] = 0 # reset 
            self.pub_joy.publish(joy_msg)

        #update
        if not y_axis:
            self.x_w += d
        else:
            self.y_w += d

    def move_front_new(self, d, joy_msg, y_axis=False):
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
        target_time = 2.1   # [seconds to get to a 90 degrees angle]
        # ideal: target_time = rad / (speed [rad/s])
        t_start = time.time()
        joy_msg.axes[THETA] = 1 # >0.1
        while time.time() < t_start + target_time:
            self.pub_joy.publish(joy_msg)
            # just wait for target_time          

        joy_msg.axes[THETA] = 0 # reset 
        self.pub_joy.publish(joy_msg)

        self.stop()

    def run_backup(self, target_position_w, tag_id):
        '''
        Args:
        target_position_w -> Target position in world coordinates 
        tag_id -> unique identifier for  tag associaetd to the target position (1m away from actual target)
        
        '''              
        print("Robot's World Position: ", self.get_current_pos())
        print("Target Position: ", target_position_w)

        # self.turn_old(0)
        # time.sleep(1)

        # Target in world coordinates
        x_target, y_target, alpha_target = target_position_w
        # Obtain Tag information
        rospy.Subscriber("/tf", TFMessage, self.tag_information)
        time.sleep(1)
        
        t_start = time.time()
        t_experiment = 15 # [s]
        while time.time() < t_start + t_experiment:

            if tag_id in self.tags:
                print("Robot's World Position: ", self.get_current_pos())
                print("Target Position: ", target_position_w)
                tag_pos_T = self.tags[tag_id] # tag position information in tag coordinate frame       

                #if first tag: NOTE: Depending of the tag, the tag coord frame maps differently to world one            

                tag_pos_x_r, tag_pos_y_r = self.get_w_cord_for_tag(tag_pos_T)
                # Flags
                moving_in_y_w = True if y_target - self.y_w > 0 else False
                

                if moving_in_y_w:
                    ''' Move Forward? '''
                    print("tag_pos_x_r: ", tag_pos_x_r)
                    # tag position minus how much we need to move
                    # NOTE: When we get to dist_to_target_x_w, we have arrived to our x coordinate destination
                    dist_to_target_x_w = tag_pos_x_r - (y_target - self.y_w)             
                    print("dist_to_target_x_w: ", dist_to_target_x_w)
                    dist_to_target_y_w = tag_pos_y_r - (x_target - self.x_w)

                    
                    # moving_in_x_w = True if x_target - self.x_w > 0 else False
                    

                    ''' Move on Y axis? if moving in positive direction, then rotate 90deg else -90deg'''
                    # if x are equal -> move +/-90 deg
                    #else: theta_1 = arctan(delta_y_w, delta_x_w)
                    '''
                    if dist_to_target_x_w == 0 and dist_to_target_y_w != 0 :
                        self.turn_old(90/180*math.pi)                    
                        dist_to_target_x_w = -dist_to_target_y_w
                        dist_to_target_y_w=0
                    el '''
                    # if dist_to_target_x_w != 0 and dist_to_target_y_w !=0 :
                    #     self.turn_old(math.atan2(dist_to_target_y_w,dist_to_target_x_w))
                    #     dist_to_target_x_w = math.sqrt(dist_to_target_x_w**2+dist_to_target_y_w**2)
                    #     dist_to_target_y_w=0

                    arrived_to_target = False
                
                    while not arrived_to_target and time.time() < t_start + t_experiment:
                        d_x = tag_pos_x_r - dist_to_target_x_w
                        self.move_front_old(d_x/8, y_axis = True) # front in direction of y axis (world coordinate)
                        d_y = tag_pos_y_r - dist_to_target_y_w               

                        if abs(d_y) > 0.005: # greater than 5cm
                            # if the robot is not at zero degrees, then rotate to make it zero
                            # print("Turning to zero degrees...")
                            # self.turn(0,joy_msg)
                            # ---------- Move Front by 1/3 of the estimated displacement ----------------
                            
                            self.readjust_angle(d_y, d_x) # not working as expected

                        # --------------  Get new position --------------
                        tag_pos_x_r, tag_pos_y_r  = self.get_w_cord_for_tag(self.tags[tag_id])

                        # check how far to dist_to_target_x_w we are   
                        print("d_y: ",  tag_pos_x_r - dist_to_target_x_w)  

                        if abs(dist_to_target_x_w - tag_pos_x_r) < 0.1:
                            arrived_to_target = True
                            self.x_w = x_target # we should be around here
                            self.y_w = y_target

                            print("ARRIVED TO {}!!!!".format(target_position_w))
                    break

                else:
                    # moving in X

                    ''' Move Forward? '''
                    print("tag_pos_x_r: ", tag_pos_x_r)
                    # tag position minus how much we need to move
                    # NOTE: When we get to dist_to_target_x_w, we have arrived to our x coordinate destination
                    dist_to_target_x_w = tag_pos_x_r - (x_target - self.x_w)             
                    print("dist_to_target_x_w: ", dist_to_target_x_w)
                    dist_to_target_y_w = tag_pos_y_r - (y_target - self.y_w)

                    
                    # moving_in_x_w = True if x_target - self.x_w > 0 else False
                    

                    ''' Move on Y axis? if moving in positive direction, then rotate 90deg else -90deg'''
                    # if x are equal -> move +/-90 deg
                    #else: theta_1 = arctan(delta_y_w, delta_x_w)
                    '''
                    if dist_to_target_x_w == 0 and dist_to_target_y_w != 0 :
                        self.turn_old(90/180*math.pi)                    
                        dist_to_target_x_w = -dist_to_target_y_w
                        dist_to_target_y_w=0
                    el '''
                    # if dist_to_target_x_w != 0 and dist_to_target_y_w !=0 :
                    #     self.turn_old(math.atan2(dist_to_target_y_w,dist_to_target_x_w))
                    #     dist_to_target_x_w = math.sqrt(dist_to_target_x_w**2+dist_to_target_y_w**2)
                    #     dist_to_target_y_w=0

                    arrived_to_target = False

                    while not arrived_to_target and time.time() < t_start + t_experiment:
                        d_x = tag_pos_x_r -  dist_to_target_x_w               
                        self.move_front_old(d_x/8) # front in direction of x axis (world coordinate)
                        d_y = tag_pos_y_r -  dist_to_target_y_w  
                        print("DY is: ", d_y)
                        if abs(d_y) > 0.005: # greater than 5cm
                            # if the robot is not at zero degrees, then rotate to make it zero
                            # print("Turning to zero degrees...")
                            # self.turn(0,joy_msg)
                            # ---------- Move Front by 1/3 of the estimated displacement ----------------
                            
                            self.readjust_angle(d_y, d_x) # not working as expected

                        # --------------  Get new position --------------
                        tag_pos_x_r, tag_pos_y_r  = self.get_w_cord_for_tag(self.tags[tag_id])

                        # check how far to dist_to_target_x_w we are   
                        print("d_x: ",  tag_pos_x_r - dist_to_target_x_w)  

                        if abs(dist_to_target_x_w - tag_pos_x_r) < 0.1:
                            arrived_to_target = True
                            self.x_w = x_target # we should be around here
                            self.y_w = y_target

                            print("ARRIVED TO {}!!!!".format(target_position_w))
                    break
                    
            else:

                
                time_per_rad = 2.5
                joy_msg = self.get_joy_msg()
                t_start = time.time()
                #rads_to_turn = (90/180)*math.pi
                rads_to_turn = math.cos((x_target-self.x_w)/(y_target-self.y_w))
                print("radians to turn is: ", rads_to_turn)
                joy_msg.axes[THETA] = 1 if rads_to_turn >= 0 else -1# >0.1
                #while time.time() < t_start + time_per_rad*abs(rads_to_turn):
                self.pub_joy.publish(joy_msg)
                time.sleep(time_per_rad*rads_to_turn)
                #time.sleep(0.1)
                    # just wait for target_time          
                joy_msg.axes[THETA] = 0 # reset 
                self.pub_joy.publish(joy_msg)
                time.sleep(1)
                self.theta_w += rads_to_turn
                print("[turn] theta updated and turned {}rads".format(rads_to_turn))
                self.stop()
                                                                
        print("closing...")
        self.stop()

    def run(self, target_position_w, tag_id):
        '''
        Args:
        target_position_w -> Target position in world coordinates 
        tag_id -> unique identifier for  tag associaetd to the target position (1m away from actual target)
        
        '''              
        print("Robot's World Position: ", self.get_current_pos())
        print("Target Position: ", target_position_w)

        rospy.Subscriber("/tf", TFMessage, self.tag_information)
        time.sleep(3)

        joy_msg = self.get_joy_msg()

        delta_x, _, _ = self.get_deltas(self.get_current_pos(), target_position_w)
        print("Navigating from {} --> {}".format((self.x_w,self.y_w, self.theta_w), target_position_w))
        print("delta_x: ", delta_x)
        # y_curr, x_curr, theta_curr = self.get_current_pos()
        # move X axis
        if abs(delta_x) > 0.1:
            # if the robot is not at zero degrees, then rotate to make it zero
            print("Turning to zero degrees...")
            self.turn_v2(0,joy_msg)
            self.move_front_old(delta_x, tag_id) # front in direction of x axis (world coordinate)
            time.sleep(1)
        # move Y axis
        _, delta_y, _ = self.get_deltas(self.get_current_pos(), target_position_w)
        print("delta_y: ", delta_y)
        time.sleep(1)
        if abs(delta_y) > 0.1:        
            self.move_sideways_no_slide(delta_y, tag_id, joy_msg)
            time.sleep(1)
        _, _, delta_theta = self.get_deltas(self.get_current_pos(), target_position_w)
        print("delta_theta: ", delta_theta)
        time.sleep(1)
        # move angle
        if abs(delta_theta)  > 0.1:
            self.turn_v2(target_position_w[2], joy_msg)
            time.sleep(1)
        print("State: ", (self.x_w, self.y_w, self.theta_w))
        self.stop()


    def print_TAG_info(self,  tag_id):
        print("Robot's World Position: ", self.get_current_pos())

        # Obtain Tag information
        rospy.Subscriber("/tf", TFMessage, self.tag_information)
        
        t_start = time.time()
        t_experiment = 30 # [s]
        while time.time() < t_start + t_experiment:
            if tag_id in self.tags:
                print("Tag info: \n",self.tags[tag_id])
                time.sleep(1)
            elif self.tags:
                print("Different Tags observed: ", list(self.tags.keys()) )

if __name__ == "__main__":
    feedback_node = FeedbackNode()
    rospy.init_node("feedback")

    # feedback_node.run_rotation_calibration()
    # points = get_points_from_file()
    # print(points)
    # points = [(0,0,0),(1,0,0),(1,1,1.57),(2,1,0),(2,2,-1.57),(1,1,-0.78),(0,0,0)]

    points = [(0.7,0,0), (0.7,1.4,np.pi), (0,0,0)]    
    tags = ["marker_1","marker_4","marker_2"] # tag ids associated to each position
    p, tag_id = (points[0], tags[0])
    '''
    Getting Tag info
    '''
    # feedback_node.print_TAG_info( tags[1])
    '''
    Running Experiment
    '''
    # print("Starting navigation to target point: ", p, " tag: ", tag_id)        
    # feedback_node.run(p, tag_id)
    
    '''
    Try this next    
    '''
    for p,tag_id in zip(points[:], tags[:]):        
        print("======================================================================")
        print("Starting navigation to target point: ", p, " tag: ", tag_id)        
        feedback_node.run(p, tag_id)
