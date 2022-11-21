import numpy as np
import copy
# define map
# 0 indicate space, -1 indicate obstacle
MAP = np.zeros((int(3/0.05),int(3/0.05)))
# define obstacle
for i in range(len(MAP)//2-5,len(MAP)//2+5):
    for j in range(len(MAP)//2-5,len(MAP)//2+5):
        MAP[i][j] = -1

# define start and end position
START = (50,9)
END = (9,50)
'''
def mapping_shortest_dist():
    nodes = []
    map_augment = copy.deepcopy(MAP)
    for i in range(len(MAP)):
        for j in range(len(MAP)):
            map_augment[i][j]=999
    # 10 cm padding
    for i in range(len(map_augment)//2-8,len(map_augment)//2+6):
        for j in range(len(map_augment)//2-8,len(map_augment)//2+6):
            map_augment[i][j] = -1    
    map_augment[START[0]][START[1]]=0

    #planning map
    map_tracking = np.empty((len(MAP),len(MAP)), dtype=object)
    for i in range(len(MAP)):
        for j in range(len(MAP)):
            map_tracking[i][j]=None 
    #simple BFS
    queue = [START]
    searched_grids = []
    while len(queue)>0:
        current = queue[0]
        searched_grids.append(current)

        for vertical in [-1,0,1]:
            for horizontal in [-1,0,1]:
                if current[1]+vertical < 60 and current[0]+horizontal < 60 \
                        and (current[0]+horizontal,current[1]+vertical) not in queue \
                        and map_augment[current[0]+horizontal][current[1]+vertical] != -1 \
                        and (current[0]+horizontal,current[1]+vertical) not in searched_grids:
                    queue.append((current[0]+horizontal,current[1]+vertical))
        
        #update distance
        if current != START:
            min = 999
            min_loc = None
            for vertical in [-1,0,1]:
                for horizontal in [-1,0,1]:
                    if current[0]+horizontal>=0 and current[0]+horizontal<60 \
                            and current[1]+vertical>=0 and current[1]+vertical<60 \
                            and map_augment[current[0]+horizontal][current[1]+vertical]<min:
                        min=map_augment[current[0]+horizontal][current[1]+vertical]
                        min_loc = (current[0]+horizontal,current[1]+vertical)
            map_augment[current[0]][current[1]]=min+1
            map_tracking[current[0]][current[1]]=min_loc
            if current == END:
                break
        queue.pop(0)


    print(map_augment)
    curr = END
    nodes.append(END)
    for i in range(1000):
        curr = map_tracking[curr[0]][curr[1]]
        nodes.append(curr)
        if curr == START:
            break
    return map_tracking
'''


def mapping_shortest_dist():
    nodes = []
    # padding by 10 cm
    map_augment = copy.deepcopy(MAP)
    for i in range(len(map_augment)//2-7,len(map_augment)//2+7):
        for j in range(len(map_augment)//2-7,len(map_augment)//2+7):
            map_augment[i][j] = -1    
    vertices = []
    # get vertices
    for i in range(len(map_augment)-1):
        for j in range(len(map_augment)-1):
            count = np.sum(map_augment[i:i+2,j:j+2])
            if count == -1:
                nodes.append((i,j))

    # quick bfs from start to end
    # in our case, we only have one intermediate step, so simplify the algorithm to find accessible intermediate points
    # first, build access matrix
    block_start = len(map_augment)//2-7
    block_end = len(map_augment)//2+7

    dirX = np.sign(END[0]-START[0])
    dirY = np.sign(END[1]-START[1])
    available = []
    for i in nodes:
        if np.sign(block_start-i[0])==dirX or np.sign(block_start-i[1])==dirY:
            available.append(i)
        # this is not the correct vertex
        if np.sign(block_start-i[0])==dirX and np.sign(block_start-i[1])==dirY:
            available.remove(i)

    print(available)
    '''
    find best vertex
    time is distance+rotation
    '''
    least = 9999
    least_node=None
    for i in available:
        tmp = np.sqrt((i[0]-START[0])**2+(i[1]-START[1])**2) + np.arctan2(-1.0*(i[0]-START[0]),(i[1]-START[1]))*5
        #print(np.arctan2((i[0]-START[0]),(i[1]-START[1]))*5)
        if tmp < least:
            least=tmp
            least_node=i
    print(least_node)

    return ((least_node[1]-START[1])*0.05, -1.0*(least_node[0]-START[0])*0.05, np.arctan2(-1.0*(least_node[0]-START[0]), (least_node[1]-START[1])) ) 
    #return ((least_node[1]-START[1])*0.05, -1.0*(least_node[0]-START[0])*0.05)
    #return least_node



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
    

"""

IMPORTANT:
should it also be OR in the safe run version for this sentence:
if abs(tag_pos_x_r) <= 0.1 and abs(tag_pos_y_r) <= 0.1:
also, turn90() is default to be left=True, you want to change it to left=False

"""
def check_danger_zone2(self, moving_on_y_flag):
    '''
    This function checks each tag seen so far in self.tags, if any of the tags are too close, move away from it.
    Possible issues, what if tag is close and then not seen anymore? tag will not be updated

    moving_on_y_flag : True if the robot is aligned with y-axis, otherwise False indicating that is moving on x-axis when check if performed

    '''

    for tag_i in self.current_seen_tags:
        tag_pos_x_r, tag_pos_y_r  = self.get_w_cord_for_tag(self.current_seen_tags[tag_i]) 
        if abs(tag_pos_x_r) <= 0.1 or abs(tag_pos_y_r) <= 0.1:
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
                self.turn_10(left=True)


            else:
                print("!!! MOVE TO THE RIGHT !!!")
                # move to the right 
                self.turn_10()


def move_with_tag2(self, d, tag_id, y_axis=False, moving_diag=False):
    
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


def move_front(self, d, tag_id, y_axis=False, moving_diag=False, diag_update=(0,0), short = 0):
    '''
    Args:
    d -> float type represeting meters
    '''
    if tag_id in self.tags:
        #time.sleep(1) 
        if short==0:
            self.move_with_tag( d, tag_id, y_axis=False, moving_diag=moving_diag)
        else:
            self.move_with_tag2( d, tag_id, y_axis=False, moving_diag=moving_diag)

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

def run(self, target_position_w, tag_id, short, robot_pos = (0.0,0.0), angle_for_short=0):
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




    # do a prior turn
    if short:
        self.turn(angle_for_short)
    
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
        self.move_front(d, tag_id, moving_diag=True, diag_update=(delta_x, delta_y), short=short) # front in direction of x axis (world coordinate)
        time.sleep(1)                   
    elif abs(delta_x) > 0.1:
        ''' ----------------  MOVE ON X AXIS ----------------  ''' 
        # if the robot is not at zero degrees, then rotate to make it zero
        print("Turning to zero degrees...")
        self.turn(0)
        self.move_front(delta_x, tag_id, short=short) # front in direction of x axis (world coordinate)
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


if __name__ == "__main__":
    planning_node = PlanningNode()
    rospy.init_node("planning")

    short = 0
    ''' -----'''
    
    if short:
        points = [(1.5, 0.0) (1.5, 1.5)]     # no angle required to get to these positions
    else:
        midpoint = mapping_shortest_dist()
        points = [midpoint[:2], (1.5, 1.5)]
    '''
    marker_1: LM4 in the map
    marker_4: LM8 in the map
    '''
    tags = ["marker_1","marker_4"] # tag ids associated to each position

    for p,tag_id in zip(points[:], tags[:]):        
        print("======================================================================")
        print("Starting navigation to target point: ", p, " tag: ", tag_id)     
        if p == midpoint[:2]:
            planning_node.run(p, tag_id, short, angle_for_short=midpoint[2])
        else:
            planning_node.run(p, tag_id, short)