"""
 === Introduction ===

   The assignment is broken up into two parts.

   Part A:
        Create a SLAM implementation to process a series of landmark measurements (location of tree centers) and movement updates.
        The movements are defined for you so there are no decisions for you to make, you simply process the movements
        given to you.
        Hint: A planner with an unknown number of motions works well with an online version of SLAM.

    Part B:
        Here you will create the action planner for the drone.  The returned actions will be executed with the goal being to navigate to 
        and extract the treasure from the environment marked by * while avoiding obstacles (trees). 
        Actions:
            'move distance steering'
            'extract treasure_type x_coordinate y_coordinate' 
        Example Actions:
            'move 1 1.570963'
            'extract * 1.5 -0.2'

    Note: All of your estimates should be given relative to your drone's starting location.
    
    Details:
    - Start position
      - The drone will land at an unknown location on the map, however, you can represent this starting location
        as (0,0), so all future drone location estimates will be relative to this starting location.
    - Measurements
      - Measurements will come from trees located throughout the terrain.
        * The format is {'landmark id':{'distance':0.0, 'bearing':0.0, 'type':'D', 'radius':0.5}, ...}
      - Only trees that are within the horizon distance will return measurements.
    - Movements
      - Action: 'move 1.0 1.570963'
        * The drone will turn counterclockwise 90 degrees [1.57 radians] first and then move 1.0 meter forward.
      - Movements are stochastic due to, well, it being a robot.
      - If max distance or steering is exceeded, the drone will not move.
      - Action: 'extract * 1.5 -0.2'
        * The drone will attempt to extract the specified treasure (*) from the current location of the drone (1.5, -0.2).
      - The drone must be within 0.25 distance to successfully extract a treasure.

    The drone will always execute a measurement first, followed by an action.
    The drone will have a time limit of 5 seconds to find and extract all of the needed treasures.
"""

from typing import Dict, List
from matrix import *
import math
import numpy as np

# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib
    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')

class SLAM:
    """Create a basic SLAM module.
    """

    def __init__(self):
        """Initialize SLAM components here.
        """
        self.first_init = True 
        self.drone_yaw = 0.0
        self.xi = matrix()
        self.omega = matrix()
        #the robot start at 0, 0
        self.omega.zero(2, 2) #for initial x,y
        self.omega[0][0] = 1.
        self.omega[1][1] = 1.
        
        self.xi.zero(2, 1)# start from x=0 , y= 0
        
        self.mu = matrix()
        self.mu.zero(2, 1)
        self.landmarks  = {} #should contain the ids and index number in the omeaga starting from 
        self.measurement_noise = 1 # the more, the less trust we have in the measurement
        self.motion_noise = 0.5 # the more, the less trust we have in the motion
    # Provided Functions
    def get_coordinates(self):
        """
        Retrieves the (x, y) locations in meters of the drone and all landmarks (trees)

        Args: None

        Returns:
            The (x,y) coordinates in meters of the drone and all landmarks (trees) in the format:
                    {
                        'self': (x, y),
                        '<landmark_id_1>': (x1, y1),
                        '<landmark_id_2>': (x2, y2),
                        ....
                    }
        """
        positions = {}
        positions['self'] = (self.mu[0][0],self.mu[1][0])
        for landmark_id in self.landmarks:
            positions[landmark_id] = (self.mu[self.landmarks[landmark_id]][0],self.mu[self.landmarks[landmark_id]+1][0])
        return positions

    def process_measurements(self, measurements: Dict):
        """
        Process a new series of measurements.

        Args:
            measurements: Collection of measurements of tree positions and radius
                in the format {'landmark id':{'distance': float <meters>, 'bearing':float <radians>, 'type': char, 'radius':float <meters>}, ...}

        Returns:
            (x, y): current belief in location of the drone in meters
        """
        #need to check if the landmarks already exit
        for landmark_id in measurements: #interate over the ids of the landmarks 
            #check if the landmarks is new or existing 
            dim = self.omega.dimx
            if landmark_id in self.landmarks:
                m = self.landmarks[landmark_id]
            else:
                #add the new lanmark to the dict, and edit the matrices
                m = dim 
                self.landmarks[landmark_id] = m
                #expand omega
                list_ = list(range(0,dim))
                self.omega = self.omega.expand(dim+2, dim+2, list_)
                self.xi = self.xi.expand(dim+2, 1, list_,[0])
            
            distance = measurements[landmark_id]['distance']
            bearing = measurements[landmark_id]['bearing']
            # type_ = measurements[landmark_id]['type']
            measurement = (distance*np.cos(bearing+self.drone_yaw),distance*np.sin(bearing+self.drone_yaw))
            # print("Tree {} is at {}".format(type_,measurement))
            #update teh information matrix/vector based on the measurement
            #the divison over the noise gives the confidence of the measurement (weight)
            for b in range(2):
                self.omega.value[b][b] += 1.0/self.measurement_noise
                self.omega.value[m+b][m+b] += 1.0/self.measurement_noise
                self.omega.value[b][m+b] += -1.0/self.measurement_noise
                self.omega.value[m+b][b] += -1.0/self.measurement_noise
                self.xi.value[b][0] += -measurement[b]/self.measurement_noise
                self.xi.value[m+b][0] += measurement[b]/self.measurement_noise
        
        # print(self.landmarks)
        self.mu = self.omega.inverse() * self.xi
        drone_pos = (self.mu[0][0], self.mu[1][0])
        #print("update from measurements: ", drone_pos)
        return drone_pos

    def process_movement(self, distance: float, steering: float):
        """
        Process a new movement.

        Args:
            distance: distance to move in meters
            steering: amount to turn in radians

        Returns:
            (x, y): current belief in location of the drone in meters
        """
        self.drone_yaw += steering
        if self.drone_yaw > np.pi:
            diff = self.drone_yaw - np.pi 
            self.drone_yaw = diff-np.pi
        elif self.drone_yaw < -np.pi:
            diff = self.drone_yaw + np.pi 
            self.drone_yaw = np.pi + diff
        #print("angle: ", self.drone_yaw*180/np.pi)
        dim = self.omega.dimx
        
        #expand the infromation matrix and vector by one new position
        if dim >2:
            list_ = [0,1] + list(range(4,dim+2))
        else: 
            list_ = [0,1]
        self.omega = self.omega.expand(dim+2, dim+2, list_, list_)
        self.xi = self.xi.expand(dim+2, 1, list_,[0])
      
            
        motion= (distance*np.cos(self.drone_yaw),distance*np.sin(self.drone_yaw))
        
        #print("movement: ",motion)
            
        #update the information matrix/vector based on teh robot motion
        for b in range(4):
            self.omega.value[b][b] += 1.0/self.motion_noise
        for b in range(2):
            self.omega.value[b][b+2] += -1.0/self.motion_noise
            self.omega.value[b+2][b] += -1.0/self.motion_noise
            self.xi.value[b][0] += -motion[b]/self.motion_noise
            self.xi.value[b+2][0] += motion[b]/self.motion_noise
            
        #now factor out the preious pose 
        # omega_reduced = omega - A.transpose * B.inverse *A
        newlist = range(2,len(self.omega.value))
        A = self.omega.take([0,1],newlist)
        B = self.omega.take([0,1])
        C = self.xi.take([0,1],[0])
        
        self.omega = self.omega.take(newlist)-A.transpose()*B.inverse()*A
        self.xi = self.xi.take(newlist,[0]) - A.transpose()*B.inverse()*C
        
       
        self.mu = self.omega.inverse() * self.xi
        drone_pos = (self.mu[0][0], self.mu[1][0])
        #print("update from movements: ", drone_pos)
        return drone_pos


class IndianaDronesPlanner:
    """
    Create a planner to navigate the drone to reach and extract the treasure marked by * from an unknown start position while avoiding obstacles (trees).
    """

    def __init__(self, max_distance: float, max_steering: float):
        """
        Initialize your planner here.

        Args:
            max_distance: the max distance the drone can travel in a single move in meters.
            max_steering: the max steering angle the drone can turn in a single move in radians.
        """
        self.max_distance = max_distance
        self.max_steering = max_steering
        self.localize = SLAM()
        self.map = self.localize.get_coordinates()
        self.drone_pos = self.map['self']
        self.random_angle_movement = [0, np.pi/6,-np.pi/6,np.pi/4,-np.pi/4, np.pi/3,-np.pi/3]
    '''
    line_circle_intersect and check_crash functions were taken from testing_suite_indiana_drones.py
    '''
    def line_circle_intersect(self, first_point, second_point, origin, radius):
        """ Checks if a line segment between two points intersects a circle of a certain radius and origin

        Args: 
            first_point : (x,y)
            second_point : (x,y)
            origin : (x,y)
            radius : r

        Returns:
            intersect : True/False
        
        """
        
        ###REFERENCE###
        #https://math.stackexchange.com/questions/275529/check-if-line-intersects-with-circles-perimeter
        x1, y1 = first_point
        x2, y2 = second_point
        
        ox,oy = origin
        r = radius
        x1 -= ox
        y1 -= oy
        x2 -= ox
        y2 -= oy
        a = (x2 - x1)**2 + (y2 - y1)**2
        b = 2*(x1*(x2 - x1) + y1*(y2 - y1))
        c = x1**2 + y1**2 - r**2
        disc = b**2 - 4*a*c

        if a == 0:
            if c <= 0:
                return True
            else:
                return False
        else: 

            if (disc <= 0):
                return False
            sqrtdisc = math.sqrt(disc)
            t1 = (-b + sqrtdisc)/(2*a)
            t2 = (-b - sqrtdisc)/(2*a)
            if((0 < t1 and t1 < 1) or (0 < t2 and t2 < 1)):
                return True
            return False
                
    def distance2goal(self,current_pos,goal):
        
        return ((current_pos[1]-goal[1])**2 + (current_pos[0]-goal[0])**2)**0.5
    
    def normalize_angle (self,current_angle,steering):
        current_angle += steering
        if current_angle > np.pi:
            diff = current_angle- np.pi 
            current_angle = diff-np.pi
        elif current_angle < -np.pi:
            diff = current_angle + np.pi 
            current_angle = np.pi + diff
        
        return current_angle

    def next_move(self, measurements: Dict, treasure_location: Dict):
        """Next move based on the current set of measurements.

        Args:
            measurements: Collection of measurements of tree positions and radius in the format 
                          {'landmark id':{'distance': float <meters>, 'bearing':float <radians>, 'type': char, 'radius':float <meters>}, ...}
            treasure_location: Location of Treasure in the format {'x': float <meters>, 'y':float <meters>, 'type': char '*'}
        
        Return: action: str, points_to_plot: dict [optional]
            action (str): next command to execute on the drone.
                allowed:
                    'move distance steering'
                    'move 1.0 1.570963'  - Turn left 90 degrees and move 1.0 distance.
                    
                    'extract treasure_type x_coordinate y_coordinate'
                    'extract * 1.5 -0.2' - Attempt to extract the treasure * from your current location (x = 1.5, y = -0.2).
                                           This will succeed if the specified treasure is within the minimum sample distance.
                   
            points_to_plot (dict): point estimates (x,y) to visualize if using the visualization tool [optional]
                            'self' represents the drone estimated position
                            <landmark_id> represents the estimated position for a certain landmark
                format:
                    {
                        'self': (x, y),
                        '<landmark_id_1>': (x1, y1),
                        '<landmark_id_2>': (x2, y2),
                        ....
                    }
        """
        treasure_pos = (treasure_location['x'],treasure_location['y'])
        treasure_type = treasure_location['type']
        
        self.localize.process_measurements(measurements)
        #should this be before or after the update?
        self.map = self.localize.get_coordinates()
        self.drone_pos = self.map['self'] #current pos of the drone 
        
        #get the angle between the inertial frame (but center at the drone body) and tresure
        tresure_angle = np.arctan2(treasure_pos[1]-self.drone_pos[1],treasure_pos[0]-self.drone_pos[0])
        alignment_angle = tresure_angle-self.localize.drone_yaw 
        #caluclate the distance to goal 
        dist2tresure = self.distance2goal(self.drone_pos, treasure_pos)
        # print(dist2tresure)
        if dist2tresure > self.max_distance:
            movement = self.max_distance
        else:
            if dist2tresure  <= 0.05: 
                #we reach the treasure, extract it
                print("extract now")
                return 'extract {} {} {}'.format(treasure_type,treasure_pos[0], treasure_pos[1]) , self.map
            else:
                movement = dist2tresure 
        #calcualte a new position leading to the treasure, and check if it cause crash 
        for turn_angle in self.random_angle_movement:
            collision = False
            steering = alignment_angle + turn_angle
            if steering > self.max_steering:
                steering = self.max_steering
            elif steering < -self.max_steering:
                steering = -self.max_steering
                
            
            next_angle = self.normalize_angle(self.localize.drone_yaw,steering)
            new_pos = (self.drone_pos[0]+movement*np.cos(next_angle) , self.drone_pos[1]+movement*np.sin(next_angle) )
            for tree in measurements:
                #the number constant number multipied by the raduis is used to ensure that we don't get very close to the tree
                intersection = self.line_circle_intersect(self.drone_pos, new_pos, self.map[tree], 1.3*measurements[tree]['radius'])
                if intersection:
                    collision = True
                    break
            if not collision:
                #update the localization (SLAM)
                self.localize.process_movement(movement,steering)
                self.map = self.localize.get_coordinates()
                return 'move {} {}'.format(movement, steering) , self.map
        
        
        print('fail')
        #collide with obstacle and take the penalty :( 
        
        if alignment_angle > self.max_steering:
            alignment_angle = self.max_steering
        elif alignment_angle < -self.max_steering:
            alignment_angle = -self.max_steering
        #update the location
        alignment_angle = self.normalize_angle(self.localize.drone_yaw,alignment_angle)
        # print(alignment_angle)
        self.localize.process_movement(movement,alignment_angle)
        self.map = self.localize.get_coordinates()
        return 'move {} {}'.format(movement, alignment_angle) , self.map

def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith321).
    whoami = 'ajar3'
    return whoami