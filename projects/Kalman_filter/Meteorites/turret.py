######################################################################
# This file copyright the Georgia Institute of Technology
#
# Permission is given to students to use or modify this file (only)
# to work on their assignments.
#
# You may NOT publish this file or make it available to others not in
# the course.
#
######################################################################

# Optional: You may use deepcopy to help prevent aliasing
# from copy import deepcopy

# You may use either the numpy library or Sebastian Thrun's matrix library for
# your matrix math in this project; uncomment the import statement below for
# the library you wish to use.
from math import sqrt
from typing import Counter
import numpy as np

# from matrix import matrix

# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.


OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib
    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')


class kalmanFilter:

    def __init__(self,x,y,dt=0.1):
        self.dt = dt
        self.sig_x = 0.2 # variance in measurment (sensor noise)
        self.sig_y = 0.2
        self.x = np.array([[x],[0],[0],[y],[0],[0]]) #x,x_dot,x_dot_dot, y, y_dot, y_dot_dot
        self.x_hat = np.array([[0.],[0.],[0.],[0.],[0.],[0.]])
        self.P = np.array([[0.2,0,0,0,0,0],
                            [0,0.01,0,0,0,0],
                            [0,0,0.0002,0,0,0],
                            [0,0,0,0.2,0,0],
                            [0,0,0,0,0.01,0],
                            [0,0,0,0,0,0.0002]])



        self.H = np.array([[1,0,0,0,0,0],
                            [0,0,0,1,0,0]]) #sensor only measure x, and y
        # Q is not need if you use acceleration approximation with accleration model, if you only rely on velcity then you will need to include Q
        self.Q = np.array([[0.0,0,0,0,0,0],
                            [0,0,0,0,0,0],
                            [0,0,0,0,0,0],
                            [0,0,0,0.0,0,0],
                            [0,0,0,0,0,0],
                            [0,0,0,0,0,0]])
        #R need to be estimated if not provided
        self.R = np.array([[self.sig_x**2,0],
                            [0,self.sig_y**2]])
        self.F = np.array([[1,dt,0.5*dt**2,0,0,0],
                            [0,1,dt,0,0,0],
                            [0,0,1,0,0,0],
                            [0,0,0,1,dt,0.5*dt**2],
                            [0,0,0,0,1,dt],
                            [0,0,0,0,0,1]])

        self.I = np.eye(6)


    def filter(self,x,y):
        
        
        #measure 
        z = np.array([[x],[y]])

        S = self.H @self.P @self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        y = z - self.H @ self.x

        # self.x = self.x_hat + K@y
        self.x = self.x + K@y
        self.P = (self.I-K@self.H)*self.P

        #predict
        # self.x_hat = self.F @ self.x 
        self.x = self.F @ self.x 
        self.P = self.F @ self.P @ self.F.T + self.Q

        
        # print(self.x)
        # innov = np.sqrt( np.diag(S) )
        # print("innov ", innov)
        # print(y)





        return self.x[0][0],self.x[3][0] , self.x[4][0]





        

class Turret(object):
    """The laser used to defend against invading Meteorites."""

    def __init__(self, init_pos, max_angle_change, dt):
        """Initialize the Turret."""
        self.x_pos = init_pos['x']
        self.y_pos = init_pos['y']
        self.max_angle_change = max_angle_change
        self.dt = dt
        self.kfs = {} # dict of kfs of all the metrorites 
        #will be used for deciding which meteorite to fire at
        self.estimated_x_postions = []
        self.estimated_y_postions = []
        self.estimated_y_vel = []
        self.ids = []
        # self.turret_pos = [0,-1]
        self.metrorites_pos = None
        self.id_to_destory = []
        self.counter =  0
        self.firing = True
        self.previous_id = None


    def observe_and_estimate(self, noisy_meteorite_observations):
        """Observe the locations of the Meteorites.

        self is a reference to the current object, the Turret.
        noisy_meteorite_observations is a list of observations of meteorite
        locations.  Each observation in noisy_meteorite_observations is a tuple
        (i, x, y), where i is the unique ID for an meteorite, and x, y are the
        x, y locations (with noise) of the current observation of that
        meteorite at this timestep. Only meteorites that are currently
        'in-bounds' will appear in this list, so be sure to use the meteorite
        ID, and not the position/index within the list to identify specific
        meteorites. (The list may change in size as meteorites move in and out
        of bounds.) In this function, return the estimated meteorite locations
        (one timestep into the future) as a tuple of (i, x, y) tuples, where i
        is a meteorite's ID, x is its x-coordinate, and y is its y-coordinate.
        """
        # TODO: Update the Turret's estimate of where the meteorites are
        # located at the current timestep and return the updated estimates
        self. metrorites_pos = ()
        self.estimated_x_postions = []
        self.estimated_y_postions = []
        self.ids = []
        for id, x, y in noisy_meteorite_observations:
            if id in self.kfs:
                new_x,new_y,y_vel = self.kfs[id].filter(x,y)
                self.metrorites_pos = ((id,new_x,new_y),) + self.metrorites_pos
                if id > 0 :
                    self.estimated_x_postions.append(new_x)
                    self.estimated_y_postions.append(new_y)
                    self.estimated_y_vel.append(abs(y_vel))
                    self.ids.append(id)
                
            else:    
                kf = kalmanFilter(x,y,self.dt)
                self.kfs[id] = kf
                # if id > 0:
                #     self.estimated_x_postions.append(x)
                #     self.estimated_y_postions.append(y)
                #     self.ids.append(id)

                #new_x,new_y = kf.filter(x,y)
                #metrorites_pos = ((id,new_x,new_y),) + metrorites_pos
        return self.metrorites_pos

    
    def get_laser_action(self, current_aim_rad):
        """Return the laser's action; it can change its aim angle or fire.

        self is a reference to the current object, the Turret.
        current_aim_rad is the laser turret's current aim angle, in radians,
        provided by the simulation.

        The laser can aim in the range [0.0, pi].
        The maximum amount the laser's aim angle can change in a given timestep
        is self.max_angle_change radians. Larger change angles will be
        clamped to self.max_angle_change, but will keep the same sign as the
        returned desired angle change (e.g. an angle change of -3.0 rad would
        be clamped to -self.max_angle_change).
        If the laser is aimed at 0.0 rad, it will point horizontally to the
        right; if it is aimed at pi rad, it will point to the left.
        If the value returned from this function is the string 'fire' instead
        of a numerical angle change value, the laser will fire instead of
        moving.
        Returns: Float (desired change in laser aim angle, in radians), OR
        String 'fire' to fire the laser
        """
        # TODO: Update the change in the laser aim angle, in radians, based
        # on where the meteorites are currently, OR return 'fire' to fire the
        # laser at a meteorite
    
        if self.estimated_y_postions and self.counter > 20: #check if list is empty

        
            min_y = min(self.estimated_y_postions)
            index = self.estimated_y_postions.index(min_y)
            # print(index)
            x = self.estimated_x_postions[index]
            y = self.estimated_y_postions[index]
            # print(y)
            # r = sqrt((x-self.turret_pos[0])**2 + (y-self.turret_pos[1])**2)
            # angle = np.arccos(x/r)
            angle = np.arctan2(y-self.y_pos,x-self.x_pos) 
            # print(angle)
            # print(angle*180.0/3.14)
            # print("x: {}, y: {}".format(x,y))
            aim_diff = angle - current_aim_rad
            # print(self.ids)
            #reinialize the list for the iteration 
            # self.estimated_y_postions = []
            # self.estimated_x_postions = []
            # self.ids = []
            # self.estimated_y_vel = []
            
            if abs(aim_diff) <= 0.017 and (x**2 + (y+1)**2 < 1):
                if self.previous_id == self.ids[index]:
                    self.firing = not self.firing
                if self.firing:
                    return 'fire'
                else:
                    return 0.0
            else:
                return aim_diff
        else:
            self.counter += 1  
        
        return 0.0
    
    # def get_laser_action(self, current_aim_rad):
    #     """Return the laser's action; it can change its aim angle or fire.

    #     self is a reference to the current object, the Turret.
    #     current_aim_rad is the laser turret's current aim angle, in radians,
    #     provided by the simulation.

    #     The laser can aim in the range [0.0, pi].
    #     The maximum amount the laser's aim angle can change in a given timestep
    #     is self.max_angle_change radians. Larger change angles will be
    #     clamped to self.max_angle_change, but will keep the same sign as the
    #     returned desired angle change (e.g. an angle change of -3.0 rad would
    #     be clamped to -self.max_angle_change).
    #     If the laser is aimed at 0.0 rad, it will point horizontally to the
    #     right; if it is aimed at pi rad, it will point to the left.
    #     If the value returned from this function is the string 'fire' instead
    #     of a numerical angle change value, the laser will fire instead of
    #     moving.
    #     Returns: Float (desired change in laser aim angle, in radians), OR
    #     String 'fire' to fire the laser
    #     """
    #     # TODO: Update the change in the laser aim angle, in radians, based
    #     # on where the meteorites are currently, OR return 'fire' to fire the
    #     # laser at a meteorite
    #     if self.estimated_y_postions: #check if list is empty
            
    #         min_y = min(self.estimated_y_postions)
    #         index = self.estimated_y_postions.index(min_y)
    #         # print(self.ids[index])
    #         # max_vel = max(self.estimated_y_vel)
    #         # index_v = self.estimated_y_vel.index(max_vel)
    #         #print(self.ids[index_v])
    #         # if self.estimated_y_postions[index_v] <-0.4:
    #         #     #meteorite to fire at
    #         #     id = self.ids[index_v]
    #         #     if id  not in set(self.id_to_destory):
    #         #         if index > 0:
    #         #             self.id_to_destory.append(id)
    #         # if self.estimated_y_postions[index] < -0.7: 
    #         #     id = self.ids[index]
    #         #     if id not in set(self.id_to_destory):
    #         #         if id > 0:
    #         #             self.id_to_destory.append(id) 
    #         id = self.ids[index]
    #         if id not in set(self.id_to_destory):
    #                     self.id_to_destory.append(id) 
    #         if self.id_to_destory:
    #             for i in self.id_to_destory:
    #                 if i not in set(self.ids): # has been destoried 
    #                     #take it out of the list 
    #                     self.id_to_destory.remove(i)
    #                     print(" {} disappered".format(i))
    #                 # self.id_to_destory = [id for id in self.id_to_destory if id>0]
                

    #         if self.id_to_destory:    
    #             index2shoot = self.ids.index(self.id_to_destory[0]) # always desory the first id in the list (First in first to destroy)
    #             # print(self.id_to_destory)
    #             # print(self.ids[index2shoot])
    #             x = self.estimated_x_postions[index2shoot]
    #             y = self.estimated_y_postions[index2shoot]
    #             # r = sqrt((x-self.turret_pos[0])**2 + (y-self.turret_pos[1])**2)
    #             # angle = np.arccos(x/r)
    #             angle = np.arctan2(y-self.y_pos,x-self.x_pos)
    #             # print(angle*180.0/3.14)
    #             # print("x: {}, y: {}".format(x,y))
    #             aim_diff = angle - current_aim_rad
                
    #             #reinialize the list for the iteration 
    #             self.estimated_y_postions = []
    #             self.estimated_x_postions = []
    #             self.ids = []
    #             self.estimated_y_vel = []
    #             if abs(aim_diff) <= 0.02:

    #                 return 'fire'
    #             else:
    #                 return aim_diff
        
    #     return 0.0
            
        


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith321).
    whoami = 'ajar3'
    return whoami
