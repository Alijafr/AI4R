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

# These import statements give you access to library functions which you may
# (or may not?) want to use.
from math import *
from glider import *
import random

# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib

    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')


# This is the function you will have to write for part A.
# -The argument 'height' is a floating point number representing
# the number of meters your glider is above the average surface based upon 
# atmospheric pressure. (You can think of this as height above 'sea level'
# except that Mars does not have seas.) Note that this sensor may be
# slightly noisy.
# This number will go down over time as your glider slowly descends.
#
# -The argument 'radar' is a floating point number representing the
# number of meters your glider is above the specific point directly below
# your glider based off of a downward facing radar distance sensor. Note that
# this sensor has random Gaussian noise which is different for each read.

# -The argument 'mapFunc' is a function that takes two parameters (x,y)
# and returns the elevation above "sea level" for that location on the map
# of the area your glider is flying above.  Note that although this function
# accepts floating point numbers, the resolution of your map is 1 meter, so
# that passing in integer locations is reasonable.
#
#
# -The argument OTHER is initially None, but if you return an OTHER from
# this function call, it will be passed back to you the next time it is
# called, so that you can use it to keep track of important information
# over time.
#

def estimate_next_pos(height, radar, mapFunc, OTHER=None,N=10000):
    """Estimate the next (x,y) position of the glider."""
    particles = []
    weights = []
    x_estimated = 0.0
    y_estimated = 0.0 
    if OTHER is None:
        #first time this function is called
        #the expected pos is at (0,0)
        #expected "sea level" is at 5000m
        glider_start_sigma_x = [-250,250]
        glider_start_sigma_y = [-250,250]
        glider_start_sigma_z = [-50,50]
        glider_heading_range =[0 , pi/4]
        for i in range(N):
            x = random.uniform(glider_start_sigma_x[0],glider_start_sigma_x[1])
            y = random.uniform(glider_start_sigma_y[0],glider_start_sigma_y[1])
            z = 5000 + random.uniform(glider_start_sigma_z[0], glider_start_sigma_z[1])
            heading = random.uniform(glider_heading_range[0], glider_heading_range[1])
            glider_particle = glider(x=x,y=y,z=z,heading=heading)
            glider_particle.s.et_noise(0.1,0.05,2.0)
            particles.append(glider_particle)
            #the estimated x and y is the avarage pos from all particles 
            x_estimated += x
            y_estimated += y
            
            #get the weight 
            estimated_elevation_xy = mapFunc(x,y)
            measured_elevation = height - radar
            error = abs(estimated_elevation_xy-measured_elevation)
            #calculate the 
            
        x_estimated /= N
        y_estimated /= N
        xy_estimate = (x_estimated,y_estimated)
        OTHER=particles
    else:
        
        pass
    
       
        # You may optionally also return a list of (x,y,h) points that you would like
        # the PLOT_PARTICLES=True visualizer to plot for visualization purposes.
        # If you include an optional third value, it will be plotted as the heading
        # of your particle.

    # optionalPointsToPlot = [(1, 1), (2, 2), (3, 3)]  # Sample (x,y) to plot
    # optionalPointsToPlot = [(1, 1, 0.5), (2, 2, 1.8), (3, 3, 3.2)]  # (x,y,heading)

    return xy_estimate, OTHER, optionalPointsToPlot


def Gaussian(self, mu, sigma, x):
        
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
    
    
def measurement_prob(self, measurement):
    
    # calculates how likely a measurement should be
    
    prob = 1.0;
    for i in range(len(landmarks)):
        dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
        prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
    return prob

# This is the function you will have to write for part B. The goal in part B
# is to navigate your glider towards (0,0) on the map steering # the glider 
# using its rudder. Note that the Z height is unimportant.

#
# The input parameters are exactly the same as for part A.

def next_angle(height, radar, mapFunc, OTHER=None):
    # How far to turn this timestep, limited to +/-  pi/8, zero means no turn.
    steering_angle = 0.0

    # You may optionally also return a list of (x,y)  or (x,y,h) points that
    # you would like the PLOT_PARTICLES=True visualizer to plot.
    #
    # optionalPointsToPlot = [ (1,1), (20,20), (150,150) ]  # Sample plot points
    # return steering_angle, OTHER, optionalPointsToPlot

    return steering_angle, OTHER


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith321).
    whoami = 'ajar3'
    return whoami
