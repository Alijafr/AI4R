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

def estimate_next_pos(height, radar, mapFunc, OTHER=None,N=30000,N_reduced=1000):
    """Estimate the next (x,y) position of the glider."""
    particles = []
    weights = []
    x_estimated = 0.0
    y_estimated = 0.0
    barometer_sigma = 10
    radar_sigma = 25
    optionalPointsToPlot =[]
    print_n = True
    n = 0
    fuz_sigma_heading = 0.15
    fuz_sigma_xy = 7
    if OTHER is None:
        #first time this function is called
        #the expected pos is at (0,0)
        #expected "sea level" is at 5000m
        glider_start_sigma_x = [-250,250]
        glider_start_sigma_y = [-250,250]
        glider_start_sigma_z = [-50,50]
        mu_heading = 0
        sigma_heading = pi/4
        for i in range(N):
            x = random.uniform(glider_start_sigma_x[0],glider_start_sigma_x[1])
            y = random.uniform(glider_start_sigma_y[0],glider_start_sigma_y[1])
            # z = 5000 + random.uniform(glider_start_sigma_z[0], glider_start_sigma_z[1])
            z = height
            heading = random.gauss(mu_heading, sigma_heading)
            # optionalPointsToPlot +=[(x,y,heading)]
            glider_particle = glider(x=x,y=y,z=z,heading=heading,mapFunc=mapFunc)
            #calculate the weight 
            particle_radar = glider_particle.sense()
            weight= measurement_prob(particle_radar,radar,radar_sigma,glider_particle.z,height,barometer_sigma)
            weights.append(weight)
            ##NOTE:the estimation is for the next step
            predicted_particle = glider_particle.glide()
            particles.append(predicted_particle)
            x_estimated +=predicted_particle.x
            y_estimated +=predicted_particle.y
            #plot the particle 
            optionalPointsToPlot +=[(predicted_particle.x,predicted_particle.y,heading)]
            # if print_n:
            #     if particle_radar-radar < 0.01:
            #         n +=1
            #         print(n)
                
        print(max(weights))   
        #calculate teh estimated x and y
        x_estimated /=N
        y_estimated /=N
        xy_estimate = (x_estimated,y_estimated)
        #resampling 
        resampled_particles=resampling(particles,weights,N_reduced)
        OTHER={"p":resampled_particles,'counts':1}
    else:
        recieved_particles = OTHER["p"]
        # print(len(recieved_particles))
        for i in range(N_reduced):
            #fuzzing
            recieved_particles[i].x = random.uniform(recieved_particles[i].x-fuz_sigma_xy, recieved_particles[i].x+fuz_sigma_xy)
            recieved_particles[i].y = random.uniform(recieved_particles[i].y-fuz_sigma_xy, recieved_particles[i].y+fuz_sigma_xy)
            recieved_particles[i].heading = random.gauss(recieved_particles[i].heading, fuz_sigma_heading)
            #calculate the weight
            particle_radar = recieved_particles[i].sense()
            weight= measurement_prob(particle_radar,radar,radar_sigma, recieved_particles[i].z,height,barometer_sigma)
            weights.append(weight)
            ##NOTE:the estimation is for the next step
            predicted_particle = recieved_particles[i].glide()
            particles.append(predicted_particle)
            x_estimated +=predicted_particle.x
            y_estimated +=predicted_particle.y
            #plot the particle 
            optionalPointsToPlot +=[(predicted_particle.x,predicted_particle.y,particles[i].heading)]
        x_estimated /=N_reduced
        y_estimated /=N_reduced
        xy_estimate = (x_estimated,y_estimated)
        #resampling 
        #resampling
        resampled_particles=resampling(particles,weights,N_reduced)
        OTHER={"p":resampled_particles}
       
        # You may optionally also return a list of (x,y,h) points that you would like
        # the PLOT_PARTICLES=True visualizer to plot for visualization purposes.
        # If you include an optional third value, it will be plotted as the heading
        # of your particle.

    # optionalPointsToPlot = [(1, 1), (2, 2), (3, 3)]  # Sample (x,y) to plot
    # optionalPointsToPlot = [(1, 1, 0.5), (2, 2, 1.8), (3, 3, 3.2)]  # (x,y,heading)

    return xy_estimate, OTHER, optionalPointsToPlot


def Gaussian( mu, sigma, x):
        
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
    
    
def measurement_prob(particle_radar,sensor_radar,radar_sigma,particle_height,sensor_height,barometer_sigma):
    
    # calculates how likely a measurement should be
    
    
    #first landmar: height
    # prob = Gaussian(particle_height, barometer_sigma, sensor_height)
    prob = Gaussian(particle_radar, radar_sigma, sensor_radar)
    return prob

def resampling(p,w,N):
    p2 = []
    index = int(random.random() * len(w))
    beta = 0.0
    mw = max(w)
    for i in range(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p2.append(p[index])
    
    return p2

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
