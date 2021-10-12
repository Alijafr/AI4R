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



def part_1_a_pd_thrust(target_elevation, drone_elevation, tau_p=0, tau_d=0, tau_i=0, data: dict() = {}):
    '''
    Student code for PD control for thrust. Gain values for P, I and D will 
    be provided by test cases, so here you only need to implement the code, not 
    tune the values. There is a separate part where you also have to come up 
    with and tune the values.
    Drone's starting x, y position is 0, 0.
    
    Args:
    target_elevation (float): The target elevation that the drone has to achieve
    drone_elevation (float): The drone's elevation at this time step
    tau_p (float): Proportional gain, supplied by the test suite
    tau_i (float): Integral gain, supplied by the test suite
    tau_d (float): Differential gain, supplied by the test suite
    data (dict): Dictionary that you can use to pass values across calls.
    
    Returns:
     alpha (float) - PD Control value for thrust
     data (dict): Dictionary that you can use to pass values to the next call to 
                  this function. This will be passed in as the data argument in the
                  next call to this function by the simulator.

    '''
    if not data.keys():
        #first call 
        integral_error = 0.0
        dif_error = 0.0
        error = target_elevation-drone_elevation
        integral_error += error
        thrust_change = tau_p*error + tau_d*dif_error + tau_i*integral_error
        #thrust_change = tau_p*error + tau_d*dif_error 
        data["last_error"] = error
        data["integral"] = integral_error
    else:
        integral_error = data["integral"]
        last_error = data["last_error"]
        error = target_elevation-drone_elevation
        integral_error += error
        dif_error = error - last_error 
        thrust_change = tau_p*error + tau_d*dif_error + tau_i*integral_error
        #thrust_change = tau_p*error + tau_d*dif_error
        data["last_error"] = error
        data["integral"] = integral_error
        
        
    return thrust_change,data
        
        


def part_1_b_pd_roll(target_x, drone_x, tau_p=0, tau_d=0, tau_i=0, data:dict() = {}):
    '''
    Student code for PD control for roll. Gain values will be provided by test
    cases, so here you only need to implement the code, not tune the values. This
    method is called by the test cases along with part_1_a_pd_thrust.
    There is a separate part where you also have to come up with and tune the gain values.
    Drone's starting x,y position is 0, 0.
    
    Args:
    target_x (float): The target horizontal displacement that the drone has to achieve
    drone_x (float): The drone's x position at this time step
    tau_p (float): Proportional gain, supplied by the test suite
    tau_i (float): Integral gain, supplied by the test suite
    tau_d (float): Differential gain, supplied by the test suite
    data (dict): Dictionary that you can use to pass values across calls.
    
    Returns:
     alpha (float) - PD Control value for Roll.
     data (dict): Dictionary that you can use to pass values to the next call to 
                  this function. This will be passed in as the data argument in the
                  next call to this function by the simulator.
     
    '''
    
    ##NOTE: the controller here is negative because the + error should be accounted by - roll 
    if not data.keys():
        #first call 
        integral_error = 0.0
        dif_error = 0.0
        error = target_x-drone_x
        integral_error += error
        roll_change = -tau_p*error - tau_d*dif_error - tau_i*integral_error
        #thrust_change = tau_p*error + tau_d*dif_error 
        data["last_error"] = error
        data["integral"] = integral_error
    else:
        integral_error = data["integral"]
        last_error = data["last_error"]
        error = target_x-drone_x
        integral_error += error
        dif_error = error - last_error 
        roll_change = -tau_p*error - tau_d*dif_error - tau_i*integral_error
        #thrust_change = tau_p*error + tau_d*dif_error
        data["last_error"] = error
        data["integral"] = integral_error
    
    return roll_change,data

def part_2_a_pd_thrust_only(target_elevation, drone_elevation, data: dict() = {},tau_p=26, tau_d=2255, tau_i=0):
    '''
    Student code for PD control for thrust. It will be exclusively used for test
    cases for elevation only, i.e. PID control for roll will not be called with
    it.
    
    You will need to find and tune the gain values for P and D. 
    You can copy your code from the corresponding thrust function above.
    Drone's starting x, y position is 0, 0.
    
    Args:
    target_elevation (float): The target elevation that the drone has to achieve
    drone_elevation (float): The drone's elevation at this time step
    data (dict): Dictionary that you can use to pass values across calls.
    
    Returns:
     alpha (float) - PD Control value for thrust
     data (dict): Dictionary that you can use to pass values to the next call to 
                  this function. This will be passed in as the data argument in the
                  next call to this function by the simulator.

    '''
    
    if not data.keys():
        #first call 
        integral_error = 0.0
        dif_error = 0.0
        error = target_elevation-drone_elevation
        integral_error += error
        thrust_change = tau_p*error + tau_d*dif_error + tau_i*integral_error
        #thrust_change = tau_p*error + tau_d*dif_error 
        data["last_error"] = error
        data["integral"] = integral_error
    else:
        integral_error = data["integral"]
        last_error = data["last_error"]
        error = target_elevation-drone_elevation
        integral_error += error
        dif_error = error - last_error 
        thrust_change = tau_p*error + tau_d*dif_error + tau_i*integral_error
        #thrust_change = tau_p*error + tau_d*dif_error
        data["last_error"] = error
        data["integral"] = integral_error
    
    return thrust_change,data


def part_2_b_pd_thrust(target_elevation, drone_elevation, data: dict() = {},tau_p=27, tau_d=3040, tau_i=0):
    '''
    Student code for PD control for thrust. It will be called 
    with the PID control for roll to control both elevation and horizontal
    position.
    
    You will need to tune the gain values for P and D. 
    You can copy your code from the corresponding thrust function above.
    Drone's starting x, y position is 0, 0.
    
    Args:
    target_elevation (float): The target elevation that the drone has to achieve
    drone_elevation (float): The drone's elevation at this time step
    data (dict): Dictionary that you can use to pass values across calls.
    
    Returns:
     alpha (float) - PD Control value for thrust
     data (dict): Dictionary that you can use to pass values to the next call to 
                  this function. This will be passed in as the data argument in the
                  next call to this function by the simulator.

    '''
    
    if not data.keys():
        #first call 
        integral_error = 0.0
        dif_error = 0.0
        error = target_elevation-drone_elevation
        integral_error += error
        thrust_change = tau_p*error + tau_d*dif_error + tau_i*integral_error
        #thrust_change = tau_p*error + tau_d*dif_error 
        data["last_error"] = error
        data["integral"] = integral_error
    else:
        integral_error = data["integral"]
        last_error = data["last_error"]
        error = target_elevation-drone_elevation
        integral_error += error
        dif_error = error - last_error 
        thrust_change = tau_p*error + tau_d*dif_error + tau_i*integral_error
        #thrust_change = tau_p*error + tau_d*dif_error
        data["last_error"] = error
        data["integral"] = integral_error
    
    return thrust_change,data


def part_2_b_pd_roll(target_x, drone_x, data:dict() = {},tau_p=2.75, tau_d=250, tau_i=0):
    '''
    Student code for PD control for roll. You will need to tune the
    gain values for P and D. You can copy your code from the
    roll function above.
    Drone's starting x, y position is 0, 0.
    
    Args:
    target_elevation (float): The target elevation that the drone has to achieve
    drone_elevation (float): The drone's elevation at this time step
    data (dict): Dictionary that you can use to pass values across calls.
    
    Returns:
     alpha (float) - PD Control value for Roll
     data (dict): Dictionary that you can use to pass values to the next call to 
                  this function. This will be passed in as the data argument in the
                  next call to this function by the simulator.

    '''
    
    if not data.keys():
        #first call 
        integral_error = 0.0
        dif_error = 0.0
        error = target_x-drone_x
        integral_error += error
        roll_change = -tau_p*error - tau_d*dif_error - tau_i*integral_error
        #thrust_change = tau_p*error + tau_d*dif_error 
        data["last_error"] = error
        data["integral"] = integral_error
    else:
        integral_error = data["integral"]
        last_error = data["last_error"]
        error = target_x-drone_x
        integral_error += error
        dif_error = error - last_error 
        roll_change = -tau_p*error - tau_d*dif_error - tau_i*integral_error
        #thrust_change = tau_p*error + tau_d*dif_error
        data["last_error"] = error
        data["integral"] = integral_error
    
    return roll_change,data


def part_3_pid_thrust(target_elevation, drone_elevation, data: dict() = {},tau_p=160, tau_d=5000, tau_i=0.0):
    '''
    Student code for PID control for thrust, with Integral gain.
    You will need to tune the gain values for P, I and D. 
    You can copy your code from the corresponding thrust function above.
    Drone's starting x, y position is 0, 0.

    
    Args:
    target_elevation (float): The target elevation that the drone has to achieve
    drone_elevation (float): The drone's elevation at this time step
    data (dict): Dictionary that you can use to pass values between calls.
          It contains the following reserved keys used by the test
          suite:
              control_saturated (bool): Flag indicating whether the Drone's RPM has reached max value. 
                  Use this if you want to handle Control Saturation as described in the assignment PDF.
              prev_thrust (float): Depending on your implementation, you might need to use this value along with 
                  the "control_saturated" flag.
    
    Returns:
     alpha (float) - PID Control value for thrust
     data (dict): Dictionary that you can use to pass values to the next call to 
                  this function. This will be passed in as the data argument in the
                  next call to this function by the simulator.

     
    '''
    
    if not data.keys():
        #first call 
        integral_error = 0.0
        dif_error = 0.0
        error = target_elevation-drone_elevation
        integral_error += error
        thrust_change = tau_p*error + tau_d*dif_error + tau_i*integral_error
        #thrust_change = tau_p*error + tau_d*dif_error 
        data["last_error"] = error
        data["integral"] = integral_error
    else:
        integral_error = data["integral"]
        last_error = data["last_error"]
        stauration = data["control_saturated"]
        error = target_elevation-drone_elevation
        if not stauration:
            integral_error += error
        dif_error = error - last_error 
        thrust_change = tau_p*error + tau_d*dif_error + tau_i*integral_error
        #thrust_change = tau_p*error + tau_d*dif_error
        data["last_error"] = error
        data["integral"] = integral_error
    
    return thrust_change,data

def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith321).
    whoami = 'ajar3'
    return whoami
