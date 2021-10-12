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


from DualRotor import DualRotor
from DroneListener import DroneListener
from drone_pid import part_1_a_pd_thrust, \
                      part_1_b_pd_roll, \
                      part_2_a_pd_thrust_only, \
                      part_2_b_pd_thrust, part_2_b_pd_roll, \
                      part_3_pid_thrust
import numpy as np                     

class DroneSimulator:
    def __init__(self):
        self.listeners: list(DroneListener) = []
    
    def add_listener(self, listener: DroneListener):
        self.listeners.append(listener)
    
    def test_core(self, test_thrust = False, test_roll=False, target_elevation=15, 
                  target_x=-1, num_steps=300, target_hover_time=10, supply_params=False, 
                  drone_mass=500, drone_rpm_error=0, thrust_params: dict() = {}, 
                  roll_params: dict() = {}, target_elev_error = 0.0067, 
                  target_x_error = 0.00, test_integral = False, 
                  DEBUG=True, VISUALIZE = True):
        
        drone           = DualRotor()
        drone.mass      = drone_mass
        drone.RPM_ERROR = drone_rpm_error
        
               
        for l in self.listeners:
            l.initialize(target_elevation, target_x, num_steps)
            
        errors              = []
        percentage_score    = 0         
        thrust_list         = [0]
        roll_list           = [0]
        thrust              = 0
        roll                = 0
        data_thrust         = dict() #{'max_rpm': drone.MAX_PROPELLER_SPEED*2}
        data_roll           = dict() #{'max_roll_angle': drone.MAX_ROLL_ANGLE, 'max_rpm': drone.MAX_PROPELLER_SPEED*2}
        
        drone_rpm_left = []
        drone_rpm_right = []
        drone_roll_angles = []
        
        for t in range(num_steps):
            
            if test_thrust:
                
                if test_integral:
                    # test integral only uses student supplied gain values
                    thrust, data_thrust = part_3_pid_thrust(target_elevation, drone.y, data_thrust)
                    #roll, data_roll     = part_3_pid_roll(target_x, drone.x, roll_params['tau_p'], roll_params['tau_d'], roll_params['tau_i'], data_roll)
                    
                    data_thrust['prev_thrust'] = thrust
                    #if thrust_params.get('tau_i', 0) != 0 and \
                    if drone.is_max_rpm_reached() and \
                                thrust > 0:
                        
                        data_thrust['control_saturated'] = True
                        thrust = 0
                    else:
                        data_thrust['control_saturated'] = False
                
                elif supply_params:
                    thrust, data_thrust = part_1_a_pd_thrust(target_elevation, drone.y, thrust_params['tau_p'], thrust_params['tau_d'], thrust_params['tau_i'], data_thrust)
                
                else:
                    # There are different functions to test thrust with or without roll.
                    if test_roll:
                        thrust, data_thrust = part_2_b_pd_thrust(target_elevation, drone.y, data_thrust)
                    else:
                        thrust, data_thrust = part_2_a_pd_thrust_only(target_elevation, drone.y, data_thrust)
                        
                thrust_list.append(thrust)
                
                
            if test_roll:
                if test_integral:
                    # Roll is not tested for integral at this time
                    pass
                    '''
                    data_roll['prev_roll'] = roll
                    #if roll_params.get('tau_i', 0) != 0:
                    if (drone.roll_angle == drone.MAX_ROLL_ANGLE and roll > 0) or \
                       (drone.roll_angle == -drone.MAX_ROLL_ANGLE  and roll < 0):
                        
                        data_roll['control_saturated'] = True
                        roll = 0
                    else:
                        data_roll['control_saturated'] = False
                    '''
                elif supply_params:
                    roll, data_roll = part_1_b_pd_roll(target_x, drone.x, roll_params['tau_p'], roll_params['tau_d'], roll_params['tau_i'], data_roll)
                else:
                    roll, data_roll = part_2_b_pd_roll(target_x, drone.x, data_roll)
                
                roll_list.append(roll)
                
            drone.update_rotor_speed(thrust, roll)
            
            error = 0
            
            if test_thrust:
                error += abs(target_elevation - drone.y)
                
            if test_roll:
                error += abs(target_x - drone.x)
                
            errors.append(error)
                
            drone.move()
            
            if DEBUG:
                print("Step " + str(t) + \
                      ": y, x, roll, propeller_speed, thrust, weight = ", \
                      drone.y, drone.x, drone.roll_angle, drone.propeller_speed, \
                      drone.thrust, drone.weight)
                
            
            if VISUALIZE:
                for l in self.listeners:
                    l.update(drone.x, drone.y, drone.roll_angle, thrust, roll, \
                             drone.propeller_speed[1], drone.propeller_speed[2])
            
            drone_rpm_left.append(drone.propeller_speed[1])
            drone_rpm_right.append(drone.propeller_speed[2])
            drone_roll_angles.append(drone.roll_angle)
            
        avg_error = max( sum( errors[-target_hover_time:] ) / target_hover_time,  0.0001 )
        
        drone_rpm_left = np.array(drone_rpm_left)/max(np.mean(drone_rpm_left), 0.0001)
        drone_rpm_right = np.array(drone_rpm_right)/max(np.mean(drone_rpm_right), 0.0001)
        
        std_rpm_left = np.std(drone_rpm_left[1:] - drone_rpm_left[:-1])
        std_rpm_right = np.std(drone_rpm_right[1:] - drone_rpm_right[:-1])
            
        std = (std_rpm_left + std_rpm_right)/2
            
        if avg_error > 0:
            percentage_score = min(  (target_elev_error + target_x_error) / avg_error,  1.01  )
            percentage_score -= (std**3 * 1*10**4)  # Penalty for high fluctuations in rpm
            percentage_score = max(percentage_score, 0)
        
        print("%Score = ", percentage_score*100) #, " std_rpm = ", std_rpm_left) #, " Sum_err_signs = ", sum(err_signs))
        #print("std_", std_rpm_left, std_rpm_right, std_err)
        
        if VISUALIZE:
            for l in self.listeners:
                l.end_simulation()
            
        return percentage_score