#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Oct 29 16:18:19 2021

@author: labuser
"""

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

######################################################################
# How to use:
#
#   For fill-in questions, change the value of each answer variable.
#
#   For programming questions, change the implementation of the
#   function provided.
#
#   Run the file from the command line to check:
#
#       python ps4_answers.py
#
######################################################################

from __future__ import print_function
from builtins import str
from builtins import range
import hashlib
from math import *
import random
import numpy as np # this is pre-imported but you don't have to use it

# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib
    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')

# STUDENT ID

# Please specify your GT login ID in the whoami variable (ex: jsmith321).

whoami = 'ajar3'

n = float('nan')

# QUESTION 1

# Is the grid function an admissable heuristic?
# Replace n with 0 for false, 1 for true

q1_yes  = 1
q1_no   = 0

# QUESTION 2
# Is the grid function an admissable heuristic?
# Replace n with 0 for false, 1 for true

q2_yes = 0
q2_no  = 1

# QUESTION 3
# What may happen if h is not admissable?
# Replace n with 0 for unchecked, 1 for checked

q3_Astar_finds_optimal_path_always   = 0
q3_Astar_may_find_suboptimal_path    = 1
q3_Astar_may_fail_to_find_path       = 0
q3_None_of_the_above                 = 0

# QUESTION 4 - Dynamic Programming
# Replace the values n with the correct values from
# the Udacity quiz

q4_dynamic_programming = [[ 3,   2,   2  ],
                                    [ 3, 'inf', 1],
                                    [ 4, 'inf', 0]]
                                    

# QUESTION 5 - Stochastic Motion

delta = [[-1, 0],   # go up
         [0, -1],   # go left
         [1, 0],    # go down
         [0, 1]]    # go right

delta_name = ['^', '<', 'v', '>']  # Use these when creating your policy grid.


# --------------------------------------------------
#  Modify the function stochastic_value below
#  using the code from the Udacity submission 
# --------------------------------------------------

def q5_stochastic_motion(grid, goal, cost_step, collision_cost, success_prob):

    # Make sure to mark the goal with an asterisk character '*' (which was not shown in the video).
    # Your policy should also ignore any squares not connected to the goal.
    # In the video Sebastian was initializing the value function with 1000 and using a collision
    # cost of 100 to get the displayed result. In the quiz the value function will be initialized
    # to what ever the collision cost is.

    # Be sure to fix or replace the following two initialization lines with the
    # correct initialization of value and policy
    value = [[4*collision_cost for col in range(len(grid[0]))] for row in range(len(grid))]
    policy = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]

    change = True

    while change:
        change = False
        
        for x in range(len(grid)):
            for y in range(len(grid[0])):

                if goal[0] == x and goal[1] == y:
                    if value[x][y] > 0:
                        value[x][y] = 0
                        policy[x][y] = '*'
                        change = True

                elif grid[x][y] == 0:
                    for a in range(len(delta)):
                        v2 = cost_step
                        for i in range(-1,2):
                            a2 = (a+i)%len(delta)
                            x2 = x + delta[a2][0]
                            y2 = y + delta[a2][1]

                            if i == 0:
                                p2 = success_prob
                            else:
                                p2 = (1.0 - success_prob)/2.

                            if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] != 1 :
                                v2 += p2* value[x2][y2] 
                            else: 
                                v2 += p2*collision_cost
                        if v2 < value[x][y]:
                                change = True
                                value[x][y] = v2
                                policy[x][y] = delta_name[a]
        

    # You will need to be sure to return the following
    return value, policy


# --------------------------------------------------
#  You can use the code below to test your solution
# --------------------------------------------------

# grid = [[0, 0, 0, 0],
#         [0, 0, 0, 0],
#         [0, 0, 0, 0],
#         [0, 1, 1, 0]]
# goal = [0, len(grid[0]) - 1]  # Goal is in top right corner
# cost_step = 1
# collision_cost = 100
# success_prob = 0.5

# value, policy = q5_stochastic_motion(grid, goal, cost_step, collision_cost, success_prob)
# for row in value:
#     print (row)
# for row in policy:
#     print (row)

# Expected outputs:

# [57.9029, 40.2784, 26.0665,  0.0000]
# [47.0547, 36.5722, 29.9937, 27.2698]
# [53.1715, 42.0228, 37.7755, 45.0916]
# [77.5858, 100.00, 100.00, 73.5458]

# ['>', 'v', 'v', '*']
# ['>', '>', '^', '<']
# ['>', '^', '^', '<']
# ['^', ' ', ' ', '^']
# 
# Make sure to mark the goal with an asterisk character '*' (which was not shown in the video). 
# If not, your submission will not be accepted.
# Make sure to output a "*" in the goal position. Your policy should also ignore any squares not connected to the goal.

######################################################################
# Grading methods
#
# Do not modify code below this point.
#
# The auto-grader does not use any of the below code
# 
######################################################################

def float_to_str(x): return "%.04f" % x

def do_nothing(x): return x

FILL_IN_TEST_CASES = ({'variable_name': 'q1_yes',
                       'str_func':          float_to_str,
                       'answer_hash':       '6631178b642694e7514dacd18f3cde95',
                       'points_avail':      1./2.},
                      {'variable_name': 'q1_no',
                          'str_func':          float_to_str,
                       'answer_hash':       '24966cce05b1de3f3a92e19695b25111',
                       'points_avail':      1./2.},
                      {'variable_name': 'q2_yes',
                          'str_func':          float_to_str,
                       'answer_hash':       'e964a07660688f477c8dd9bbe735ccaa',
                       'points_avail':      1./2.},
                      {'variable_name': 'q2_no',
                          'str_func':          float_to_str,
                       'answer_hash':       '03c7721e1b6f42e30e04ecf61127b9e2',
                       'points_avail':      1./2.},
                      {'variable_name': 'q3_Astar_finds_optimal_path_always',
                          'str_func':          float_to_str,
                       'answer_hash':       'd9116fe59e7dacb20397c29fc721a1a9',
                       'points_avail':      1./4.},
                      {'variable_name': 'q3_Astar_may_find_suboptimal_path',
                          'str_func':          float_to_str,
                       'answer_hash':       'b24b910189f73d58bdab93b75a53427c',
                       'points_avail':      1./4.},
                      {'variable_name': 'q3_Astar_may_fail_to_find_path',
                          'str_func':          float_to_str,
                       'answer_hash':       '3176dd3a23222212919eb60083a62d4d',
                       'points_avail':      1./4.},
                      {'variable_name': 'q3_None_of_the_above',
                          'str_func':          float_to_str,
                       'answer_hash':       '6407ff0c145be7c3a815c7f9393a03c7',
                       'points_avail':      1./4.},
                      { 'variable_name': 'q4_dynamic_programming',
                        'variable_idxs':     (0,0),
                        'str_func':          float_to_str,
                        'answer_hash':       '3a1932aa70013ec7aae31073bfea82f0',
                        'points_avail':      1./6. },
                      { 'variable_name': 'q4_dynamic_programming',
                        'variable_idxs':     (0,1),
                        'str_func':          float_to_str,
                        'answer_hash':       'c1e220592da6ca631bdb5cd276dc0df9',
                        'points_avail':      1./6},                      
                      { 'variable_name': 'q4_dynamic_programming',
                        'variable_idxs':     (0,2),
                        'str_func':          float_to_str,
                        'answer_hash':       'e6897e3da513e9a670e3a58495e93b56',
                        'points_avail':      1./6. },
                      { 'variable_name': 'q4_dynamic_programming',
                        'variable_idxs':     (1,0),
                        'str_func':          float_to_str,
                        'answer_hash':       'b7dc6f2f034d48f14aa8398df8d2618b',
                        'points_avail':      1./6. },
                      { 'variable_name': 'q4_dynamic_programming',
                        'variable_idxs':     (1,2),
                        'str_func':          float_to_str,
                        'answer_hash':       '587fcfffa049e6d3c1aefdb1577a081d',
                        'points_avail':      1./6},                      
                      { 'variable_name': 'q4_dynamic_programming',
                        'variable_idxs':     (2,0),
                        'str_func':          float_to_str,
                        'answer_hash':       'bb61102429aa1b3271c6119379b87e0d',
                        'points_avail':      1./6. }, )

# PROGRAMMING


def format_policy_output(pol_arrays):

    v = pol_arrays[0]
    pol = pol_arrays[1]

    try:
        vrows = ['[' + ','.join(['{0:.4f}'.format(x) for x in r]) + ']' for r in v]
        vstr = '[' + ',\n '.join(vrows) + ']\n\n'
        prows = [ str( r ) for r in pol ]
        pstr = '[' + ',\n '.join(prows) + ']'
        return vstr + pstr
    except Exception as e:
        return str(pol_arrays)


def q5_check_output(values, expected):

    # margin for difference between expected and resulting policy values
    eps = 1.e-3

    # default return value
    ret_val = True

    # check that input is as expected
    if len(values) != len(expected):
        print("Expected two inputs received {}".format(len(values)))
        return False

    # Check the values first
    res_vals = values[0]
    exp_vals = expected[0]

    if len(res_vals) != len(exp_vals) or len(res_vals[0]) != len(exp_vals[0]):
        print("Values matrix is the wrong size: {}x{}".format(len(res_vals),len(res_vals[0])))
        return False

    for i in range(len(res_vals)):
        for j in range( len(res_vals[i])):
            if abs(res_vals[i][j] - exp_vals[i][j]) > eps :
                ret_val = False
                break
        else:
            continue  # Continue if haven't hit a break
        break # break out of outer loop

    # Check the policy next
    res_pols = values[1]
    exp_pols = expected[1]

    if len(res_pols) != len(exp_pols) or len(res_pols[0]) != len(exp_pols[0]):
        print("Policy matrix is the wrong size: {}x{}".format(len(res_pols),len(res_pols[0])))
        return False

    for i in range(len(res_pols)):
        for j in range( len(res_pols[i])):
            if res_pols[i][j] != exp_pols[i][j] :
                ret_val = False
                break
        else:
            continue  # Continue if haven't hit a break
        break # break out of outer loop               

    return ret_val


# CODE_TEST_CASES = ({'function_name': 'q5_stochastic_motion',
#                     'function_input': dict( grid = [[0,0,0,0],
#                                                     [0,0,0,0],
#                                                     [0,0,0,0],
#                                                     [0,1,1,0]],
#                                              goal = [0, 3],
#                                              cost_step = 1,
#                                              collision_cost = 100,
#                                              success_prob = 0.5),
#                     'expected_output': ( [[57.9029, 40.2784, 26.0665, 0.0000],
#                                           [47.0547, 36.5722, 29.9937, 27.2698],
#                                           [53.1715, 42.0228, 37.7755, 45.0916],
#                                           [77.5858, 100.00, 100.00, 73.5458]],
#                                           [['>', 'v', 'v', '*'],
#                                            ['>', '>', '^', '<'],
#                                            ['>', '^', '^', '<'],
#                                            ['^', ' ', ' ', '^']]),
#                     'outputs_match_func': q5_check_output,
#                     'output_to_str_func': format_policy_output}, )
                     

CODE_TEST_CASES = ({
    'function_name': 'q5_stochastic_motion',
    'function_input': dict( grid = [[0,1,0],
                                [0,1,0],
                                [0,0,0],]
                                ,
                         goal = [0, 2],
                         cost_step = 20,
                         collision_cost = 100,
                         success_prob = 1),
    'expected_output': ( [[120.0000, 100.0000, 0.0000],
                      [100.0000, 100.0000,20.0000],
                      [80.0000,60.0000,40.0000],]
                      ,
                      [['v', ' ', '*'],
                       ['v', ' ', '^'],
                       ['>', '>', '^'],]
                       ),
    'outputs_match_func': q5_check_output,
    'output_to_str_func': format_policy_output})
                     
if __name__ == '__main__':
    import checkutil
    checkutil.check( FILL_IN_TEST_CASES, CODE_TEST_CASES, locals() )
