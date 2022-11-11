#!/usr/bin/env python3

import numpy as np
import sys
import matplotlib.pyplot as plt
from controller import Controller
from drone import Drone
from plot import *
from traj import *
from stateManager import *

def execute(params, stateManager : StateManager = None):

    dt = params['dt'] # in secs

    state = np.zeros((15, 1))

    # state[0] = waypoints[0]
    # state[1] = waypoints[1]
    # state[2] = waypoints[2]
    # state[8] = waypoints[3]

    # [x, y, z, xdot, ydot, zdot, phi, theta, psi, phidot, thetadot, psidot, xacc, yacc, zacc]

    actual_desired_state_matrix = []
    actual_state_matrix = [np.vstack((state[0:12], np.array([[0],[0],[0]]))).flatten()]

    drone = Drone(params)
    controller = Controller(params)
    planner = TrajectoryGenerator(params)
    i = 0

    t = 0
    times = []
    while not stateManager.isComplete():
        i += 1
        t += dt
        times.append(t)


        desired_state = stateManager.getDesiredState(t)
        
        current_state = {"pos":state[0:3],"vel":state[3:6],"rot":state[6:9], \
            "omega":state[9:12],"rpm":state[12:16]}
        

        F_desired, desired_state["acc"] = controller.position_controller(current_state, desired_state,question)
        desired_state["rot"],desired_state["omega"] = controller.attitude_by_flatness(desired_state)        

        M_desired = controller.attitude_controller(current_state,desired_state,question)
        sol = drone.step(F_desired,M_desired,current_state,params)

        # we need to set the accelerations ourselves because we are setting our snap to be zero.
        # use first order approximations

        state_list = sol.y[:,-1]
        acc = (sol.y[3:6,-1]-sol.y[3:6,-2])/(sol.t[-1]-sol.t[-2])

        state_list[12:15] = acc
        actual_state_matrix.append(state_list.copy())

        temp  = np.zeros((15, 1))
        temp[0:3,i+1] = desired_state["pos"]
        temp[3:6,i+1] = desired_state["vel"]
        temp[6:9,i+1] = desired_state["rot"]
        temp[9:12,i+1] = desired_state["omega"]
        temp[12:15,i+1] = desired_state["acc"]
        actual_desired_state_matrix.append(temp.copy())

        
        
    actual_desired_state_matrix = np.array(actual_desired_state_matrix).T
    actual_state_matrix = np.array(actual_state_matrix).T
    time_vec = np.array(times)
    return actual_state_matrix, actual_desired_state_matrix, time_vec


def plot(actual_state_matrix, actual_desired_state_matrix, time_vec, params):
    plot_state_error(actual_state_matrix,actual_desired_state_matrix,time_vec, params)
    plot_position_3d(actual_state_matrix,actual_desired_state_matrix, params)
    plt.show()  


        
def main(params):
    manager = StateManager(params['question'])
    outs = execute(params, manager)


    plot(*outs)

        
if __name__ == '__main__':
    '''
    Usage: main takes in a question number and executes all necessary code to
    construct a trajectory, plan a path, simulate a quadrotor, and control
    the model. Possible arguments: 2, 3, 5, 6.2, 6.3, 6.5, 7, 9. THE
    TAS WILL BE RUNNING YOUR CODE SO PLEASE KEEP THIS MAIN FUNCTION CALL 
    STRUCTURE AT A MINIMUM.
    '''
    # run the file with command "python3 main.py question_number" in the terminal
    
    question = int(sys.argv[1])
    params = {
        "mass": 0.770, 
        "gravity": 9.80665, 
        "arm_length": 0.1103, 
        "motor_spread_angle": 0.925, 
        "thrust_coefficient": 8.07e-9, 
        "moment_scale": 1.3719e-10, 
        "motor_constant": 36.5, 
        "rpm_min": 3000,             
        "rpm_max": 20000, 
        "inertia": np.diag([0.0033,0.0033,0.005]), 
        "COM_vertical_offset": 0.05,            
        'kpatt': [190, 198, 80], 
        'kdatt' : [30,30,17.88], 
        'kppos':[17, 17, 20], 
        'kdpos': [6.6, 6.6, 9], 
        'question' : question,
        'dt' : 0.005      
    }
    main(params)    
    