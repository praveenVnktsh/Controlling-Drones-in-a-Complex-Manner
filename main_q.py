#!/usr/bin/env python3
"""
Air Mobility Project- 16665
Author: Shruti Gangopadhyay (sgangopa), Rathn Shah(rsshah)
"""
from tqdm import tqdm
from distutils.log import error
from locale import currency
import numpy as np
import sys
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
from att import *
from plot import *
from traj import *

def position_controller(current_state,desired_state,params,question):
    '''
    Input parameters:
  
    current_state: The current state of the robot with the following fields:
    current_state["pos"] = [x, y, z],
    current_state["vel"] = [x_dot, y_dot, z_dot]
    current_state["rot"] = [phi, theta, psi]
    current_state["omega"] = [phidot, thetadot, psidot]
    current_state["rpm"] = [w1, w2, w3, w4]
 
    desired_state: The desired states are:
    desired_state["pos"] = [x, y, z] 
    desired_state["vel"] = [x_dot, y_dot, z_dot]
    desired_state["rot"] = [phi, theta, psi]
    desired_state["omega"] = [phidot, thetadot, psidot]
    desired_state["acc"] = [xdotdot, ydotdot, zdotdot]
 
    params: Quadcopter parameters
 
    question: Question number
 
    Output parameters
 
    F: u1 or thrust
 
    acc: will be stored as desired_state["acc"] = [xdotdot, ydotdot, zdotdot]
    '''

    Kp = params['kppos']
    Kd = params['kdpos']


    xerror = desired_state['pos'] - current_state['pos']
    xerrordot = desired_state['vel'] - current_state['vel']
    accs = [0, 0, 0]

    for i in range(3):
        accs[i] = (xerror[i] * Kp[i] + xerrordot[i] * Kd[i])

    accs = np.array(accs)   

    g = params['gravity']
    F = params['mass'] * (accs + desired_state["acc"] + np.array([0, 0, g]))
    desired_state["acc"] = accs
    return F, accs


def motor_model(F,M,current_state,params):

    '''
    Input parameters"

    F,M: required force and moment

    motor_rpm: current motor RPM

    params: Quadcopter parameters

    Output parameters:

    F_motor: Actual thrust generated by Quadcopter's Motors

    M_motor: Actual Moment generated by the Quadcopter's Motors

    rpm_dot: Derivative of the RPM
    '''
    rpm = current_state['rpm']
    ct = params['thrust_coefficient']
    cq = params['moment_scale']
    d = params['arm_length']
    rpm = np.array(rpm)
    
    A = np.array(
        [
            [ct, ct, ct, ct],
            [0, d*ct, 0, -d*ct],
            [-d*ct, 0, d*ct, 0],
            [-cq, cq, -cq, cq],
        ]
    )
    

    rpmd = np.sqrt(np.dot(np.linalg.inv(A), np.array([F[2], M[0], M[1], M[2]])))
    rpmd = np.clip(rpmd, params['rpm_min'], params['rpm_max'])

    rpm = np.clip(rpm,  params['rpm_min'], params['rpm_max'])
    vals = np.dot(A, np.square(rpm))
    F_motor = vals[0]
    M_motor = vals[1:]

    rpm_dot = params['motor_constant'] *(rpmd - rpm)

    return F_motor, M_motor, rpm_dot

def dynamics(t,state,params,F_actual,M_actual,rpm_motor_dot):

    '''
    Input parameters:
  
    state: current state, will be using RK45 to update
 
    F, M: actual force and moment from motor model
 
    rpm_motor_dot: actual change in RPM from motor model
  
    params: Quadcopter parameters
 
    question: Question number
 
    Output parameters:
 
    state_dot: change in state
    '''
    phi, theta, psi = state[6:9]

    m = params['mass']
    g = params['gravity']

    f = F_actual
    F = np.array(
        [
            [f*(np.cos(phi) * np.cos(psi) * np.sin(theta) + np.sin(phi)*np.sin(psi))],
            [f*(np.cos(phi) * np.sin(psi) * np.sin(theta) - np.cos(psi)*np.sin(phi))],
            [f*np.cos(theta)*np.cos(phi) - m*g]
        ]
    )
    accs = (F/m).flatten()
    omegas = state[9:12]
    sub =  np.dot(params['inertia'] , omegas)
    alphas = np.dot(np.linalg.inv(params['inertia']), (M_actual))

    # state: [
        # x,
        # y,
        # z,
        # xdot,
        # ydot,
        # zdot,
        # phi,
        # theta,
        # psi,
        # phidot,
        # thetadot,
        # psidot,
        # rpm
    # ]

    statedot = [
        state[3],
        state[4],
        state[5], 
        accs[0], 
        accs[1], 
        accs[2],
        state[9],
        state[10],
        state[11],
        alphas[0],
        alphas[1],
        alphas[2],
        rpm_motor_dot[0],
        rpm_motor_dot[1],
        rpm_motor_dot[2],
        rpm_motor_dot[3],
    ]
    statedot = np.array(statedot)
    return statedot

    

def main(question):

    # Set up quadrotor physical parameters
    params = {"mass": 0.770, "gravity": 9.80665, "arm_length": 0.1103, "motor_spread_angle": 0.925, \
        "thrust_coefficient": 8.07e-9, "moment_scale": 1.3719e-10, "motor_constant": 36.5, "rpm_min": 3000, \
            "rpm_max": 20000, "inertia": np.diag([0.0033,0.0033,0.005]), "COM_vertical_offset": 0.05,
            'kpatt': [190, 198, 80], 'kdatt' : [30,30,17.88], 'kppos':[17, 17, 20], 'kdpos': [6.6, 6.6, 9], 'question' : question      
            }
    
    # Get the waypoints for this specific question
    [waypoints, waypoint_times] = lookup_waypoints(question)
    # waypoints are of the form [x, y, z, yaw]
    # waypoint_times are the seconds you should be at each respective waypoint
    # make sure the simulation parameters below allow you to get to all points

    # Set the simualation parameters
    time_initial = 0
    time_final = 10
    time_step = 0.005 # in secs
    # 0.005 sec is a reasonable time step for this system
    
    time_vec = np.arange(time_initial, time_final, time_step).tolist()
    max_iteration = len(time_vec)

    # Create the state vector
    state = np.zeros((16,1))
    # state[-4:-1] = 20000
    # state: [x,y,z,xdot,ydot,zdot,phi,theta,psi,phidot,thetadot,psidot,rpm]

    # Populate the state vector with the first waypoint 
    # (assumes robot is at first waypoint at the initial time)
    state[0] = waypoints[0,0]
    state[1] = waypoints[1,0]
    state[2] = waypoints[2,0]
    state[8] = waypoints[3,0]

    #Create a trajectory consisting of desired state at each time step
    # Some aspects of this state we can plan in advance, some will be filled during the loop
    trajectory_matrix = trajectory_planner(question, waypoints, max_iteration, waypoint_times, time_step)
    # [x, y, z, xdot, ydot, zdot, phi, theta, psi, phidot, thetadot, psidot, xacc, yacc, zacc]

    # Create a matrix to hold the actual state at each time step
    actual_state_matrix = np.zeros((15,max_iteration))
    # [x, y, z, xdot, ydot, zdot, phi, theta, psi, phidot, thetadot, psidot, xacc, yacc, zacc]
    actual_state_matrix[:,0] = np.vstack((state[0:12], np.array([[0],[0],[0]])))[:,0]
    
    #Create a matrix to hold the actual desired state at each time step

    # Need to store the actual desired state for acc, omega dot, omega as it will be updated by the controller
    actual_desired_state_matrix = np.zeros((15,max_iteration))
    
    # state list created for the RK45 solver
    state_list = state.T.tolist()
    state_list = state_list[0]

    # Loop through the timesteps and update quadrotor
    for i in tqdm(range(max_iteration-1)):
        
        # convert current state to stuct for control functions
        current_state = {"pos":state_list[0:3],"vel":state_list[3:6],"rot":state_list[6:9], \
            "omega":state_list[9:12],"rpm":state_list[12:16]}
        
        # Get desired state from matrix, put into struct for control functions
        desired_state = {"pos":trajectory_matrix[0:3,i],"vel":trajectory_matrix[3:6,i],\
            "rot":trajectory_matrix[6:9,i],"omega":trajectory_matrix[9:12,i],"acc":trajectory_matrix[12:15,i]}
        

        # Get desired acceleration from position controller
        [F_desired, desired_state["acc"]] = position_controller(current_state,desired_state,params,question)
        
        # Computes desired pitch and roll angles
        [desired_state["rot"],desired_state["omega"]] = attitude_by_flatness(desired_state,params)        
        
        # Get torques from attitude controller
        M_desired = attitude_controller(params,current_state,desired_state,question)
        
        # Motor model
        [F_actual,M_actual,rpm_motor_dot] = motor_model(F_desired,M_desired,current_state,params)
        # print("F actual", F_actual, "F des " , F_desired)
        # Get the change in state from the quadrotor dynamics
        time_int = tuple((time_vec[i],time_vec[i+1]))

        # print(trajectory_matrix[6:9,i].shape, desired_state["rot"])
        # print(desired_state)
        sol = solve_ivp(dynamics,time_int,state_list,args=(params,F_actual,M_actual,rpm_motor_dot),t_eval=np.linspace(time_vec[i],time_vec[i+1],(int(time_step/0.00005))))
        
        state_list = sol.y[:,-1]
        acc = (sol.y[3:6,-1]-sol.y[3:6,-2])/(sol.t[-1]-sol.t[-2])
        # print(acc)
        # Update desired state matrix
        actual_desired_state_matrix[0:3,i+1] = desired_state["pos"]
        actual_desired_state_matrix[3:6,i+1] = desired_state["vel"]
        actual_desired_state_matrix[6:9,i+1] = desired_state["rot"]#[:,0]
        actual_desired_state_matrix[9:12,i+1] = desired_state["omega"]#[:,0]
        actual_desired_state_matrix[12:15,i+1] = desired_state["acc"]#[:,0]

        # Update actual state matrix
        actual_state_matrix[0:12,i+1] = state_list[0:12]
        actual_state_matrix[12:15,i+1] = acc
        
    # plot for values and errors
    plot_state_error(actual_state_matrix,actual_desired_state_matrix,time_vec, params)

    # plot for 3d visualization
    plot_position_3d(actual_state_matrix,actual_desired_state_matrix, params)
    plt.show()
        
        
if __name__ == '__main__':
    '''
    Usage: main takes in a question number and executes all necessary code to
    construct a trajectory, plan a path, simulate a quadrotor, and control
    the model. Possible arguments: 2, 3, 5, 6.2, 6.3, 6.5, 7, 9. THE
    TAS WILL BE RUNNING YOUR CODE SO PLEASE KEEP THIS MAIN FUNCTION CALL 
    STRUCTURE AT A MINIMUM.
    '''
    # run the file with command "python3 main.py question_number" in the terminal
    # question = int(sys.argv[1])
    question = 3
    
    main(question)