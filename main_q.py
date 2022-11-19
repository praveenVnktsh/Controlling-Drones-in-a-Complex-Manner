#!/usr/bin/env python3

import numpy as np
import sys
import matplotlib.pyplot as plt
from controller import Controller
from drone import Drone
from plot import *
from stateManager import StateManager
from utils import NumpyEncoder, State

def execute(params : dict, stateManager : StateManager = None):

    dt = params['dt'] # in secs


    desired_states = []
    actual_states = []
    initState = np.zeros((16,))
    # initState[2] = 0.5
    drone = Drone(params, initState =initState)
    controller = Controller(params)

    i = 0

    t = 0
    times = []

    stateManager.setNextState(t, drone.state)
    
    thrustToMassPlot = []

    trackingIntervalPlot = {
        'actualstates': [],
        'desiredstates' : []
    }

    while not stateManager.isComplete():

        i += 1
        t += dt
        
        desired_state_dic = stateManager.getDesiredState(t, drone.state)

        if stateManager.isComplete():
            break

        current_state_dic = {
            "pos":drone.state[0:3].squeeze(),
            "vel":drone.state[3:6].squeeze(),
            "rot":drone.state[6:9].squeeze(), 
            "omega":drone.state[9:12].squeeze(),
            "rpm":drone.state[12:16].squeeze()
        }



        F_desired, desired_state_dic['acc'] = controller.position_controller(current_state_dic, desired_state_dic)
        desired_state_dic["rot"],desired_state_dic["omega"] = controller.attitude_by_flatness(desired_state_dic)        

        M_desired = controller.attitude_controller(current_state_dic,desired_state_dic)
        # print(M_desired)
        sol = drone.step(F_desired, M_desired)

        # we need to set the accelerations ourselves because we are setting our snap to be zero.
        # use first order approximations

        state_list = sol.y[:,-1].copy()
        acc = (sol.y[3:6,-1]-sol.y[3:6,-2])/(sol.t[-1]-sol.t[-2])

        state_list[12:15] = acc
        actual_states.append(state_list[:15].copy())

        temp  = np.zeros((15, 1))
        temp[0:3] = desired_state_dic["pos"].reshape(-1, 1)
        temp[3:6] = desired_state_dic["vel"].reshape(-1, 1)
        temp[6:9] = desired_state_dic["rot"].reshape(-1, 1)
        temp[9:12] = desired_state_dic["omega"].reshape(-1, 1)
        temp[12:15] = desired_state_dic["acc"].reshape(-1, 1)
        
        desired_states.append(temp.copy())
        if stateManager.state == State.TRACK:
            trackingIntervalPlot['actualstates'].append(state_list[:15].copy())
            trackingIntervalPlot['desiredstates'].append(temp.copy())

        times.append(t)
        thrustToMassPlot.append((drone.thrust) / (params['mass'] * params['gravity']))

        # if stateManager.state != State.TAKEOFF:
        #     break

    print("Statemanager isComplete", stateManager.isComplete())
        
    desired_states = np.array(desired_states).T.squeeze()
    actual_states = np.array(actual_states).T.squeeze()
    time_vec = np.array(times)
    plotDic = {
        'FbyW' : thrustToMassPlot,
        'actual_states' : actual_states,
        'desired_states' : desired_states,
        'time_vec' : time_vec,
        'trackingintervals' : trackingIntervalPlot
    }
    return plotDic


def plot(plotDic, params):

    plt.figure()
    plt.plot(plotDic['time_vec'], plotDic['FbyW'])
    plt.title("Thrust to Weight ratio")
    plt.xlabel("Time (s)")
    plt.ylabel("Amplitude")
    plt.savefig(f'outputs/{params["question"]}/fbyw.png')

    plot_state_error(plotDic["actual_states"],plotDic["desired_states"],plotDic["time_vec"], params)
    plot_position_3d(plotDic["actual_states"],plotDic["desired_states"], params)
    plt.show()  


        
def main(params):
    manager = StateManager(params)
    plotDic = execute(params, manager)
    plot(plotDic, params)

        
if __name__ == '__main__':
    '''
    Usage: main takes in a question number and executes all necessary code to
    construct a trajectory, plan a path, simulate a quadrotor, and control
    the model. Possible arguments: 2, 3, 5, 6.2, 6.3, 6.5, 7, 9. THE
    TAS WILL BE RUNNING YOUR CODE SO PLEASE KEEP THIS MAIN FUNCTION CALL 
    STRUCTURE AT A MINIMUM.
    '''
    # run the file with command "python3 main.py question_number" in the terminal
    import os
    question = int(sys.argv[1])
    os.makedirs(f'outputs/{question}/', exist_ok= True)
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
    import json
    with open(f'outputs/{question}/params.json', 'w') as f:
        json.dump(params, f,indent=4,  cls=NumpyEncoder)
    exit()
    main(params)    
    