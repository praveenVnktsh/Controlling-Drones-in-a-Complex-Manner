### `controller.py`

```python
import numpy as np

class Controller():
    def __init__ (self, params):
        self.params = params

    def attitude_by_flatness(self, desired_state):
        '''
        Input parameters:
            desired_state: The desired states are:
                desired_state["pos"] = [x, y, z]
                desired_state["vel"] = [x_dot, y_dot, z_dot]
                desired_state["rot"] = [phi, theta, psi]
                desired_state["omega"] = [phidot, thetadot, psidot]
                desired_state["acc"] = [xdotdot, ydotdot, zdotdot]

            params: Quadcopter parameters
            
        Output parameters:
            rot: will be stored as desired_state["rot"] = [phi, theta, psi]
            omega: will be stored as desired_state["omega"] = [phidot, thetadot, psidot]
        
        '''
        params = self.params
        psi = desired_state['rot'][2]
        arr = np.array(
            [
                [np.sin(psi), -np.cos(psi)],
                [np.cos(psi), np.sin(psi)],
            ]
        )
        g = params['gravity']


        desired_state['rot'][:2] = (1/g)*np.dot(arr, desired_state['acc'][:2])

        arr1 = np.array(
            [
                [np.cos(psi), np.sin(psi)],
                [-np.sin(psi), np.cos(psi)],
            ]
        )
        desired_state['omega'][:2] = (1/g)*np.dot(arr1, (desired_state['acc'][:2])*desired_state['omega'][2])
        return desired_state['rot'], desired_state["omega"]


    def attitude_controller(self, current_state,desired_state):
        '''
        Input parameters
        
            current_state: The current state of the robot with the following fields:
                current_state["pos"] = [x, y, z]
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

        Output parameters:
            M: u2 or moment [M1, M2, M3]
        '''
        params = self.params

        Kp = params['kpatt']
        Kd = params['kdatt']


        terror = desired_state['rot'] - current_state['rot']
        terrordot = desired_state['omega'] - current_state['omega']


        errorheadddot = [0, 0, 0]
        for i in range(3):
            errorheadddot[i] = (terror[i] * Kp[i] + terrordot[i] * Kd[i])
        
        errorheadddot = np.array(errorheadddot)
        M = np.dot(params['inertia'], errorheadddot)

        return M


    def position_controller(self, current_state,desired_state):
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
        params = self.params

        Kp = params['kppos']
        Kd = params['kdpos']


        xerror = desired_state['pos'] - current_state['pos']
        xerrordot = desired_state['vel'] - current_state['vel']
        accs = [0, 0, 0]

        for i in range(3):
            accs[i] = (xerror[i] * Kp[i] + xerrordot[i] * Kd[i])

        accs = np.array(accs)   
        # print(current_state['pos'], current_state['vel'], desired_state['vel'])


        g = params['gravity']
        F = params['mass'] * (accs + desired_state["acc"] + np.array([0, 0, g]))
        return F, accs
```

### `drone.py`

```python
import numpy as np
from scipy.integrate import solve_ivp


class Drone:

    def __init__(self, params, initState = np.zeros((16,))):
        self.params = params
        self.state = initState
        self.thrust = 0

    @staticmethod
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

    
    def motor_model(self, F,M):

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
        params = self.params
        rpm = self.state[12:]
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
        rpmd = (np.dot(np.linalg.inv(A), np.array([F[2], M[0], M[1], M[2]])))
        rpmd = np.clip(rpmd, params['rpm_min'] ** 2, params['rpm_max'] ** 2)
        rpmd = np.sqrt(rpmd)
        rpm = np.clip(rpm,  params['rpm_min'], params['rpm_max'])
        vals = np.dot(A, np.square(rpm))
        rpm_dot = params['motor_constant'] *(rpmd - rpm)
        F_motor = vals[0]
        M_motor = vals[1:]

        return F_motor, M_motor, rpm_dot


    def step(self, F_desired, M_desired ):
        [F_actual,M_actual,rpm_motor_dot] = self.motor_model(F_desired,M_desired)
        self.thrust = np.linalg.norm(F_actual)

        sol = solve_ivp(
            self.dynamics, 
            (0, self.params['dt']), 
            self.state,
            args=(self.params,F_actual,M_actual,rpm_motor_dot),)
        self.state = sol.y[:,-1]

        return sol
```

### `main_q.py`

```python
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
    if params['question'] == 2:
        initState[2] = 0.5
    drone = Drone(params, initState =initState)
    controller = Controller(params)

    i = 0

    t = 0
    times = []

    stateManager.setNextState(t, drone.state)
    
    thrustToMassPlot = []

    trackingIntervalPlot = {
        'actualstates': [],
        'desiredstates' : [],
        'times' : []
    }
    velocities = []
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
            trackingIntervalPlot['times'].append(t)

        times.append(t)
        thrustToMassPlot.append((drone.thrust) / (params['mass'] * params['gravity']))
        velocities.append(np.linalg.norm(state_list[3:6]))
        # if stateManager.state != State.TAKEOFF:
        #     break

    print("Statemanager isComplete", stateManager.isComplete())
    trackingIntervalPlot['desiredstates'] = np.array(trackingIntervalPlot['desiredstates']).T.squeeze()
    trackingIntervalPlot['actualstates'] = np.array(trackingIntervalPlot['actualstates']).T.squeeze()
    trackingIntervalPlot['times'] = np.array(trackingIntervalPlot['times'])

    desired_states = np.array(desired_states).T.squeeze()
    actual_states = np.array(actual_states).T.squeeze()
    time_vec = np.array(times)
    plotDic = {
        'FbyW' : thrustToMassPlot,
        'velocities' : velocities,
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
    plt.savefig(f'outputs/{params["question"]}/{params["plotprefix"]}_fbyw.png')

    plt.figure()
    plt.plot(plotDic['time_vec'], plotDic['velocities'])
    plt.title("Velocity over time")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.savefig(f'outputs/{params["question"]}/{params["plotprefix"]}_velocities.png')

    plot_state_error(plotDic["actual_states"],plotDic["desired_states"],plotDic["time_vec"], params)
    plot_state_error(plotDic["trackingintervals"]['actualstates'],plotDic["trackingintervals"]['desiredstates'],plotDic['trackingintervals']["times"], params, isTrack = True)
    plot_position_3d(plotDic["actual_states"],plotDic["desired_states"], params)
    if question == 5:
        plotq5(plotDic, params)


        
def main(params):
    manager = StateManager(params)
    plotDic = execute(params, manager)
    print("Execution complete...")

    print("Plotting...")
    plot(plotDic, params)
    plt.close("all")

        
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
        'kpatt': [190, 190, 70], 
        'kdatt' : [30,30, 18], 
        'kppos':[20, 20, 18], 
        'kdpos': [8, 8, 9], 
        'question' : question,
        'q5trackpsi' : 0,
        'dt' : 0.005,
        'plotprefix' : 'a'      
    }
    import json
    with open(f'outputs/{question}/params.json', 'w') as f:
        json.dump(params, f,indent=4,  cls=NumpyEncoder)

    main(params)   

    if question == 5:
        

        params['q5trackpsi'] = 15 * np.pi / 180
        params['plotprefix'] = 'b'
        main(params)

        params['q5trackpsi'] = 15 * np.pi / 180
        params['plotprefix'] = 'gains2'
        params['kpatt'] = [190, 190, 20]
        params['kdatt'] = [30, 30, 18]
        params['kppos'] = [20, 20, 10]
        params['kdpos'] = [8, 8, 19]
        
        main(params)
    
    if question in [2, 3]:
        
        params['plotprefix'] = 'gains2_kdpos'
        params['kpatt'] = [190, 190, 70]
        params['kdatt'] = [30, 30, 18]
        params['kppos'] = [20, 20, 10]
        params['kdpos'] = [12, 12, 12]
        
        main(params)

        params['plotprefix'] = 'gains3_kppos'
        params['kpatt'] = [190, 190, 20]
        params['kdatt'] = [30, 30, 18]
        params['kppos'] = [25, 25, 10]
        params['kdpos'] = [8, 8, 9]
        
        main(params)


        params['plotprefix'] = 'gains4_kpatt'
        params['kpatt'] = [220, 220, 20]
        params['kdatt'] = [30, 30, 18]
        params['kppos'] = [25, 25, 10]
        params['kdpos'] = [8, 8, 9]
        
        main(params)

        params['plotprefix'] = 'gains4_kdatt'
        params['kpatt'] = [190, 190, 20]
        params['kdatt'] = [35, 35, 20]
        params['kppos'] = [25, 25, 10]
        params['kdpos'] = [8, 8, 9]
        
        main(params)
    if question == 3:

        params['plotprefix'] = 'gains2_kdpos'
        params['kpatt'] = [190, 190, 70]
        params['kdatt'] = [30, 30, 18]
        params['kppos'] = [20, 20, 10]
        params['kdpos'] = [12, 12, 12]
        
        main(params)

        params['plotprefix'] = 'gains3_kppos'
        params['kpatt'] = [190, 190, 20]
        params['kdatt'] = [30, 30, 18]
        params['kppos'] = [25, 25, 10]
        params['kdpos'] = [8, 8, 9]
        
        main(params)
```


### `stateManager.py`

```python
from enum import Enum
from utils import State

from traj import TrajectoryGenerator

import numpy as np


class StateManager():

    def __init__(self, params):
        self.state = State.IDLE
        self.prevstate = State.IDLE
        self.params = params
        self.question = params['question']
        self.planner : TrajectoryGenerator = TrajectoryGenerator(params)

    def isComplete(self):
        if self.state == State.COMPLETE:
            self.state = State.COMPLETE
            return True
        return False


    def setNextState(self, t, curstatevec):

        
        if self.question  == 1:
            if self.state == State.IDLE:
                self.state = State.HOVER1
            elif self.state == State.HOVER1:
                self.state = State.COMPLETE

        if self.question in [2, 3]:
            if self.state == State.IDLE:
                self.state = State.TRACK
            elif self.state == State.TRACK:
                self.state = State.COMPLETE

        if self.question in [4, 5, 8, 9]:
            self.prevstate = self.state
            self.state = State((self.state.value + 1) % 7)
        

        
        print("State Change |", self.prevstate, '->', self.state)
        self.planner.planTrajectory(t, curstatevec, self.state, )
        # print(self.planner.trajectory)



    def getDesiredState(self, t, curstatevec):
        err = np.linalg.norm(self.planner.trajectory[-1][:3] - curstatevec[:3])
        print( curstatevec[:3])
        if self.planner.complete and  err < 0.045:
            print("REACHED")
            self.setNextState(t, curstatevec)
        a = None        
        if not self.isComplete():
            a = self.planner.getDesiredState(t)
            

        return a
```

### `traj.py`

```python
import numpy as np
from stateManager import State

class TrajectoryGenerator():

    def __init__(self, params):
        self.question = params['question']
        self.dt = params['dt']
        self.params = params
        
        self.trajectory = []
        self.times = []

        self.curtrackpoint = 0
        self.starttime = 0
        self.endtime = 0
        self.complete = False

    def getDesiredState(self, t):
        ind = max(0, min(len(self.trajectory) - 1, int((t - self.starttime)/self.dt)))
        print(self.trajectory[ind][:3], end = ' | ')
        if int((t - self.starttime)/self.dt) > len(self.trajectory) - 1:
        

            self.complete = True

        traj = self.trajectory[ind]
        desired_state_dic = {
            "pos":  traj[0:3],
            "vel":  traj[3:6],
            "rot":  traj[6:9], 
            "omega":traj[9:12],
            "rpm":  traj[12:16],
            'acc' : np.array([0, 0, 0])
        }
        
        
        return desired_state_dic

    def planTrajectory(self, curtime, curstatevec = None, state = None):
        '''
        Input parameters:
            curtime (float): current time stamp
            curstatevec (np.ndarray): current state vector of the drone
            state (State(Enum)) : current state in state machine.

        Stores generated trajectories in class.
        '''

        print("Planning Trajectory for", state,)
        self.complete = False

        self.trajectory = []
        self.times = []

        desstatevec = np.zeros_like(curstatevec)
        desstatevec[:3] = curstatevec[:3]

        desstatevec[8] = curstatevec[8]

        if state == State.IDLE:
            for i in np.arange(0, 2, self.dt):
                self.trajectory.append(desstatevec.copy())
                self.times.append(curtime + i)

        elif state in [State.HOVER1, State.HOVER2]:
            for i in np.arange(0, 3, self.dt):
                self.trajectory.append(desstatevec.copy())
                self.times.append(curtime + i)

        elif state == State.TAKEOFF:
            duration = 2.0
            for i in np.arange(0, duration, self.dt):
                vec = desstatevec.copy() 
                vec[2] = 0.4*(i/duration)
                vec[5] = 1.0/duration
                self.trajectory.append(vec.copy())
                self.times.append(curtime + i)
            for i in np.arange(0, 0.5, self.dt):
                vec = desstatevec.copy() 
                vec[2] = 0.5
                vec[5] = 0
                self.trajectory.append(vec.copy())
                self.times.append(duration + curtime + i)

        elif state == State.TRACK:
            question = self.question
            if question == 4:
                duration = 6.
                for i in np.arange(0, duration, self.dt):
                    vec = desstatevec.copy() 
                    vec[0] = 0.3 * (i/duration)
                    # omega = 2*np.pi * i/duration
                    # vec[0] = vec[0] + 0.3 * np.cos(omega)
                    # vec[1] = vec[1] + 0.3 * np.sin(omega)

                    vec[5] = 0.3/duration
                    self.trajectory.append(vec.copy())
                    self.times.append(curtime + i)
            if question == 5:
                duration = 6.
                for i in np.arange(0, duration, self.dt):
                    vec = desstatevec.copy() 
                    vec[2] = 0.1
                    vec[8] = self.params['q5trackpsi']
                    
                    self.trajectory.append(vec.copy())
                    self.times.append(curtime + i)
            if question == 2:
                duration = 2
                for loc in [0.0, 0.1, 0.2, 0.3, 0.4]:
                    for i in np.arange(0, duration, self.dt):
                        vec = desstatevec.copy() 
                        vec[0] = loc
                        self.trajectory.append(vec.copy())
                        self.times.append(curtime + i)
                
            if question == 3:
                duration = 4
                a = 2 * 1.0 /  (duration**2)
                for i in np.arange(0, duration, self.dt):
                    vec = desstatevec.copy() 
                    vec[2] = 0.5 * a * (i**2)
                    vec[5] = a * i
                    self.trajectory.append(vec.copy())
                    self.times.append(curtime + i)
                for i in np.arange(duration, duration + 2, self.dt):
                    vec = desstatevec.copy() 
                    vec[2] = 1.0
                    vec[5] = 0
                    self.trajectory.append(vec.copy())
                    self.times.append(curtime + i)

                for i in np.arange(duration + 2, 2*duration + 2, self.dt):
                    vec = desstatevec.copy() 
                    vec[2] = 0.5 * a * ((2*duration + 2 - i)**2)
                    vec[5] = -a * (2*duration + 2 - i )
                    self.trajectory.append(vec.copy())
                    self.times.append(curtime + i)
            if question == 8:
                omega = 0.5
                i = 0
                for i in np.arange(0, 24 * np.pi * omega, self.dt):
                    vec = desstatevec.copy() 
                    theta = omega * i

                    vec[0] = 2 * np.cos(theta - np.pi/2)
                    vec[1] = 1 * np.sin(theta - np.pi/2) + 1
                    i += self.dt 
                    self.trajectory.append(vec.copy())
                    self.times.append(curtime + i)

            if question == 9:
                omega = 0.5
                i = 0
                for i in np.arange(0, 24 * np.pi * omega, self.dt):
                    vec = desstatevec.copy() 
                    theta = omega * i

                    vec[0] = 2 * np.cos(theta - np.pi/2)
                    vec[1] = 1 * np.sin(theta - np.pi/2) + 1
                    vec[8] = theta
                    i += self.dt 
                    self.trajectory.append(vec.copy())
                    self.times.append(curtime + i)
                




        elif state == State.LAND:
            height = desstatevec[2]
            duration = 4.0
            for i in np.arange(0, duration, self.dt):
                vec = desstatevec.copy() 
                vec[2] = height*(1 - i/duration)
                # vec[5] = -1/duration
                self.trajectory.append(vec.copy())
                self.times.append(curtime + i)

        self.trajectory = np.array(self.trajectory).squeeze()
        self.times = np.array(self.times)
        self.starttime = curtime


        # height of 15 for: [x, y, z, xdot, ydot, zdot, phi, theta, psi, phidot, thetadot, psidot, xacc, yacc, zacc]
```

### `utils.py`

```python
from enum import Enum
import json
import numpy as np

class State(Enum):

    COMPLETE = 0
    IDLE = 1
    TAKEOFF = 2
    HOVER1 = 3
    TRACK = 4
    HOVER2 = 5
    LAND = 6


class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)
```