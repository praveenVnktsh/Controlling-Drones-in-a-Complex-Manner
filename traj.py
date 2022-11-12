import numpy as np
from stateManager import State

class TrajectoryGenerator():

    def __init__(self, params):
        self.question = params['question']
        self.dt = params['dt']
        
        self.trajectory = []
        self.times = []

        self.curtrackpoint = 0
        self.starttime = 0
        self.endtime = 0
        self.complete = False

    def getDesiredState(self, t):
        ind = max(0, min(len(self.trajectory) - 1, int((t - self.starttime)/self.dt)))
        # if ind > len(self.trajectory) - 1:
        #     self.complete = True
        #     return None

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
            for i in np.arange(0, 1, self.dt):
                self.trajectory.append(desstatevec.copy())
                self.times.append(curtime + i)

        elif state == State.TAKEOFF:
            duration = 4.0
            for i in np.arange(0, duration, self.dt):
                vec = desstatevec.copy() 
                vec[2] = i/duration
                # vec[5] = 1.0/duration
                self.trajectory.append(vec.copy())
                self.times.append(curtime + i)

        elif state == State.TRACK:
            duration = 6.
            for i in np.arange(0, duration, self.dt):
                vec = desstatevec.copy() 
                # vec[0] = 0.3 * (i/duration)
                omega = 2*np.pi * i/duration
                vec[0] = vec[0] + 0.3 * np.cos(omega)
                vec[1] = vec[1] + 0.3 * np.sin(omega)

                vec[5] = 0.3/duration
                self.trajectory.append(vec.copy())
                self.times.append(curtime + i)

        elif state == State.LAND:
            height = desstatevec[2]
            duration = 4.0
            for i in np.arange(0, duration, self.dt):
                vec = desstatevec.copy() 
                vec[2] = height - i/duration
                vec[5] = -1/duration
                self.trajectory.append(vec.copy())
                self.times.append(curtime + i)

        self.trajectory = np.array(self.trajectory).squeeze()
        self.times = np.array(self.times)
        self.starttime = curtime


        # height of 15 for: [x, y, z, xdot, ydot, zdot, phi, theta, psi, phidot, thetadot, psidot, xacc, yacc, zacc]