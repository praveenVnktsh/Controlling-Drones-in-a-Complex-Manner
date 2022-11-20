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
                omega = 0 # by setting omega = 0.5, we can get a velocty of 1 at the end.
                alpha = 0.5/(2*np.pi)
                theta = 0
                # for i in np.arange(0, duration, self.dt):
                i = 0
                while theta < 2 * np.pi / 4 :
                    vec = desstatevec.copy() 
                    theta = 0.5 * alpha * (i**2)

                    vec[0] = 2 * np.cos(theta - np.pi/2)
                    vec[1] = 1 * np.sin(theta - np.pi/2) + 1
                    vec[3] = -2 * np.sin(theta - np.pi/2) * alpha * i
                    vec[4] = 1 * np.cos(theta - np.pi/2) * alpha * i
                    i += self.dt 
                    self.trajectory.append(vec.copy())
                    self.times.append(curtime + i)
                # duration = i
                # for i in np.arange(duration, duration + 6 * np.pi / omega, self.dt):
                #     vec = desstatevec.copy() 
                #     theta = omega * i
                #     vec[0] = 2 * np.cos(theta - np.pi/2)
                #     vec[1] = 1 * np.sin(theta - np.pi/2) + 1
                #     # vec[3] = -2 * np.sin(theta - np.pi/2) * omega 
                #     # vec[4] = 1 * np.cos(theta - np.pi/2) * omega  
                #     self.trajectory.append(vec.copy())
                #     self.times.append(curtime + i)

                # for i in np.arange( 0.25 * 2 * np.pi / omega,  6 * np.pi / omega, self.dt):
                #     vec = desstatevec.copy() 
                #     theta = omega * i
                #     vec[0] = 2 * np.cos(theta - np.pi/2)
                #     vec[1] = 1 * np.sin(theta - np.pi/2) + 1
                #     vec[3] = -2 * np.sin(theta - np.pi/2) / np.sqrt(5)
                #     vec[4] = 1 * np.cos(theta - np.pi/2)/ np.sqrt(5)
                #     self.trajectory.append(vec.copy())
                #     self.times.append(curtime + i)


                # for i in np.arange(  6 * np.pi / omega,  (6 + 0.5) * np.pi / omega, self.dt):
                #     vec = desstatevec.copy() 
                #     nomega = omega * 
                #     theta = (nomega  ) * i
                #     vec[0] = 2 * np.cos(theta - np.pi/2)
                #     vec[1] = 1 * np.sin(theta - np.pi/2) + 1
                #     vec[3] = -2 * np.sin(theta - np.pi/2) / np.sqrt(5)
                #     vec[4] = 1 * np.cos(theta - np.pi/2)/ np.sqrt(5)
                #     self.trajectory.append(vec.copy())
                #     self.times.append(curtime + i)




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