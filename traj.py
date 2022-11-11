import numpy as np
from stateManager import State



# def lookup_waypoints(question):
#     '''
#     Input parameters:

#     question: which question of the project we are on 
#     (Possible arguments for question: 2, 3, 5, 6.2, 6.3, 6.5, 7, 9, 10)

#     Output parameters:

#     waypoints: of the form [x, y, z, yaw]
 
#     waypoint_times: vector of times where n is the number of waypoints, 
#     represents the seconds you should be at each respective waypoint
#     '''

#     # TO DO:

#     # sample waypoints for hover trajectory 
#     if question == 2:
#         waypoints = np.array([[0, 0.1, 0.2, 0.3],[0, 0, 0, 0], [0.5, 0.5, 0.5, 0.5], [0,0,0,0]])
#         waypoint_times = np.array([0,2,4,6])
#     elif question == 3:
#         print("Computing waypoints!")
#         timeduration = 4
#         zs = []
#         times = []
#         a = 2/(timeduration**2)
        
#         for t in np.arange(0, timeduration, 0.1):
#             zs.append(0.5 * a * t**2)
#             times.append(t)


#         zs1 = zs[::-1]
#         zs = np.hstack((zs, zs1))
#         ys = np.zeros_like(zs)
#         xs = np.zeros_like(zs)
#         times = np.hstack((times, np.array(times) + timeduration + 2 ))


#         waypoints = np.array([xs, ys, zs, np.zeros_like(zs)])
#         waypoint_times = np.array(times)
#     return([waypoints, waypoint_times])


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
        ind = int(t - self.starttime)/self.dt
        if ind > len(self.trajectory):
            self.complete = True
            return None

        traj = self.trajectory[ind]
        desired_state_dic = {
            "pos":  traj[0:3],
            "vel":  traj[3:6],
            "rot":  traj[6:9], 
            "omega":traj[9:12],
            "rpm":  traj[12:16]
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
        print("Planning Trajectory for ", end = '')
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

        elif state == State.HOVER:
            for i in np.arange(0, 2, self.dt):
                self.trajectory.append(desstatevec.copy())
                self.times.append(curtime + i)

        elif state == State.TAKEOFF:
            print(state)
            for i in np.arange(0, 4, self.dt):
                vec = desstatevec.copy() 
                vec[2] = i/4.
                vec[5] = 1/4.
                self.trajectory.append(vec.copy())
                self.times.append(curtime + i)

        elif state == State.TRACK:
            duration = 6.
            for i in np.arange(0, duration, self.dt):
                vec = desstatevec.copy() 
                vec[0] = 0.3 * (i/duration)
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

        # height of 15 for: [x, y, z, xdot, ydot, zdot, phi, theta, psi, phidot, thetadot, psidot, xacc, yacc, zacc]