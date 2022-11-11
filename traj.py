import numpy as np



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

    def __init__(self, question):
        self.question = question

    def trajectory_planner(self, waypoints):
        '''
        Input parameters:
            waypoints (list) : List of tuples ordered as milestones in state (state, t)
        Output parameters:
            trajectory_sate: [15 x max_iter] interpolated trajectory as a matrix of states:
            [x, y, z, xdot, ydot, zdot, phi, theta, psi, phidot, thetadot, psidot, xacc, yacc, zacc]   
        '''

        trajectory_state = np.zeros((15, max_iteration))
        # height of 15 for: [x, y, z, xdot, ydot, zdot, phi, theta, psi, phidot, thetadot, psidot, xacc, yacc, zacc]

        current_waypoint_number = 0
        for i in range(0,max_iteration):
            if current_waypoint_number < len(waypoint_times)-1:
                if (i*time_step) >= waypoint_times[current_waypoint_number+1]:
                    current_waypoint_number +=1

            trajectory_state[0:3, i] = waypoints[0:3, current_waypoint_number]
            trajectory_state[8,i] = waypoints[3, current_waypoint_number]
        
        return trajectory_state