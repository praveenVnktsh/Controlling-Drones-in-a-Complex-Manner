import numpy as np
def attitude_by_flatness(desired_state,params):
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


def attitude_controller(params, current_state,desired_state,question):
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
    # Example PD gains
    Kpphi = 190
    Kdphi = 30

    Kptheta = 198
    Kdtheta = 30

    Kppsi = 80
    Kdpsi = 17.88

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
    # TO DO:

