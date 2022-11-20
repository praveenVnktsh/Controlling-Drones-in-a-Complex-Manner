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