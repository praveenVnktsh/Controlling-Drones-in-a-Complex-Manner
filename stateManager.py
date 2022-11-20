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

        if self.question in [4, 5, 8]:
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



    