from enum import Enum
from utils import State

from traj import TrajectoryGenerator




class StateManager():

    def __init__(self, params):
        self.state = State.IDLE
        self.prevstate = State.IDLE
        self.params = params
        self.question = params['question']
        self.planner : TrajectoryGenerator = TrajectoryGenerator(params)

    def isComplete(self, t):
        print(self.state, self.planner.complete)
        if self.state == State.COMPLETE or self.planner.complete:
            self.state = State.COMPLETE
            return True
        return False


    def setNextState(self, t, curstatevec):

        self.prevstate = self.state
        if self.question in [2, 3]:
            if self.state == State.IDLE and self.prevstate == State.IDLE:
                self.state = State.TRACK
            elif self.state == State.TRACK:
                self.state = State.COMPLETE

            return

         
        if self.question in [4]:
            

            if(self.state == State.IDLE):
                self.state = State.TAKEOFF
            elif self.state == State.TAKEOFF:
                self.state = State.HOVER
            elif self.state == State.HOVER and self.prevstate == State.TAKEOFF:
                self.state = State.TRACK
            elif self.state == State.TRACK:
                self.state = State.HOVER
            elif self.state == State.HOVER and self.prevstate == State.TRACK:
                self.state = State.LAND
            elif self.state ==  State.LAND:
                self.state = State.IDLE
        print("State Change |", self.prevstate, '->', self.state)
        self.planner.planTrajectory(t, curstatevec, self.state, )



    def getDesiredState(self, t, curstatevec):

        a = self.planner.getDesiredState(t)
        if  a is None:
            self.setNextState()
            self.planner.planTrajectory(t, curstatevec, self.state, )
            a = self.planner.getDesiredState(t)

        return a



    