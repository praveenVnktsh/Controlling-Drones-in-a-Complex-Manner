from enum import Enum


class State(Enum):

    COMPLETE = -1
    IDLE = 1
    TRACK = 2
    TAKEOFF = 3
    HOVER = 4
    LAND = 5

class StateManager():

    def __init__(self, question):
        self.state = State.IDLE
        self.prevstate = State.IDLE
        self.question = question

    def isComplete(self):
        return self.state == State.COMPLETE

    def setNextState(self):

        if self.question in [2, 3]:
            if(self.state == State.IDLE):
                self.state = State.TRACK
            elif self.state == State.TRACK:
                self.state = State.COMPLETE

            return

         

        self.prevstate = self.state

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

    def getDesiredState(self, t):
        
        desired_state = {"pos":trajectory_matrix[0:3,i],"vel":trajectory_matrix[3:6,i],\
            "rot":trajectory_matrix[6:9,i],"omega":trajectory_matrix[9:12,i],"acc":trajectory_matrix[12:15,i]}
        pass
        

    