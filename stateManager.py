

class StateManager():

    def __init__(self):
        self.state = 'IDLE'
        self.prevstate = "IDLE"


    def setNextState(self):
        self.prevstate = self.state

        if(self.state == 'IDLE'):
            self.state = 'TAKEOFF'
        elif self.state == 'TAKEOFF':
            self.state = 'HOVER'
        elif self.state == 'HOVER' and self.prevstate == 'TAKEOFF':
            self.state = 'TRACK'
        elif self.state == 'TRACK':
            self.state = 'HOVER'
        elif self.state == 'HOVER' and self.prevstate == 'TRACK':
            self.state = 'LAND'

    def getWaypoints(self):

        if self.state == 'IDLE':
            pass