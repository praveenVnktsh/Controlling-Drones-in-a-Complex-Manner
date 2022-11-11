from enum import Enum


class State(Enum):

    COMPLETE = 0
    IDLE = 1
    TRACK = 2
    TAKEOFF = 3
    HOVER = 4
    LAND = 5