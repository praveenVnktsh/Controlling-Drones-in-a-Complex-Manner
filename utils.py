from enum import Enum


class State(Enum):

    COMPLETE = 0
    IDLE = 1
    TAKEOFF = 2
    HOVER1 = 3
    TRACK = 4
    HOVER2 = 5
    LAND = 6
