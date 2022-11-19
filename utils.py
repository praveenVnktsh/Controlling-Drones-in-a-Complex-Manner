from enum import Enum
import json
import numpy as np

class State(Enum):

    COMPLETE = 0
    IDLE = 1
    TAKEOFF = 2
    HOVER1 = 3
    TRACK = 4
    HOVER2 = 5
    LAND = 6


class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)