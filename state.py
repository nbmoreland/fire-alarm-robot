from enum import Enum


class State(Enum):
    WANDER = 1
    WALL_FOLLOWING = 2
    FIRE_DETECTION = 3
    EXTINGUISH = 4
