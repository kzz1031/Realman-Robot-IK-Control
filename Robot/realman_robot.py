import numpy as np
import os
import carb
from termcolor import cprint

from isaacsim.core.api.robots import Robot
from isaacsim.core.api import World
from isaacsim.core.utils.rotations import quat_to_rot_matrix

class Realman(Robot):
    def __init__(self, prim_path: str, name: str, world: World):
        super().__init__(prim_path=prim_path, name=name)
        self.world = world
        # Add any Realman-specific initialization here
    