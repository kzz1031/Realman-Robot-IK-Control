from isaacsim.simulation_app import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api.objects import GroundPlane
from isaacsim.core.api import World
import omni.replicator.core as rep


class BaseEnv:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add(GroundPlane(prim_path="/World/Ground", name="Ground", color=np.array([0.2, 0.2, 0.2])))
        self.demo_light = rep.create.light(position=[0, 0, 0], light_type="dome")
    def reset(self):
        self.world.reset()

    def step(self):
        self.world.step()