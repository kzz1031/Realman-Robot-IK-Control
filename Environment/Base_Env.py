from isaacsim.simulation_app import SimulationApp
# simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api.objects import GroundPlane
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
import omni.replicator.core as rep
from isaacsim.core.prims import SingleXFormPrim
class BaseEnv:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add(GroundPlane(prim_path="/World/Ground", name="Ground", color=np.array([0.2, 0.2, 0.2])))
        self.demo_light = rep.create.light(position=[0, 0, 0], light_type="dome")

        # self.realman_usd_path = "/home/kzzwang/Documents/code/DigitTwin/Assets/overseas_75_b_v_description/overseas_75_b_v_description_lockplatform2.usd"
        self.realman_usd_path = "/home/kzzwang/Documents/code/DigitTwin/Assets/overseas_75_b_v_description/realman_with_pads_cameras.usd"
        add_reference_to_stage(
            usd_path=self.realman_usd_path,
            prim_path="/World/Realman"
        )

        self.table = SingleXFormPrim(
            prim_path="/World/Table",
            name="Table",
            position=np.array([-0.1, -0.85, 0.12]),
            scale=np.array([1.0, 1.0, 1.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )

        add_reference_to_stage(
            usd_path="/home/kzzwang/Documents/code/DigitTwin/Assets/assembly.usd",
            prim_path="/World/Table"
        )
        

    def reset(self):
        self.world.reset()

    def step(self):
        self.world.step(render=True)

# if __name__ == "__main__":
#     env = BaseEnv()
#     while True:
#         env.step()

# simulation_app.close()