from isaacsim.simulation_app import SimulationApp
# simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api.objects import GroundPlane
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
import omni.replicator.core as rep

class BaseEnv:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add(GroundPlane(prim_path="/World/Ground", name="Ground", color=np.array([0.2, 0.2, 0.2])))
        self.demo_light = rep.create.light(position=[0, 0, 0], light_type="dome")

        self.realman_usd_path = "/home/kzzwang/Documents/code/DigitTwin/Assets/overseas_75_b_v_description/overseas_75_b_v_description_lockplatform.usd"
        self.realman_urdf_path = "/home/kzzwang/Documents/code/DigitTwin/Assets/realman/rmc_aida_l_ros1-develop2/rmc_aida_l_ros1-develop/src/dual_arm_control/dual_arm_description/overseas_75_b_v_description/urdf/overseas_75_b_v_description.urdf"
        self.realman_discription_path = "/home/kzzwang/Documents/code/DigitTwin/Assets/Lula/realman.yaml"
        
        add_reference_to_stage(
            usd_path=self.realman_usd_path,
            prim_path="/World/Realman"
        )

    def reset(self):
        self.world.reset()

    def step(self):
        self.world.step(render=True)

# if __name__ == "__main__":
#     env = BaseEnv()
#     for _ in range(1000):
#         env.step()

# simulation_app.close()