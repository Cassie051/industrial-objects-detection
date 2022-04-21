from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.nucleus import find_nucleus_server
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.jetbot import Jetbot
from omni.isaac.motion_generation import WheelBasePoseController
from omni.isaac.jetbot.controllers import DifferentialController
from omni.isaac.synthetic_utils import SyntheticDataHelper
import carb
import numpy as np
import math

class ObjectDetection(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        self.world = self.get_world()
        self.circle_radius = 40
        result, nucleus_server = find_nucleus_server()
        if result is False:
            carb.log_error("Could not find nucleus server with /Isaac folder")
        asset_path = nucleus_server +"/Isaac/Environments/Simple_Warehouse/warehouse.usd" # warehouse_multiple_shelves.usd
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Warehouse")
        asset_path = nucleus_server +"/Isaac/Environments/Simple_Warehouse/Props/SM_PaletteA_01.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Palette")
        self._pallet = self.world.scene.add(Robot(prim_path="/World/Palette", name="palette"))
        # asset_path = nucleus_server +"/Isaac/Environments/Simple_Warehouse/Props/SM_PushcartA_02.usd"
        # add_reference_to_stage(usd_path=asset_path, prim_path="/World/Pushcart"))
        # self._pushcart = self.world.scene.add(Robot(prim_path="/World/Pushcart", name="pushcart"))
        self._jetbot = self.world.scene.add(Jetbot(prim_path="/World/jetbot",
                                        name="fancy_jetbot",
                                        position=np.array([0, 30, 0])))
        return

    async def setup_post_load(self):
        self.rand_object_position(self._pallet)
        # self.rand_object_position(self._pushcart)
        self._world.add_physics_callback("sending_actions", callback_fn=self.send_robot_actions)
        self._my_controller = WheelBasePoseController(name="cool_controller",
                                                    open_loop_wheel_controller=DifferentialController(name="open_loop_controller"),
                                                    is_holonomic=False)
        return

    def rand_object_position(self, object):
        self._world.reset()
        alpha = 2 * math.pi * np.random.rand()
        r = 100 * math.sqrt(np.random.rand()) + 20
        object.set_world_pose(np.array([math.sin(alpha) * r, math.cos(alpha) * r, 2.5]))
        return

    def send_robot_actions(self, step_size):
        position, orientation = self._jetbot.get_world_pose()
        self._jetbot.apply_wheel_actions(self._my_controller.forward(start_position=position,
                                                                     start_orientation=orientation,
                                                                     goal_position=np.array([280, 80], [280, 280], [80, 280], [280, 80])))
        return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        return
