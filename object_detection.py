from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.motion_generation import WheelBasePoseController
from omni.isaac.jetbot.controllers import DifferentialController
from omni.isaac.synthetic_utils import SyntheticDataHelper
from omni.isaac.examples.objects_detection.robot_task import RobotTask
import carb
import omni
import numpy as np

class ObjectDetection(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        from omni.isaac.core.utils.nucleus import find_nucleus_server
        from omni.isaac.core.utils.stage import add_reference_to_stage
        from omni.isaac.core.prims import GeometryPrim
        world = self.get_world()
        world.stage_units_in_meters=0.01
        world.add_task(RobotTask(name="awesome_task"))
        # world.scene.add_default_ground_plane()
        result, nucleus_server = find_nucleus_server()
        if result is False:
            carb.log_error("Could not find nucleus server with /Isaac folder")
        asset_path = nucleus_server +"/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd" # warehouse.usd 
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Warehouse")
        # asset_path = nucleus_server +"/Isaac/Environments/Simple_Warehouse/Props/SM_PaletteA_01.usd"
        # add_reference_to_stage(usd_path=asset_path, prim_path="/World/Palette")
        # self._pallet = world.scene.add(GeometryPrim(prim_path="/World/Palette", name="palette"))
        # asset_path = nucleus_server +"/Isaac/Environments/Simple_Warehouse/Props/SM_PushcartA_02.usd"
        # add_reference_to_stage(usd_path=asset_path, prim_path="/World/Pushcart")
        # self._pushcart = world.scene.add(GeometryPrim(prim_path="/World/Pushcart", name="pushcart"))
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self.set_camera()
        # self.rand_object_position(self._pallet)
        # self.rand_object_position(self._pushcart)
        task_params = self._world.get_task("awesome_task").get_params()
        self._jetbot = self._world.scene.get_object(task_params["jetbot_name"]["value"])
        self._my_controller = WheelBasePoseController(name="cool_controller",
                                                    open_loop_wheel_controller=DifferentialController(name="open_loop_controller"),
                                                    is_holonomic=False)
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        await self._world.play_async()
        return

    async def setup_post_reset(self):
        self._my_controller.reset()
        await self._world.play_async()
        return

    def rand_object_position(self, object):
        y = np.random.randint(100, 400)
        object.set_world_pose(np.array([150, y, 2.5]))
        return

    def physics_step(self, step_size):
        current_observations = self._world.get_observations()
        self._jetbot.apply_wheel_actions(
            self._my_controller.forward(
                start_position=current_observations[self._jetbot.name]["position"],
                start_orientation=current_observations[self._jetbot.name]["orientation"],
                goal_position=np.array(current_observations[self._jetbot.name]["goal_position"])))
        return

    def set_camera(self):
        camera_path = "/World/jetbot/chassis/rgb_camera/jetbot_camera"
        viewport_handle = omni.kit.viewport.get_viewport_interface().create_instance()
        viewport_window = omni.kit.viewport.get_viewport_interface().get_viewport_window(viewport_handle)
        viewport_window.set_active_camera(camera_path)
        viewport_window.set_texture_resolution(640, 640)
        viewport_window.set_window_pos(1000, 400)
        viewport_window.set_window_size(420, 420)
        self.viewport_window = viewport_window
        self.sd_helper = SyntheticDataHelper()
        self.sd_helper.initialize(sensor_names=["rgb"], viewport=self.viewport_window)
        self.sd_helper.get_groundtruth(["rgb"], self.viewport_window)
        return

    def close(self):
        self._simulation_app.close()
        return