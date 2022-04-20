# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.nucleus import find_nucleus_server
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.jetbot import Jetbot
import numpy as np
import carb

# Note: checkout the required tutorials at https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html


class ObjectDetection(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        result, nucleus_server = find_nucleus_server()
        if result is False:
            carb.log_error("Could not find nucleus server with /Isaac folder")
        asset_path = nucleus_server +"/Isaac/Environments/Simple_Warehouse/warehouse.usd" # warehouse_multiple_shelves.usd
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Warehouse")
        self._jetbot = world.scene.add(Jetbot(prim_path="/World/Fancy_Jetbot",
                                        name="fancy_jetbot",
                                        position=np.array([0, 30, 0])))
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self.log_info("Num of degrees of freedom after first reset: " + str(self._jetbot.num_dof)) # prints 2
        self.log_info("Joint Positions after first reset: " + str(self._jetbot.get_joint_positions()))
        return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        return
