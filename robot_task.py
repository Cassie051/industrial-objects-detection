from omni.isaac.core.tasks import BaseTask
from omni.isaac.jetbot import Jetbot
import numpy as np


class RobotTask(BaseTask):
    def __init__(
        self,
        name
    ):
        super().__init__(name=name, offset=None)
        self.goals_list = [[300, 0], [300, 500]]
        self._task_event = 0
        self._accuracy = 20
        return

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._jetbot = scene.add(Jetbot(prim_path="/World/jetbot",
                                        name="fancy_jetbot",
                                        position=np.array([0, 0, 2.5]),
                                        orientation=np.array([0,0,0,0])))
        self.params_representation = {"jetbot_name": {}}
        return

    def get_observations(self):
        current_jetbot_position, current_jetbot_orientation = self._jetbot.get_world_pose()
        observations= {
            "task_event": self._task_event,
            self._jetbot.name: {
                "position": current_jetbot_position,
                "orientation": current_jetbot_orientation,
                "goal_position": self.goals_list[self._task_event]
            }
        }
        return observations

    def get_params(self):
        self.params_representation["jetbot_name"] = {"value": self._jetbot.name, "modifiable": False}
        return self.params_representation

    def pre_step(self, control_index, simulation_time):
        current_jetbot_position, _ = self._jetbot.get_world_pose()
        diff_x = np.abs(current_jetbot_position[0] - self.goals_list[self._task_event][0])
        diff_y = np.abs(current_jetbot_position[1] - self.goals_list[self._task_event][1])
        if self._task_event == 0:
            if diff_x + diff_y < self._accuracy:
                self._task_event += 1
        elif self._task_event == 1:
            if diff_x + diff_y < self._accuracy:
                self._task_event = 0
        return

    def post_reset(self):
        self._task_event = 0
        return