from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid,VisualCuboid
from omni.isaac.core.robots import Robot
from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.utils.prims import is_prim_path_valid
# robot
from omni.isaac.motion_generation import WheelBasePoseController
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers import PickPlaceController, StackingController
from omni.isaac.franka.tasks import PickPlace,Stacking
#others
import numpy as np
import time
import carb

franka_position=np.array([0.0, 0, 0])
wheel_position=np.array([0.0, 0.3, 0])
cube_position=np.array([0.1, 0.3, 0.03])
target_position=np.array([0.5, 0.5, 0.28])
wheel_target=np.array([0.27, 0.3, 0])
wheel_target2=np.array([0.8, 0.8, 0])
wheel_target3=np.array([0.8, 0.6, 0])
wheel_target4=np.array([0.6, 0.5, 0])

# class HelloWorld(BaseSample):
#     def __init__(self) -> None:
#         super().__init__()
#         return

#     def setup_scene(self):
#         world = self.get_world()
#         # We add the task to the world here
#         world.add_task(PickPlace(name="awesome_task"))
#         return

#     async def setup_post_load(self):
#         self._world = self.get_world()
#         # The world already called the setup_scene from the task so
#         # we can retrieve the task objects
#         # Each defined task in the robot extensions
#         # has set_params and get_params to allow for changing tasks during
#         # simulation, {"task_param_name": "value": [value], "modifiable": [True/ False]}
#         task_params = self._world.get_task("awesome_task").get_params()
#         self._franka = self._world.scene.get_object(task_params["robot_name"]["value"])
#         self._cube_name = task_params["cube_name"]["value"]
#         self._controller = PickPlaceController(
#             name="pick_place_controller",
#             gripper=self._franka.gripper,
#             robot_articulation=self._franka,
#         )
#         self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
#         await self._world.play_async()
#         return

#     async def setup_post_reset(self):
#         self._controller.reset()
#         await self._world.play_async()
#         return

#     def physics_step(self, step_size):
#         # Gets all the tasks observations
#         current_observations = self._world.get_observations()
#         actions = self._controller.forward(
#             picking_position=current_observations[self._cube_name]["position"],
#             placing_position=current_observations[self._cube_name]["target_position"],
#             current_joint_positions=current_observations[self._franka.name]["joint_positions"],
#         )
#         self._franka.apply_action(actions)
#         if self._controller.is_done():
#             self._world.pause()
#         return

###################################################################################################

# class HelloWorld(BaseSample):
#     def __init__(self) -> None:
#         super().__init__()
#         self._controller = None
#         self._articulation_controller = None

#     def setup_scene(self):
#         world = self.get_world()
#         world.add_task(Stacking(name="stacking_task",target_position=np.array([1,1,0])))
#         return

#     async def setup_post_load(self):
#         self._franka_task = self._world.get_task(name="stacking_task")
#         self._task_params = self._franka_task.get_params()
#         my_franka = self._world.scene.get_object(self._task_params["robot_name"]["value"])
#         self._controller = StackingController(
#             name="stacking_controller",
#             gripper_dof_indices=my_franka.gripper.dof_indices,
#             robot_articulation=my_franka,
#             picking_order_cube_names=self._franka_task.get_cube_names(),
#             robot_observation_name=my_franka.name,
#         )
#         self._articulation_controller = my_franka.get_articulation_controller()
#         return

#     def _on_stacking_physics_step(self, step_size):
#         observations = self._world.get_observations()
#         actions = self._controller.forward(observations=observations)
#         self._articulation_controller.apply_action(actions)
#         if self._controller.is_done():
#             self._world.pause()
#         return

#     async def _on_stacking_event_async(self):
#         world = self.get_world()
#         world.add_physics_callback("sim_step", self._on_stacking_physics_step)
#         await world.play_async()
#         return

#     async def setup_pre_reset(self):
#         world = self.get_world()
#         if world.physics_callback_exists("sim_step"):
#             world.remove_physics_callback("sim_step")
#         self._controller.reset()
#         return

#     def world_cleanup(self):
#         self._controller = None
#         return

###################################################################################################

class RobotsPlaying(BaseTask):
    # Adding offset to move the task objects with and the targets..etc
    def __init__(self, name, offset=None):
        super().__init__(name=name, offset=offset)
        self._task_event = 0
        self._jetbot_goal_position =wheel_target# + self._offset #defined in the base task
        self._pick_place_task = PickPlace(cube_initial_position=cube_position,
                                        target_position=target_position,
                                        offset=offset)
        return

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        #This will already translate the pick and place assets by the offset
        self._pick_place_task.set_up_scene(scene)
        # Find a unique scene name
        jetbot_name = find_unique_string_name(
            initial_name="fancy_jetbot", is_unique_fn=lambda x: not self.scene.object_exists(x)
        )
        # Find a unique prim path
        jetbot_prim_path = find_unique_string_name(
            initial_name="/World/Fancy_Jetbot", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        assets_root_path = get_assets_root_path()
        jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        self._jetbot = scene.add(
            WheeledRobot(
                prim_path=jetbot_prim_path,
                name=jetbot_name,
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True,
                usd_path=jetbot_asset_path,
                position=wheel_position,
            )
        )
        # Add Jetbot to this task objects
        self._task_objects[self._jetbot.name] = self._jetbot
        pick_place_params = self._pick_place_task.get_params()
        self._franka = scene.get_object(pick_place_params["robot_name"]["value"])
        # translate the franka by 100 in the x direction
        current_position, _ = self._franka.get_world_pose()
        self._franka.set_world_pose(position=current_position + franka_position)
        self._franka.set_default_state(position=current_position +franka_position)
        # This will only translate the task_objects by the offset specified (defined in the BaseTask)
        # Note: PickPlace task objects were already translated when setting up its scene
        self._move_task_objects_to_their_frame()
        return

    def get_observations(self):
        current_jetbot_position, current_jetbot_orientation = self._jetbot.get_world_pose()
        observations= {
            "task_event": self._task_event,
            self._jetbot.name: {
                "position": current_jetbot_position,
                "orientation": current_jetbot_orientation,
                "goal_position": self._jetbot_goal_position
            }
        }
        observations.update(self._pick_place_task.get_observations())
        return observations

    def get_params(self):
        pick_place_params = self._pick_place_task.get_params()
        params_representation = pick_place_params
        params_representation["jetbot_name"] = {"value": self._jetbot.name, "modifiable": False}
        params_representation["franka_name"] = pick_place_params["robot_name"]
        return params_representation

    def pre_step(self, control_index, simulation_time):
        if self._task_event == 0:
            current_jetbot_position, _ = self._jetbot.get_world_pose()
            #print(np.mean(np.abs(current_jetbot_position[:2] -wheel_target[:2])) < 0.05)
            if np.mean(np.abs(current_jetbot_position[:2] -wheel_target[:2])) < 0.05:
                self._task_event += 1
                self._cube_arrive_step_index = control_index
        elif self._task_event == 1:
            if control_index - self._cube_arrive_step_index>10:
                self._task_event += 1
            # if self._pick_place_task.is_done():
            #     self._task_event += 1
            #     self._jetbot_goal_position=wheel_target2
        elif self._task_event == 2:
            cube=self._pick_place_task.get_task_objects()["cube"]
            pos, _=cube.get_world_pose()
            if np.mean(np.abs(pos[:2]-target_position[:2])) < 0.05 and pos[-1]<0.15:
                self._task_event += 1
                self._jetbot_goal_position=wheel_target2
        elif self._task_event==3:
            current_jetbot_position, _ = self._jetbot.get_world_pose()
            if np.mean(np.abs(current_jetbot_position[:2] -wheel_target2[:2])) < 0.05:
                self._task_event += 1
                self._jetbot_goal_position=wheel_target3
        elif self._task_event==4:
            current_jetbot_position, _ = self._jetbot.get_world_pose()
            if np.mean(np.abs(current_jetbot_position[:2] -wheel_target3[:2])) < 0.05:
                self._task_event += 1
                self._jetbot_goal_position=wheel_target4
        elif self._task_event==5:
            current_jetbot_position, _ = self._jetbot.get_world_pose()
            if np.mean(np.abs(current_jetbot_position[:2] -wheel_target4[:2])) < 0.05:
                self._task_event += 1
        elif self._task_event==6:
            current_jetbot_position, _ = self._jetbot.get_world_pose()
        return

    def post_reset(self):
        self._task_event = 0
        return


class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        # world.scene.add_default_ground_plane(name='default_ground_plane', 
        # static_friction = 0.5, 
        # dynamic_friction = 0.1) 
        world.add_task(RobotsPlaying(name="awesome_task", offset=np.array([0, 0, 0])))
        # For visualization purposes only, so we don't need to add it to the scene
        # Its there to only see where Franka was supposed to be vs after adding the offset
        #world.scene.add()
        world.scene.add(
            DynamicCuboid(
                prim_path="/World/random_cube", # The prim path of the cube in the USD stage
                name="cube1", # The unique name used to retrieve the object from the scene later on
                position=np.array([0.5, 0.5, 0.3]), # Using the current stage units which is in meters by default.
                scale=np.array([0.1, 0.1, 0.1]), # most arguments accept mainly numpy arrays.
                color=np.array([0, 1, 1.0]), # RGB channels, going from 0-
                mass=0.001
            ))
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        task_params = self._world.get_task("awesome_task").get_params()
        self._franka = self._world.scene.get_object(task_params["franka_name"]["value"])
        self._jetbot = self._world.scene.get_object(task_params["jetbot_name"]["value"])
        self._cube_name = task_params["cube_name"]["value"]
        self._franka_controller = PickPlaceController(name="pick_place_controller",
                                                    gripper_dof_indices=self._franka.gripper.dof_indices,
                                                    #gripper=self._franka.gripper,
                                                    robot_articulation=self._franka)
        self._jetbot_controller = WheelBasePoseController(name="cool_controller",
                                                        open_loop_wheel_controller=
                                                            DifferentialController(name="simple_control",
                                                                                    wheel_radius=0.03, wheel_base=0.1125))
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        await self._world.play_async()
        return

    async def setup_post_reset(self):
        self._franka_controller.reset()
        self._jetbot_controller.reset()
        await self._world.play_async()
        return

    def physics_step(self, step_size):
        current_observations = self._world.get_observations()
        if current_observations["task_event"] == 0:
            self._jetbot.apply_wheel_actions(ArticulationAction(joint_velocities=[8, 8]))
        elif current_observations["task_event"]==1:
            self._jetbot.apply_wheel_actions(ArticulationAction(joint_velocities=[-8,-8]))
            #if self._franka_controller.is_done():self._world._task_event+=1
        elif current_observations["task_event"] ==2:
            self._jetbot.apply_wheel_actions(ArticulationAction(joint_velocities=[-0.0, -0.0]))
            actions = self._franka_controller.forward(
                picking_position=current_observations[self._cube_name]["position"],
                placing_position=current_observations[self._cube_name]["target_position"],
                current_joint_positions=current_observations[self._franka.name]["joint_positions"])
            self._franka.apply_action(actions)
        elif current_observations["task_event"] ==3:
            self._jetbot.apply_wheel_actions(
                self._jetbot_controller.forward(
                    start_position=current_observations[self._jetbot.name]["position"],
                    start_orientation=current_observations[self._jetbot.name]["orientation"],
                    goal_position=current_observations[self._jetbot.name]["goal_position"]))
        elif current_observations["task_event"] ==4:
            self._jetbot.apply_wheel_actions(
                self._jetbot_controller.forward(
                    start_position=current_observations[self._jetbot.name]["position"],
                    start_orientation=current_observations[self._jetbot.name]["orientation"],
                    goal_position=current_observations[self._jetbot.name]["goal_position"]))
        elif current_observations["task_event"] ==5:
            self._jetbot.apply_wheel_actions(
                self._jetbot_controller.forward(
                    start_position=current_observations[self._jetbot.name]["position"],
                    start_orientation=current_observations[self._jetbot.name]["orientation"],
                    goal_position=current_observations[self._jetbot.name]["goal_position"]))
        elif current_observations["task_event"] == 6:
            self._jetbot.apply_wheel_actions(ArticulationAction(joint_velocities=[8, 8]))
            #self._jetbot.apply_wheel_actions(ArticulationAction(joint_velocities=[8.0, 8.0]))
        else:
            self._world.pause()
        return