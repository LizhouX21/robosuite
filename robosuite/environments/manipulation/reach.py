from collections import OrderedDict

import numpy as np

from robosuite.environments.manipulation.single_arm_env import SingleArmEnv
from robosuite.models.arenas import TableArena
from robosuite.models.objects import BoxObject
from robosuite.models.tasks import ManipulationTask
from robosuite.utils.mjcf_utils import CustomMaterial
from robosuite.utils.observables import Observable, sensor
from robosuite.utils.placement_samplers import UniformRandomSampler
from robosuite.utils.transform_utils import convert_quat


class Reach(SingleArmEnv):
    """
    This class corresponds to the lifting task for a single robot arm.

    Args:
        robots (str or list of str): Specification for specific robot arm(s) to be instantiated within this env
            (e.g: "Sawyer" would generate one arm; ["Panda", "Panda", "Sawyer"] would generate three robot arms)
            Note: Must be a single single-arm robot!

        env_configuration (str): Specifies how to position the robots within the environment (default is "default").
            For most single arm environments, this argument has no impact on the robot setup.

        controller_configs (str or list of dict): If set, contains relevant controller parameters for creating a
            custom controller. Else, uses the default controller for this specific task. Should either be single
            dict if same controller is to be used for all robots or else it should be a list of the same length as
            "robots" param

        gripper_types (str or list of str): type of gripper, used to instantiate
            gripper models from gripper factory. Default is "default", which is the default grippers(s) associated
            with the robot(s) the 'robots' specification. None removes the gripper, and any other (valid) model
            overrides the default gripper. Should either be single str if same gripper type is to be used for all
            robots or else it should be a list of the same length as "robots" param

        initialization_noise (dict or list of dict): Dict containing the initialization noise parameters.
            The expected keys and corresponding value types are specified below:

            :`'magnitude'`: The scale factor of uni-variate random noise applied to each of a robot's given initial
                joint positions. Setting this value to `None` or 0.0 results in no noise being applied.
                If "gaussian" type of noise is applied then this magnitude scales the standard deviation applied,
                If "uniform" type of noise is applied then this magnitude sets the bounds of the sampling range
            :`'type'`: Type of noise to apply. Can either specify "gaussian" or "uniform"

            Should either be single dict if same noise value is to be used for all robots or else it should be a
            list of the same length as "robots" param

            :Note: Specifying "default" will automatically use the default noise settings.
                Specifying None will automatically create the required dict with "magnitude" set to 0.0.

        table_full_size (3-tuple): x, y, and z dimensions of the table.

        table_friction (3-tuple): the three mujoco friction parameters for
            the table.

        use_camera_obs (bool): if True, every observation includes rendered image(s)

        use_object_obs (bool): if True, include object (cube) information in
            the observation.

        reward_scale (None or float): Scales the normalized reward function by the amount specified.
            If None, environment reward remains unnormalized

        reward_shaping (bool): if True, use dense rewards.

        placement_initializer (ObjectPositionSampler): if provided, will
            be used to place objects on every reset, else a UniformRandomSampler
            is used by default.

        has_renderer (bool): If true, render the simulation state in
            a viewer instead of headless mode.

        has_offscreen_renderer (bool): True if using off-screen rendering

        render_camera (str): Name of camera to render if `has_renderer` is True. Setting this value to 'None'
            will result in the default angle being applied, which is useful as it can be dragged / panned by
            the user using the mouse

        render_collision_mesh (bool): True if rendering collision meshes in camera. False otherwise.

        render_visual_mesh (bool): True if rendering visual meshes in camera. False otherwise.

        render_gpu_device_id (int): corresponds to the GPU device id to use for offscreen rendering.
            Defaults to -1, in which case the device will be inferred from environment variables
            (GPUS or CUDA_VISIBLE_DEVICES).

        control_freq (float): how many control signals to receive in every second. This sets the amount of
            simulation time that passes between every action input.

        horizon (int): Every episode lasts for exactly @horizon timesteps.

        ignore_done (bool): True if never terminating the environment (ignore @horizon).

        hard_reset (bool): If True, re-loads model, sim, and render object upon a reset call, else,
            only calls sim.reset and resets all robosuite-internal variables

        camera_names (str or list of str): name of camera to be rendered. Should either be single str if
            same name is to be used for all cameras' rendering or else it should be a list of cameras to render.

            :Note: At least one camera must be specified if @use_camera_obs is True.

            :Note: To render all robots' cameras of a certain type (e.g.: "robotview" or "eye_in_hand"), use the
                convention "all-{name}" (e.g.: "all-robotview") to automatically render all camera images from each
                robot's camera list).

        camera_heights (int or list of int): height of camera frame. Should either be single int if
            same height is to be used for all cameras' frames or else it should be a list of the same length as
            "camera names" param.

        camera_widths (int or list of int): width of camera frame. Should either be single int if
            same width is to be used for all cameras' frames or else it should be a list of the same length as
            "camera names" param.

        camera_depths (bool or list of bool): True if rendering RGB-D, and RGB otherwise. Should either be single
            bool if same depth setting is to be used for all cameras or else it should be a list of the same length as
            "camera names" param.

        camera_segmentations (None or str or list of str or list of list of str): Camera segmentation(s) to use
            for each camera. Valid options are:

                `None`: no segmentation sensor used
                `'instance'`: segmentation at the class-instance level
                `'class'`: segmentation at the class level
                `'element'`: segmentation at the per-geom level

            If not None, multiple types of segmentations can be specified. A [list of str / str or None] specifies
            [multiple / a single] segmentation(s) to use for all cameras. A list of list of str specifies per-camera
            segmentation setting(s) to use.

    Raises:
        AssertionError: [Invalid number of robots specified]
    """

    def __init__(
        self,
        robots,
        env_configuration="default",
        controller_configs=None,
        gripper_types="default",
        initialization_noise={"magnitude": 0.0001, "type": "gaussian"},
        table_full_size=(0.8, 0.8, 0.05),
        table_friction=(1.0, 5e-3, 1e-4),
        use_camera_obs=True,
        use_object_obs=False,
        reward_scale=1.0,
        reward_shaping=False,
        placement_initializer=None,
        has_renderer=False,
        has_offscreen_renderer=True,
        render_camera="frontview",
        render_collision_mesh=False,
        render_visual_mesh=True,
        render_gpu_device_id=-1,
        control_freq=20,
        horizon=50,
        ignore_done=False,
        hard_reset=True,
        camera_names="agentview",
        camera_heights=256,
        camera_widths=256,
        camera_depths=False,
        camera_segmentations=None,  # {None, instance, class, element}
        renderer="mujoco",
        renderer_config=None,
        early_terminations=True
    ):
        # settings for table top
        self.table_full_size = table_full_size
        self.table_friction = table_friction
        self.table_offset = np.array((0, 0, 0.8))

        # reward configuration
        self.reward_scale = reward_scale
        self.reward_shaping = reward_shaping

        # whether to use ground-truth object states
        self.use_object_obs = use_object_obs

        # object placement initializer
        self.placement_initializer = placement_initializer

        self.early_terminations=early_terminations

        # reach target pos
        self.target_pos=np.array([0, 0, 1])

        super().__init__(
            robots=robots,
            env_configuration=env_configuration,
            controller_configs=controller_configs,
            mount_types="default",
            gripper_types=gripper_types,
            initialization_noise=initialization_noise,
            use_camera_obs=use_camera_obs,
            has_renderer=has_renderer,
            has_offscreen_renderer=has_offscreen_renderer,
            render_camera=render_camera,
            render_collision_mesh=render_collision_mesh,
            render_visual_mesh=render_visual_mesh,
            render_gpu_device_id=render_gpu_device_id,
            control_freq=control_freq,
            horizon=horizon,
            ignore_done=ignore_done,
            hard_reset=hard_reset,
            camera_names=camera_names,
            camera_heights=camera_heights,
            camera_widths=camera_widths,
            camera_depths=camera_depths,
            camera_segmentations=camera_segmentations,
            renderer=renderer,
            renderer_config=renderer_config,
        )

    def reward(self, action=None):
        """
        Reward function for the task.

        Sparse un-normalized reward:

            - a discrete reward of 2.25 is provided if the cube is lifted

        Un-normalized summed components if using reward shaping:

            - Reaching: in [0, 1], to encourage the arm to reach the cube
            - Grasping: in {0, 0.25}, non-zero if arm is grasping the cube
            - Lifting: in {0, 1}, non-zero if arm has lifted the cube

        The sparse reward only consists of the lifting component.

        Note that the final reward is normalized and scaled by
        reward_scale / 2.25 as well so that the max score is equal to reward_scale

        Args:
            action (np array): [NOT USED]

        Returns:
            float: reward value
        """
        reward = 0.0

        gripper_site_pos = self.sim.data.site_xpos[self.robots[0].eef_site_id]
        # if self._check_success():
        #     a=1
        # sparse completion reward
        if not self.reward_shaping:
            if self._check_success():
                reward = 0
            # elif self._below_table():
            #     reward=-500
            else:
                reward=-1

        # # use a shaping reward
        # elif self.reward_shaping:

        #     # reaching reward
        #     cube_pos = self.sim.data.body_xpos[self.cube_body_id]
        #     gripper_site_pos = self.sim.data.site_xpos[self.robots[0].eef_site_id]
        #     dist = np.linalg.norm(gripper_site_pos - cube_pos)
        #     reaching_reward = 1 - np.tanh(10.0 * dist)
        #     reward += reaching_reward

        #     # grasping reward
        #     if self._check_grasp(gripper=self.robots[0].gripper, object_geoms=self.cube):
        #         reward += 0.25

        # # Scale reward if requested
        # if self.reward_scale is not None:
        #     reward *= self.reward_scale / 2.25
        # #print(reward)

        if self.reward_shaping:
            dist = np.linalg.norm(gripper_site_pos - self.target_pos)
            #reward = 1 - np.tanh(10.0 * dist)   
            reward=-dist    
        return reward
    



        # my rewards
        # reward = 0.0

        # sparse completion reward
        # if self._check_success():
        #     reward = 2.25

        # use a shaping reward
        # elif self.reward_shaping:

        #     reaching reward
        #     cube_pos = self.sim.data.body_xpos[self.cube_body_id]
        #     gripper_site_pos = self.sim.data.site_xpos[self.robots[0].eef_site_id]
        #     dist = np.linalg.norm(gripper_site_pos - cube_pos)
        #     reaching_reward = 1 - np.tanh(10.0 * dist)
        #     reward += reaching_reward

        #     grasping reward
        #     if self._check_grasp(gripper=self.robots[0].gripper, object_geoms=self.cube):
        #         reward += 0.25

        # Scale reward if requested
        # if self.reward_scale is not None:
        #     reward *= self.reward_scale / 2.25
        # print(reward)
        # return reward
        # my rewards



    def _load_model(self):
        """
        Loads an xml model, puts it in self.model
        """
        #print("episode begin")
        super()._load_model()

        # Adjust base pose accordingly
        xpos = self.robots[0].robot_model.base_xpos_offset["table"](self.table_full_size[0])
        self.robots[0].robot_model.set_base_xpos(xpos)

        # load model for table top workspace
        mujoco_arena = TableArena(
            table_full_size=self.table_full_size,
            table_friction=self.table_friction,
            table_offset=self.table_offset,
        )

        # Arena always gets set to zero origin
        mujoco_arena.set_origin([0, 0, 0])

        # initialize objects of interest
        tex_attrib = {
            "type": "cube",
        }
        mat_attrib = {
            "texrepeat": "1 1",
            "specular": "0.4",
            "shininess": "0.1",
        }
        redwood = CustomMaterial(
            texture="WoodRed",
            tex_name="redwood",
            mat_name="redwood_mat",
            tex_attrib=tex_attrib,
            mat_attrib=mat_attrib,
        )

        # task includes arena, robot, and objects of interest
        self.model = ManipulationTask(
            mujoco_arena=mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
        )

    def _setup_references(self):
        """
        Sets up references to important components. A reference is typically an
        index or a list of indices that point to the corresponding elements
        in a flatten array, which is how MuJoCo stores physical simulation data.
        """
        super()._setup_references()


    def _setup_observables(self):
        """
        Sets up observables to be used for this environment. Creates object-based observables if enabled

        Returns:
            OrderedDict: Dictionary mapping observable names to its corresponding Observable object
        """
        observables = super()._setup_observables()
            # cube-related observables
        modality = "object"

        # cube-related observables
        @sensor(modality=modality)
        def target_pos(obs_cache):
            return self.target_pos
        
        @sensor(modality=modality)
        def current_ee_vel(obs_cache):
            return self.robots[0].recent_ee_vel.current[0:3]
        # @sensor(modality=modality)
        # def cube_quat(obs_cache):
        #     return convert_quat(np.array(self.sim.data.body_xquat[self.cube_body_id]), to="xyzw")

        # @sensor(modality=modality)
        # def gripper_to_cube_pos(obs_cache):
        #     return (
        #         obs_cache[f"{pf}eef_pos"] - obs_cache["cube_pos"]
        #         if f"{pf}eef_pos" in obs_cache and "cube_pos" in obs_cache
        #         else np.zeros(3)
        #     )

        sensors = [target_pos,current_ee_vel]
        names = [s.__name__ for s in sensors]

        # Create observables
        for name, s in zip(names, sensors):
            observables[name] = Observable(
                name=name,
                sensor=s,
                sampling_rate=self.control_freq,
            ) 

        return observables

    def _reset_internal(self):
        """
        Resets simulation internal configurations.
        """
        self.target_pos=self._sample_target_goal()
        super()._reset_internal()

    def visualize(self, vis_settings):
        """
        In addition to super call, visualize gripper site proportional to the distance to the cube.

        Args:
            vis_settings (dict): Visualization keywords mapped to T/F, determining whether that specific
                component should be visualized. Should have "grippers" keyword as well as any other relevant
                options specified.
        """
        # Run superclass method first
        super().visualize(vis_settings=vis_settings)

        # Color the gripper visualization site according to its distance to the cube
        if vis_settings["grippers"]:
            self._visualize_gripper_to_target(gripper=self.robots[0].gripper, target=self.cube)

    def _check_success(self):
        """
        Check if cube has been lifted.

        Returns:
            bool: True if cube has been lifted
        """
        # cube_height = self.sim.data.body_xpos[self.cube_body_id][2]
        # table_height = self.model.mujoco_arena.table_offset[2]
        # np.linalg.norm()
        # # cube is higher than the table top above a margin
        # return cube_height > table_height + 0.04
        return self._reached_pos()

    def _below_table(self):
        if self._eef_xpos[2]<=self.table_offset[2]:
            #print("Below Table!Early Stop!")
            return True
        else:
            return False
        
    def _reached_pos(self):
        gripper_site_pos = self.sim.data.site_xpos[self.robots[0].eef_site_id]
        dist=np.linalg.norm(gripper_site_pos - self.target_pos)
        if dist<0.05:
            return True
        else:
            return False
        
        
    def _post_action(self, action):
        """
        In addition to super method, add additional info if requested
        Args:
            action (np.array): Action to execute within the environment
        Returns:
            3-tuple:
                - (float) reward from the environment
                - (bool) whether the current episode is completed or not
                - (dict) info about current env step
        """
        reward, done, info = super()._post_action(action)

        # allow episode to finish early if allowed
        if self.early_terminations:
            done = done or self._check_terminated()


        info["is_success"]=self._check_success()
        return reward, done, info

    def _check_terminated(self):
        """
        Check if the task has completed one way or another. The following conditions lead to termination:
            - Collision
            - Task completion (insertion succeeded)
            - Joint Limit reached
        Returns:
            bool: True if episode is terminated
        """

        terminated = False

        # # Prematurely terminate if contacting the table with the arm
        # if self.check_contact(self.robots[0].robot_model):
        #     #if self.print_results:
        #         #print(40 * "-" + " COLLIDED " + 40 * "-")
        #     terminated = True

        # # Prematurely terminate if task is success
        # if self._check_success():
        #     if self.print_results:
        #         print(40 * "+" + " FINISHED WIPING " + 40 * "+")
        #     terminated = True

        # Prematurely terminate if over q limit
        # if self.robots[0].check_q_limits():
        #     terminated = True

        # Prematurely terminate below table
        if self._below_table():
            terminated = True
        if self._reached_pos():
            terminated=True

        return terminated

    def _sample_target_goal(self):
        plane_xy=np.random.uniform(low=-0.2, high=0.2, size=(2,))
        height_z=np.random.uniform(low=0.92, high=1.32, size=(1,))
        target=np.concatenate((plane_xy,height_z))
        # target=np.array([0,0,1.2])#fixed target
        return target