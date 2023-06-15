import numpy as np
from robosuite.environments.manipulation.single_arm_env import SingleArmEnv
from robosuite.models.arenas import *
from robosuite.models.objects.my_xml_objects import PulleyXMLObject
from robosuite.models.tasks import ManipulationTask
from robosuite.utils.observables import Observable, sensor
from robosuite.utils.placement_samplers import UniformRandomSampler
#from robosuite.utils.binding_utils import *
from robosuite.utils.transform_utils import *
from math import pi
import robosuite.utils.transform_utils as tf_utils

class Insertion(SingleArmEnv):
    def __init__(
        self,
        robots,
        env_configuration="default",
        controller_configs=None,
        gripper_types="Robotiq85GripperVshape",
        initialization_noise={"magnitude": 0.0001, "type": "gaussian"},
        table_full_size=(0.8, 0.8, 0.05),
        table_friction=(1,5e-3, 1e-4),
        use_camera_obs=False,
        use_object_obs=True,
        reward_scale=1.0,
        reward_shaping=True,
        placement_initializer=None,
        has_renderer=False,
        has_offscreen_renderer=True,
        render_camera="insertion_view",
        render_collision_mesh=False,
        render_visual_mesh=True,
        render_gpu_device_id=-1,
        control_freq=20,
        horizon=500,
        ignore_done=False,
        hard_reset=True,
        camera_names="agentview",
        camera_heights=256,
        camera_widths=256,
        camera_depths=False,
        camera_segmentations=None,  # {None, instance, class, element}
        renderer="mujoco",
        renderer_config=None,
    ):
        camer_view_list=('frontview', 'birdview', 'agentview', 'sideview', 'robot0_robotview', 'robot0_eye_in_hand')
        # task settings
        self.obj_to_use = None

        # settings for table top
        self.table_full_size = table_full_size
        self.table_friction = table_friction
        self.table_offset = np.array((0, 0, 0.80))

        # reward configuration
        self.reward_scale = reward_scale
        self.reward_shaping = reward_shaping

        # whether to use ground-truth object states
        self.use_object_obs = use_object_obs

        # object placement initializer
        self.placement_initializer = placement_initializer

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
        self.gripper_pos = self.sim.data.get_site_xpos(self.robots[0].gripper.important_sites["grip_site"])
        self.gripper_mat=self.sim.data.get_site_xmat(self.robots[0].gripper.important_sites["grip_site"])
        self.taskboard_origin_pos = self.sim.data.site_xpos[self.taskboard_origin_site_id]
        self.cylinder_insertion_center_pos = self.sim.data.site_xpos[self.taskboard_cylinder_insertion_center_id]
        self.cylinder_insertion_center_ori=self.sim.data.site_xmat[self.taskboard_cylinder_insertion_center_id]


        #print(taskboard_origin_pos)
        #print(cylinder_insertion_center_pos)
        #print(cylinder_insertion_center_ori)
        # print("gripper pos")
        # print(gripper_pos)
        # print("gripper mat")
        # print(gripper_mat)
        return 1

    def in_hole(self, obj_pos, peg_id):

        return True

    def _load_model(self):
        """
        Loads an xml model, puts it in self.model
        """
        super()._load_model()

        # Adjust base pose accordingly
        xpos = self.robots[0].robot_model.base_xpos_offset["table"](self.table_full_size[0])
        my_xpos=(-0.9,0,0)
        self.robots[0].robot_model.set_base_xpos(my_xpos)

        # load model for table top workspace
        mujoco_arena = TableArenaWithTaskboard(
            table_full_size=self.table_full_size,
            table_friction=self.table_friction,
            table_offset=self.table_offset,
        )

        # Arena always gets set to zero origin
        mujoco_arena.set_origin([0, 0, 0])


        # self.cylinder = CylinderObject(
        #     name="cylinder",
        #     size=(0.019,0.015),
        #     rgba=[1, 0, 0, 1]
        # )
        # self.cylinder.density=5000
        # self.pot=PotWithHandlesObject(name="pot")
        # self.peg = CylinderObject(
        #     name="peg",
        #     size=(0.015,0.05),
        #     rgba=[0, 1, 0, 1],
        #     joints=None,
        # )

        #self.my_cylinder=MyCylinderXMLObject(name="my_cylinder")

        self.part=PulleyXMLObject("pulley")


        # for save the xml for hollow cylinder
        # self.hollow_cylinder=HollowCylinderObject(name="hollow_cylinder",outer_radius=0.03,inner_radius=0.018017, height=0.0025, ngeoms=256)
        # hollow_cylinder_obj=self.hollow_cylinder.get_obj()
        # hollow_cylinder_obj.set("pos", array_to_string((0, 0, 1.5)))
        # mujoco_arena.worldbody.append(hollow_cylinder_obj)



        #self.taskboard=TaskboardXMLObject("taskboard")
        # taskboard_obj=self.taskboard.get_obj()
        # taskboard_obj.set("pos", array_to_string((0, 0, 0.4)))
        # taskboard_obj_set_mat=euler2mat((0,pi,pi))
        # taskboard_obj_set_quat=mat2quat(taskboard_obj_set_mat)
        # taskboard_obj.set("quat", taskboard_obj_set_quat)

        # peg_obj = self.peg.get_obj()
        # peg_obj.set("pos", array_to_string((0, 0, 1.2)))
        # pot_obj=self.pot.get_obj()
        # pot_obj.set("pos", array_to_string((0, 0, 0.3)))


        robot_eef=self.robots[0].robot_model.eef_name
        robot_model=self.robots[0].robot_model

        #r_body = find_elements(root=robot_model.worldbody, tags="body", attribs={"name": robot_eef}, return_first=True)
        #r_body.append(pot_obj)
        #my_box = mujoco_arena.worldbody.append(hollow_cylinder_obj)
        #peg_obj.append(hollow_cylinder_obj)
        # mujoco_arena.worldbody.append(peg_obj)



        if self.placement_initializer is not None:
            self.placement_initializer.reset()
            self.placement_initializer.add_objects(self.part)
        else:
            self.placement_initializer = UniformRandomSampler(
                name="ObjectSampler",
                mujoco_objects=self.part,
                x_range=[-0.000, 0.000],
                y_range=[-0.000, 0.000],
                # rotation=-pi/2,
                # rotation_axis='y',
                rotation=-pi/2,
                rotation_axis='z',
                ensure_object_boundary_in_range=False,
                ensure_valid_placement=True,
                reference_pos=np.array((-0.3, -0.3, 0.80)),#pulley placing position
                # z_offset=0.05,
                z_offset=0.03,                
            )

        self.model = ManipulationTask(
            mujoco_arena=mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=self.part
        )
        #pulley_insertion_center = find_elements(root=self.model.worldbody, tags="body", attribs={"name": "taskboard"}, return_first=True)
        #pulley_insertion_center.append(hollow_cylinder_obj)
        

        #self.model.merge_assets(self.hammer)
        #self.model.merge_assets(self.pulley)
        #self.model.merge_assets(self.pot)
        #self.model.merge_assets(self.peg)
        #self.model.merge_assets(self.hollow_cylinder)
        

    def _setup_references(self):
        super()._setup_references()

        # Additional object references from this env
        #self.obj_body_id = {}
        self.object_body_ids = dict()
        self.obj_geom_id = {}

        self.table_body_id = self.sim.model.body_name2id("table")
        self.taskboard_body_id=self.sim.model.body_name2id("taskboard")
        self.taskboard_origin_site_id=self.sim.model.site_name2id("taskboard_origin")
        self.taskboard_cylinder_insertion_center_id=self.sim.model.site_name2id("pulley_insertion_center")
        self.part_body_id=self.sim.model.body_name2id("pulley_main")
        # #self.cylinder_top_site_id  = self.sim.model.site_name2id(self.cylinder.important_sites["top"])

        self.part_insertion_center_pos = self.sim.data.site_xpos[self.taskboard_cylinder_insertion_center_id]
        self.part_insertion_center_ori=self.sim.data.site_xmat[self.taskboard_cylinder_insertion_center_id]
        self.part_center_pos=self.sim.data.body_xpos[self.part_body_id]


    def _setup_observables(self):
        """
        Sets up observables to be used for this environment. Creates object-based observables if enabled
        Returns:
            OrderedDict: Dictionary mapping observable names to its corresponding Observable object
        """
        observables = super()._setup_observables()

        # low-level object information
        if self.use_object_obs:
            # Get robot prefix and define observables modality
            pf = self.robots[0].robot_model.naming_prefix
            modality = "object"

            # part-related observables
            @sensor(modality=modality)
            def part_pos(obs_cache):
                return np.array(self.sim.data.body_xpos[self.part_body_id])

            @sensor(modality=modality)
            def part_quat(obs_cache):
                return convert_quat(np.array(self.sim.data.body_xquat[self.part_body_id]), to="xyzw")

            @sensor(modality=modality)
            def gripper_to_part_pos(obs_cache):
                return (
                    obs_cache[f"{pf}eef_pos"] - obs_cache["part_pos"]
                    if f"{pf}eef_pos" in obs_cache and "part_pos" in obs_cache
                    else np.zeros(3)
                )

            sensors = [part_pos, part_quat, gripper_to_part_pos]
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
        super()._reset_internal()

        if not self.deterministic_reset:

            # Sample from the placement initializer for all objects
            object_placements = self.placement_initializer.sample()

            # Loop through all objects and reset their positions
            for obj_pos, obj_quat, obj in object_placements.values():
                    self.sim.data.set_joint_qpos(obj.joints[0], np.concatenate([np.array(obj_pos), np.array(obj_quat)]))

        self._initiliaze_scene()

    def _check_success(self):
        return True

    def visualize(self, vis_settings):
        # Run superclass method first
        super().visualize(vis_settings=vis_settings)

        # Color the gripper visualization site according to its distance to the closest nut
        if vis_settings["grippers"]:
            # find closest object
            dists = [
                self._gripper_to_target(
                    gripper=self.robots[0].gripper,
                    target=nut.important_sites["handle"],
                    target_type="site",
                    return_distance=True,
                )
                for nut in self.nuts
            ]
            closest_nut_id = np.argmin(dists)
            # Visualize the distance to this target
            self._visualize_gripper_to_target(
                gripper=self.robots[0].gripper,
                target=self.nuts[closest_nut_id].important_sites["handle"],
                target_type="site",
            )

    def _keep_part_stationary(self,x,y,z,w,xa,ya,za):#w is the last one
        self.sim.data.qvel[12] = 0
        self.sim.data.qvel[13] = 0
        self.sim.data.qvel[14] = 0
        self.sim.data.qvel[15] = 0
        self.sim.data.qvel[16] = 0
        self.sim.data.qvel[17] = 0

        self.sim.data.qpos[12] = x
        self.sim.data.qpos[13] = y
        self.sim.data.qpos[14] = z
        self.sim.data.qpos[15] = xa
        self.sim.data.qpos[16] = ya
        self.sim.data.qpos[17] = za
        self.sim.data.qpos[18] = w
        self.sim.forward()

    def _initiliaze_scene(self):
        phase=0
        closing_timesteps=12
        j=0
        i_action=np.array([0,0,0,0,0,0,0])
        # for i in range(1000):
        #     current_ee_mat=self.sim.data.site_xmat[self.robots[0].eef_site_id].reshape(3,3)
        #     current_ee_position_controller=self.robots[0].controller.ee_pos
        #     current_ee_position=self.sim.data.site_xpos[self.robots[0].eef_site_id]
        #     part_pos=self.sim.data.body_xpos[self.part_body_id]
        #     if phase==0:#initialization the pulley
        #         self._keep_part_stationary(current_ee_position[0]+0.003,current_ee_position[1],current_ee_position[2], 0.7071068, 0, 0, 0.7071068)
        #         phase=1
        #     if phase==1:
        #         i_action[-1]=1
        #         self._keep_part_stationary(current_ee_position[0]+0.003,current_ee_position[1],current_ee_position[2], 0.7071068, 0, 0, 0.7071068)
        #         if self.check_contact(self.robots[0].gripper,self.part):
        #             phase=2
        #             i_action[-1]=1
        #     if phase==2:#close a bit more to ensure firmly contact
        #             self._keep_part_stationary(current_ee_position[0]+0.003,current_ee_position[1],current_ee_position[2], 0.7071068, 0, 0, 0.7071068)
        #             i_action[-1]=1
        #             j=j+1
        #             if j==closing_timesteps:
        #                 phase=3
        #     if phase==3:
        #         i_action[-1]=0
        #     self._initialize_step(i_action)
            

    def _initialize_step(self,action):
            self.sim.forward()
            self._pre_action(action,policy_step=True)
            self.sim.step()
            self._update_observables()