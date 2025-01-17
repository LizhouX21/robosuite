import numpy as np

from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel
from robosuite.utils.mjcf_utils import xml_path_completion


class UR5e(ManipulatorModel):
    """
    UR5e is a sleek and elegant new robot created by Universal Robots

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    def __init__(self, idn=0):
        super().__init__(xml_path_completion("robots/ur5e/robot.xml"), idn=idn)

    @property
    def default_mount(self):
        return "RethinkMount"

    @property
    def default_gripper(self):
        return "Robotiq85Gripper"

    @property
    def default_controller_config(self):
        return "default_ur5e"

    @property
    def init_qpos(self):
        #return np.array([-0.470, -1.735, 2.480, -2.275, -1.590, -1.991])
        #return np.array([-2.470, -1.735, 2.480, -2.275, -1.590, -1.991])
        #return np.array([-0.3038833367471597, -0.32803181351392996, 2.1259880185425066, -4.939554550746945, -1.2668474985454037, -3.141605185438652])
        return np.array([-0.89907357, -1.10682245,  2.38630521, -2.82886112, -1.59070717, -2.44207431])
    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.16 - table_length / 2, 0, 0),
        }

    @property
    def top_offset(self):
        return np.array((0, 0, 1.0))

    @property
    def _horizontal_radius(self):
        return 0.5

    @property
    def arm_type(self):
        return "single"
