import numpy as np

from robosuite.models.objects import MujocoXMLObject
from robosuite.utils.mjcf_utils import array_to_string, find_elements, xml_path_completion

class PulleyXMLObject(MujocoXMLObject):
    """
    Square plate with a hole in the center (used in PegInHole)
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/pulley.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )
    @property
    def important_sites(self):
        """
        Returns:
            dict: In addition to any default sites for this object, also provides the following entries

                :`'handle'`: Name of nut handle location site
        """
        # Get dict from super call and add to it
        dic = super().important_sites
        dic.update({"top_center": self.naming_prefix + "top_center_site"})
        return dic
    
class BeltXMLObject(MujocoXMLObject):

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/belt.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="collision",
            duplicate_collision_geoms=False,
        )
    @property
    def important_sites(self):
        """
        Returns:
            dict: In addition to any default sites for this object, also provides the following entries

                :`'handle'`: Name of nut handle location site
        """
        # Get dict from super call and add to it
        dic = super().important_sites
        dic.update({"top_center": self.naming_prefix + "top_center_site"})
        return dic
    
class TaskboardXMLObject(MujocoXMLObject):
    """
    Square plate with a hole in the center (used in PegInHole)
    """

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/taskboard.xml"),
            name=name,
            joints=None,
            obj_type="all",
            duplicate_collision_geoms=True,
        )


class MyCylinderXMLObject(MujocoXMLObject):

    def __init__(self, name):
        super().__init__(
            xml_path_completion("objects/my_cylinder.xml"),
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )
