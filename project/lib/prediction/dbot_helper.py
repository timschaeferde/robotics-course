import copy

import numpy as np

from prediction.dbot import gaussian_tracker_service_call, object_tracker_service_call


def call_dbot(obj_metadata, particle_tracker=True, gaussian_tracker=True):
    data_list = copy.deepcopy(obj_metadata)
    meshes = []
    rel_positions = []
    rel_quaternions = []
    names = []
    for data in data_list:
        names.append(data["name"])
        meshes.append(data["mesh_name"])
        rel_positions.append(position_rai2dbot(data["rel_mesh_pose"][:3]))
        rel_quaternions.append(quaternion_rai2dbot(data["rel_mesh_pose"][3:]))
    # call tracker services
    if particle_tracker:
        object_tracker_service_call.track_object(names, meshes, rel_positions, rel_quaternions)
    if gaussian_tracker:
        gaussian_tracker_service_call.track_object(names, meshes, rel_positions, rel_quaternions)


def pose_rai2dbot(pose):
    return np.append(position_rai2dbot(pose[:3]), -quaternion_rai2dbot(pose[3:]))


def position_rai2dbot(position):
    """
        reverse coordinates y and z for rai for dbot compatibility.
        Also works the other way: dbot->rai
    Args:
        position: np 3d array

    Returns: np 3d array
    """
    position[1:3] = - position[1:3]
    return position


def quaternion_rai2dbot(quaternion):
    """
        swaps and inverts coordinates y and w for rai for dbot compatibility.
        Also works the other way: dbot->rai
    Args:
        position: np 4d array

    Returns: np 4d array
    """
    return np.array([quaternion[0], -quaternion[3], quaternion[2], -quaternion[1]])
