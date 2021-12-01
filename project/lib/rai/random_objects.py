import random
import sys

import numpy as np

from config.CONFIG import *
from lib.prediction import color_seg
from lib.rai.rai_helper import set_frame_properties

# parent script need to append path to (package_name)/scripts

# append rai lib path
sys.path.append(RAI_PATH)
import libry as ry


###### finished import here ###########


def get_random_rgb_val():
    return np.append(np.random.randint(0, 255, 3, np.uint8), np.array([255]))  # last number is rgbA value transparency


def vec_dist(x: np.array, y: np.array):
    """
    Args:
        x: numpy array
        y: numpy array
    Returns: Distance between x and y
    """
    return np.linalg.norm(x - y)


def get_random_unique_colors(number_colors: int, background_colors=[], hue_tolerance=3):
    """
    Returns a list of different colors, which are determined by interval steps in the colorspace.
    Args:
        background_colors (list): background colors to avoid
        hue_tolerance (int): min distance of bgr colors in hue space
        number_colors (int): number of colors to return
    """
    found_colors = False
    while not found_colors:
        colors = []
        found_colors = True
        init_color = get_random_rgb_val()
        hue = color_seg.single_rgb2hsv(init_color)[0]

        for idx in range(number_colors):
            hue += (180 / number_colors)
            if hue > 180: hue -= 180

            # check if colors are not to close to background colors
            for bgr_color in background_colors:
                bgr_hue = color_seg.single_rgb2hsv(bgr_color)[0]
                if abs(bgr_hue - hue) <= hue_tolerance:
                    found_colors = False
                    print("Hue to close to background. Trying again.", abs(bgr_hue - hue))
                    break
            # saturation and value
            sat = random.randint(120, 180)
            val = random.randint(120, 180)

            hsv_color = np.array([hue, sat, val], np.uint8)
            rgb_color = color_seg.single_hsv2rgb(hsv_color)
            colors.append(rgb_color)
    return colors


def random_positions(number_positions: int, table_frame: ry.Frame, min_distance: float = 0.):
    """
    Returns a list of different positions. And makes sure the positions are not to close.
    Args:
        table_frame: frame of table where the objects should be placed
        min_distance (float): min distance of positions
        number_positions (int): number of colors to return
    """
    positions = []
    while number_positions > len(positions):
        position = random_position_on_table(table_frame)
        append_position = True
        for pos in positions:
            if vec_dist(pos, position) <= min_distance:
                print("to close")
                append_position = False

        if append_position:
            # print("accepted")
            positions.append(position)

    return positions


def random_position_on_table(table_frame: ry.Frame):
    size = table_frame.getSize()
    pose = table_frame.getPosition()
    # calc pose on table
    x = random.uniform(pose[0] - size[0] * .37, pose[0] + size[0] * .37)  # about 40% of size around the pose
    y = random.uniform(pose[1] - size[1] * .37, pose[1] + size[1] * .37)
    z = pose[2] + size[2] + 0.1
    return np.array([x, y, z])


def get_random_quaternion(prob_no_rotation=.05):
    if random.random() < prob_no_rotation:
        return np.array([1, 0, 0, 0])
    else:
        return np.random.rand(4)


def get_random_size_of_shape(shape):
    if shape is ry.ST.ssBox:
        # box size
        edge_len = random.uniform(.04, .16)
        size = [edge_len] * 3 + [edge_len * .06]  # with 6% round edges
    elif shape is ry.ST.sphere:
        # sphere size
        size = [random.uniform(.02, .08)]
    elif shape is ry.ST.cylinder:
        # sphere cylinder size
        size = [random.uniform(.06, .18), random.uniform(.01, .04)]
    else:
        print("shape not yet defined in generator")
        sys.exit(1)
    return size


def create_random_rai_objs(Rai, max_number_objs: int, shapes, random_object_count=True):
    objs = []

    if random_object_count:
        number_objs = random.randint(1, max_number_objs)
    else:
        number_objs = max_number_objs

    # background colors
    baxter_sensor = np.array([0.85, 0.2, 0.2]) * 255
    baxter_arm = np.array([0.8, 0.2, 0.2]) * 255
    baxter_gripper = np.array([85, 85, 85])
    floor = np.array([111, 125, 139])

    background_colors = [floor, baxter_arm, baxter_gripper, baxter_sensor]

    colors = get_random_unique_colors(number_objs + 1, background_colors)

    # random table color
    table = Rai.RealWorld.getFrame("table")
    set_frame_properties(table, color=colors[-1])

    for idx in range(number_objs):
        name = "obj{}".format(idx)
        # choose from shapes list
        shape = random.choice(shapes)
        # shapes
        size = get_random_size_of_shape(shape)

        color = colors[idx]

        position = random_position_on_table(Rai.RealWorld.getFrame("table"))
        quaternion = get_random_quaternion()
        frame = Rai.add_object(name, shape, size, color, position, quaternion, mass=0.2)
        # store object properties in dict
        objs.append({"id": idx, "name": name, "shape": shape.name, "size": size, "color": color, "position": position,
                     "quaternion": quaternion})

    # print(str(objs))
    return objs


def random_obj_frames(Rai, frames: list, classes: list, max_objects: int, random_object_count=True):
    assert (len(frames) >= max_objects)
    assert (len(frames) == len(classes))

    metadata = []

    if random_object_count:
        # randomly choose how many frames to keep
        number_objs = random.randint(0, max_objects)
    else:
        # use every frame
        number_objs = max_objects
    selected_idx = random.sample(range(len(frames)), number_objs)
    selected_frames = [frames[i] for i in selected_idx]
    selected_classes = [classes[i] for i in selected_idx]

    # background colors
    baxter_sensor = np.array([0.85, 0.2, 0.2]) * 255
    baxter_arm = np.array([0.8, 0.2, 0.2]) * 255
    baxter_gripper = np.array([85, 85, 85])
    floor = np.array([111, 125, 139])

    background_colors = [floor, baxter_arm, baxter_gripper, baxter_sensor]

    colors = get_random_unique_colors(number_objs + 1, background_colors)

    # random table color
    table = Rai.RealWorld.getFrame("table")
    set_frame_properties(table, color=colors[-1])

    positions = random_positions(number_objs, Rai.RealWorld.getFrame("table"), min_distance=0.1)

    for idx, frame in enumerate(selected_frames):
        color = colors[idx]

        position = positions[idx]
        quaternion = get_random_quaternion()

        rai_frame = Rai.RealWorld.getFrame(frame)

        set_frame_properties(rai_frame, color=color, position=position, quaternion=quaternion)

        # store object properties in dict
        metadata.append(
            {"id": idx, "name": frame, "class": selected_classes[idx], "color": color, "position": position,
             "quaternion": quaternion})

    del_frames = [frame for frame in frames if frame not in selected_frames]

    for frame in del_frames:
        Rai.RealWorld.delFrame(frame)

    return metadata


def add_dataset_objects(Rai, frames, classes, max_objects, rai_primitives=False, random_object_count=True):
    # choose if dataset uses rai primitive objects or frames from g-file
    if rai_primitives:
        rai_objs = [getattr(ry.ST, name) for name in frames]
        objects_metadata = create_random_rai_objs(Rai, max_objects, rai_objs, random_object_count)
    else:  # else it uses frames form a g-file
        # randomize frames in list
        if classes is None: classes = frames
        objects_metadata = random_obj_frames(Rai, frames, classes, max_objects, random_object_count)

    return frames, objects_metadata
