import json
import os
import pickle
import shutil
from datetime import datetime

import cv2 as cv
import numpy as np
from pyquaternion import Quaternion
from sklearn.metrics import euclidean_distances

from lib.prediction import color_seg


def get_gt(pred_metadata, real_metadata):
    for real in real_metadata:
        position = real["position"]

        predicted_list = [predicted for predicted in pred_metadata if predicted["class"] == real["class"]]
        if len(predicted_list) < 1: continue
        # determine closest predicted point
        dist_list = np.asarray([np.linalg.norm(data["mesh_pose"][:3] - position) for data in predicted_list])
        id_pred = dist_list.argmin()
        predicted = predicted_list[id_pred]

        # avoid overwriting better existing ground truth (always keep closest ground truth)
        if predicted["gt_pose"] is not None:
            old_distance = np.linalg.norm(predicted["mesh_pose"][:3] - predicted["gt_pose"][:3])
            if old_distance < dist_list[id_pred]: continue
        # set ground truth for closest object
        predicted["gt_pose"] = np.append(position, Quaternion(real["quaternion"]).normalised.elements)

    return pred_metadata


def normalize_img(img):
    return (img / img.max() * 255).astype("uint8")


#########################################
# file i/o tools                        #
#########################################
def save_img2file(cv_img, filename="test_img.png"):
    cv.imwrite(filename, cv_img)
    # print("File saved: ", filename)


def load_imgfile(filename="test_img.png", gray=False):
    if gray: return cv.imread(filename, cv.IMREAD_GRAYSCALE)
    return cv.imread(filename)


def check_and_create_dir(path: str):
    # create directory if not exists
    if not os.path.exists(path):
        os.makedirs(path)


def delete_folder(path: str):
    try:
        shutil.rmtree(path)
    except Exception as error:
        print("folder not found")
        print(error)


def dump_data(data_path, stamp, pred_metadata=None, real_metadata=None, mrcnn_results=None, rai_state=None, img=None):
    print("dump data to: {}".format(os.path.abspath(data_path)))
    check_and_create_dir(data_path)

    # dump mrcnn results
    if mrcnn_results is not None:
        mrcnn_path = os.path.join(data_path, "mrcnn")
        check_and_create_dir(mrcnn_path)
        pickle_dump(mrcnn_results, os.path.join(mrcnn_path, "{}_results.pkl".format(stamp)))
    # dump object data
    if real_metadata is not None:
        pose_data_path = os.path.join(data_path, "pose_data")
        check_and_create_dir(pose_data_path)
        # pickle_dump(real_metadata, os.path.join(pose_data_path, "{}_real.pkl".format(stamp)))
        json_dump(real_metadata, os.path.join(pose_data_path, "{}_real.json".format(stamp)))
    if pred_metadata is not None:
        pose_data_path = os.path.join(data_path, "pose_data")
        check_and_create_dir(pose_data_path)
        # pickle_dump(pred_metadata, os.path.join(pose_data_path, "{}_pred.pkl".format(stamp)))
        json_dump(pred_metadata, os.path.join(pose_data_path, "{}_pred.json".format(stamp)))
    # dump simulation state
    if rai_state is not None:
        rai_states_path = os.path.join(data_path, "rai_states")
        check_and_create_dir(rai_states_path)
        pickle_dump(rai_state, os.path.join(rai_states_path, "{}_sim_state.pkl".format(stamp)))
    # dump images and pointcloud
    if img is not None:
        img_path = os.path.join(data_path, "images")
        check_and_create_dir(img_path)
        save_img2file(color_seg.bgr2rgb(img['rgb']), os.path.join(img_path, "{}_rgb.png".format(stamp)))
        pickle_dump(img["depth"], os.path.join(img_path, "{}_depth.pkl".format(stamp)))


def load_data(data_path, stamp=None):
    # print(data_path, stamp)
    if stamp is None:
        # take last file
        stamp = sorted(os.listdir(data_path + "/pose_data"))[-1].split("_")[0]
        print("loading last file with timestamp: {}".format(stamp))

    # create time stamp
    mrcnn_path = os.path.join(data_path, "mrcnn")
    pose_data_path = os.path.join(data_path, "pose_data")
    rai_states_path = os.path.join(data_path, "rai_states")
    img_path = os.path.join(data_path, "images")

    # dump mrcnn results
    mrcnn_results = pickle_load(os.path.join(mrcnn_path, "{}_results.pkl".format(stamp)))
    # dump object data
    real_metadata = json_load(os.path.join(pose_data_path, "{}_real.json".format(stamp)))
    pred_metadata = json_load(os.path.join(pose_data_path, "{}_pred.json".format(stamp)))
    # dump simulation state
    rai_state = pickle_load(os.path.join(rai_states_path, "{}_sim_state.pkl".format(stamp)))

    if os.path.exists(os.path.join(img_path, "{}_rgb.png".format(stamp))):
        rgb = load_imgfile(os.path.join(img_path, "{}_rgb.png".format(stamp)))
        depth = pickle_load(os.path.join(img_path, "{}_depth.pkl".format(stamp)))
        img = {"rgb": rgb, "depth": depth}
    else:
        img = None

    return pred_metadata, real_metadata, mrcnn_results, rai_state, img


#########################################
# pickle tools                            #
#########################################

def pickle_load(path):
    if os.path.exists(path):
        return pickle.load(open(path, "rb"))
    else:
        return None


def pickle_dump(data, path):
    pickle.dump(data, open(path, "wb"))


#########################################
# json tools                            #
#########################################
def json2numpy(dictionary: dict):
    for data in dictionary:
        for key in data.keys():
            if type(data[key]) == list:
                data[key] = np.asarray(data[key])
    return dictionary


class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


def json_dump(data, path):
    json.dump(data, open(path, "w"), cls=NumpyEncoder)


def json_load(path):
    if os.path.exists(path):
        return json.load(open(path, "r"))
    else:
        return None


def position_loss(position, gt_position):
    """
    Classic euclidean distance.
    Args:
        position: predicted position
        gt_position: ground truth position

    Returns: euclidean distance

    """
    return np.linalg.norm(gt_position - position)


def sloss(quaternion, gt_quaternion, points: np.array):
    """
    Implementation of SLOSS form Xiang et al.  https://arxiv.org/abs/1711.00199
    Args:
        points: number of sampled points on mesh
        quaternion: predicted quaternion
        gt_quaternion: ground truth quaternion

    Returns: SLOSS

    """
    assert (points is not None)

    rot = Quaternion(quaternion).rotation_matrix
    gt_rot = Quaternion(gt_quaternion).rotation_matrix

    distances = euclidean_distances(points @ rot.T, points @ gt_rot.T)
    loss = np.sum(np.min(distances, axis=1)) / len(points)

    return loss


def quaternion_loss(quaternion, gt_quaternion, points=None):
    return sloss(quaternion, gt_quaternion, points)


def add_loss(pose, gt_pose, points=None):
    """
    Implementation of ADD-S loss form Xiang et al.  https://arxiv.org/abs/1711.00199
    Args:
        points: number of sampled points on mesh
        pose: predicted pose
        gt_pose: ground truth pose

    Returns: SLOSS

    """
    assert (points is not None)

    rot = Quaternion(pose[3:]).rotation_matrix
    gt_rot = Quaternion(gt_pose[3:]).rotation_matrix

    distances = euclidean_distances(points @ rot.T + pose[:3], points @ gt_rot.T + gt_pose[:3])
    loss = np.sum(np.min(distances, axis=1)) / len(points)

    return loss


def create_pointcloud(depth, fxfypxpy):
    pointcloud = np.empty((depth.shape[0], depth.shape[1], 3))
    for y in range(depth.shape[0]):
        for x in range(depth.shape[1]):
            d = depth[y, x]

            px = fxfypxpy[-2]
            py = fxfypxpy[-1]

            pointcloud[y, x, 0] = d * (x - px) / fxfypxpy[0]
            pointcloud[y, x, 1] = -d * (y - py) / fxfypxpy[1]
            pointcloud[y, x, 2] = -d
    return pointcloud


def add_visual_artifacts(img, fxfypxpy, max_depth_blur=0.4, max_depth_noise=0.007):
    # Adding noise...
    import imgaug.augmenters as iaa

    color_seq = iaa.Sequential([
        # Small gaussian blur with random sigma between 0 and 0.5.
        # But we only blur about 50% of all images.
        iaa.Sometimes(
            0.8,
            iaa.GaussianBlur(sigma=(0, 0.5))
        ),
        # Add gaussian noise.
        # For 50% of all images, we sample the noise once per pixel.
        # For the other 50% of all images, we sample the noise per pixel AND
        # channel. This can change the color (not only brightness) of the
        # pixels.
        iaa.AdditiveGaussianNoise(loc=0, scale=(0.0, 0.05 * 255), per_channel=0.5),
    ], random_order=True)  # apply augmenters in random order

    img["rgb"] = color_seq(images=[img["rgb"]])[0]

    depth_blur = iaa.Sometimes(0.8, iaa.GaussianBlur(sigma=(0, max_depth_blur)))
    depth_noise = iaa.AdditiveGaussianNoise(loc=0, scale=(0.0, max_depth_noise))
    depth_seq = iaa.Sequential([
        # Small gaussian blur with random sigma between 0 and 0.5.
        # But we only blur about 50% of all images.
        iaa.OneOf([
            iaa.BlendAlphaSimplexNoise(
                foreground=depth_blur,
                per_channel=False,
                upscale_method="linear"
            ),
            iaa.BlendAlphaFrequencyNoise(
                foreground=depth_blur,
                per_channel=False,
                upscale_method="linear"
            )
        ]),
        iaa.OneOf([
            iaa.BlendAlphaSimplexNoise(
                foreground=depth_noise,
                per_channel=False,
                upscale_method="linear"
            ),

            iaa.BlendAlphaFrequencyNoise(
                foreground=depth_noise,
                per_channel=False,
                upscale_method="linear"
            ),
            iaa.BlendAlphaSimplexNoise(
                foreground=depth_noise,
                aggregation_method="max",
                per_channel=False,
                sigmoid=False,
                upscale_method="linear"
            )
        ]),
    ], random_order=True)  # apply augmenters in random order

    img["depth"] = depth_seq(images=[img["depth"]])[0]

    img["pointcloud"] = create_pointcloud(img["depth"], fxfypxpy)

    return img
