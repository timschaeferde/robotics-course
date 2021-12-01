import glob
import json
import os
import sys
from datetime import datetime

import pandas as pd

sys.path.append("./scripts/")

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

from tools.utils import position_loss, quaternion_loss, json_load, json_dump, add_loss, pickle_dump, pickle_load

# changing working directory to the package location
# import rospkg
# rospack = rospkg.RosPack()
# rai_baxter_path = rospack.get_path('rai_baxter')
# os.chdir(rai_baxter_path)
# print("working in: ", rai_baxter_path)
# sys.path.append("./scripts/")
# sys.path.append("..")
# rospack = rospkg.RosPack()

from CONFIG import *


def calc_losses():
    # generate pointcloud models of objects if not existing
    if os.path.exists(model_points_path):
        print("loading existing models")
        model_points = pickle_load(model_points_path)
    else:
        number_of_points = 1000
        model_points = {}
        for cls, mesh in zip(classes, OBJ_MESHES):
            mesh_path = os.path.join(mesh_dir_path, mesh + ".obj")
            # load mesh
            mesh = o3d.io.read_triangle_mesh(mesh_path)
            pointcloud = mesh.sample_points_poisson_disk(number_of_points)
            # add to dict
            model_points[cls] = np.asarray(pointcloud.points)

        pickle_dump(model_points, model_points_path)

    # setup losses dict
    losses = {}
    for key in pose_keys:
        losses[key] = {}
        for cls in classes:
            losses[key][cls] = {"position": [], "quaternion": [], "add-s": []}

    # calculate losses
    for path in file_paths[:]:
        pred_metadata = json_load(path)
        for data in pred_metadata:
            print(data["name"])
            if data["gt_pose"] is not None:
                gt_pose = np.asarray(data["gt_pose"])
                for key in pose_keys:
                    pose = np.asarray(data[key])
                    losses[key][data["class"]]["position"].append(position_loss(pose[:3], gt_pose[:3]))
                    losses[key][data["class"]]["quaternion"].append(quaternion_loss(pose[3:], gt_pose[3:],
                                                                                    model_points[data["class"]]))
                    losses[key][data["class"]]["add-s"].append(add_loss(pose, gt_pose, model_points[data["class"]]))

    # save data to files here
    # save losses
    json_dump(losses, losses_path + "/losses.json")
    # save mean losses
    for key in pose_keys:
        for cls in classes:
            losses[key][cls]["quaternion"] = np.asarray(losses[key][cls]["quaternion"]).mean()
            losses[key][cls]["add-s"] = np.asarray(losses[key][cls]["add-s"]).mean()
            losses[key][cls]["position"] = np.asarray(losses[key][cls]["position"]).mean()

    json_dump(losses, losses_path + "/mean_losses.json")


def auc_metric(loss_array: np.array, plot=False):
    points = 100
    max_threshold = .1
    accuracy = [0]
    threshold = [0]
    for i in range(points):
        t = (i + 1) / points * max_threshold
        threshold.append(t)
        count = (loss_array < t).sum()
        accuracy.append(count / loss_array.shape[0])
    if plot:
        plt.plot(threshold, accuracy)
        plt.ylabel('accuracy')
        plt.xlabel('threshold in meter')
        plt.show()
    return sum(accuracy) / len(accuracy), threshold, accuracy


def smaller_2cm_metric(loss_array: np.array):
    return (loss_array < 0.02).sum() / loss_array.shape[0]


def plot_auc_metric(timestamp):
    print(losses_path + "/losses.json")
    losses = json_load(losses_path + "/losses.json")
    loss = "add-s"
    for pose_name in pose_keys:
        auc_all = 0
        for i, class_name in enumerate(classes):
            loss_array = np.asarray(losses[pose_name][class_name][loss])
            auc, threshold, accuracy = auc_metric(loss_array, plot=False)
            auc_all += auc
            if i == 0:
                thres, acc = np.asarray(threshold), np.asarray(accuracy)
            else:
                acc += accuracy
        auc_all /= len(classes)
        print(auc_all)
        acc /= len(classes)

        plt.plot(thres, acc, label=pose_name)
    plt.legend()
    plt.title("add-s all 7 objects")
    plt.ylabel('accuracy')
    plt.xlabel('threshold in meter')

    plt.savefig('{}_all_{}.pdf'.format(timestamp, loss))
    plt.show()


def plot_single_auc_metric(timestamp):
    print(timestamp)
    losses = json_load(losses_path + "/losses.json")
    loss = "add-s"
    pose_name = "mesh_pose"
    class_name = "lego_toy"
    loss_array = np.asarray(losses[pose_name][class_name][loss])
    auc, threshold, accuracy = auc_metric(loss_array, plot=False)

    plt.plot(threshold, accuracy, label=pose_name)
    plt.legend()
    plt.title("add-s {} {}".format(pose_name, class_name))
    plt.ylabel('accuracy')
    plt.xlabel('threshold in meter')
    plt.savefig('{}_add-s_{}_{}.pdf'.format(timestamp, pose_name, class_name))
    plt.show()


def plot_all_single_auc_metric(timestamp):
    print(timestamp)
    losses = json_load(losses_path + "/losses.json")
    loss = "add-s"
    for pose_name in pose_keys:
        for i, class_name in enumerate(classes):
            loss_list = losses[pose_name][class_name][loss]
            if loss_list is None: print("ups NONE!")
            loss_array = np.asarray(loss_list)
            auc, threshold, accuracy = auc_metric(loss_array, plot=False)
            plt.plot(threshold, accuracy, label=pose_name)
            plt.legend()
            plt.title("add-s {} {}".format(pose_name, class_name))
            plt.ylabel('accuracy')
            plt.xlabel('threshold in meter')
            plt.savefig('{}_add-s_{}_{}.pdf'.format(timestamp, pose_name, class_name))
            plt.show()


def print_table_auc_all():
    losses = json_load(losses_path + "/losses.json")
    # create multicolums for table
    colums = [[], []]
    for pose_name in pose_keys:
        for metric in ["AUC", "<2cm"]:
            colums[0].append(pose_name.replace('_pose', '').replace("_", " "))
            colums[1].append(metric)

    loss = "add-s"
    # numpy data for table
    data = np.zeros((len(classes), len(colums[0])))
    for j, pose_name in enumerate(pose_keys):
        for i, class_name in enumerate(classes):
            loss_array = np.asarray(losses[pose_name][class_name][loss])
            auc, _, _ = auc_metric(loss_array, plot=False)
            data[i, j * 2 + 0] = auc * 100
            data[i, j * 2 + 1] = smaller_2cm_metric(loss_array) * 100

    df = pd.DataFrame(data, index=classes, columns=colums)
    df.loc['mean'] = df.mean()

    print(df.to_latex(index=True, float_format="{:0.1f}".format))


def print_table_position_quaternion():
    losses = json_load(losses_path + "/losses.json")

    # create multicolums for table
    colums = [[], []]
    for pose_name in pose_keys:
        for metric in ["t", "R"]:
            colums[0].append(pose_name.replace('_pose', '').replace("_", " "))
            colums[1].append(metric)

    loss = "add-s"
    # numpy data for table
    data = np.zeros((len(classes), len(colums[0])))
    for j, pose_name in enumerate(pose_keys):
        for i, class_name in enumerate(classes):
            data[i, j * 2 + 0] = auc_metric(np.asarray(losses[pose_name][class_name]["position"]), plot=False)[0] * 100
            data[i, j * 2 + 1] = auc_metric(np.asarray(losses[pose_name][class_name]["quaternion"]), plot=False)[
                                     0] * 100

    df = pd.DataFrame(data, index=classes, columns=colums)
    df.loc['mean'] = df.mean()

    print(df.to_latex(index=True, float_format="{:0.1f}".format))


if __name__ == '__main__':
    set_name = "set7.1"

    mesh_dir_path = MESH_DIR  # os.path.join(rospack.get_path(MESH_PKG), MESH_DIR)
    model_points_path = POINT_MODELS_PATH
    losses_path = "./data/{}/".format(set_name)
    data_path = "./data/{}/pose_data/".format(set_name)

    timestamp = datetime.now().strftime("%y%m%dT%H%M%S")
    print(timestamp)

    classes = ["cube", "sphere", "lego_toy", "teapot", "cup", "jug", "bowl"]
    # name of the corresponding mesh (.obj) files for dbot tracking
    OBJ_MESHES = ["cube_8cm", "sphere130_0.6k", "lego_toy_1k_centered", "teapot_centered", "cup", "jug2_centered",
                  "bowl_centered"]

    pose_keys = ['mask_pose', 'mesh_pose']#, 'particle_pose_1s', 'gaussian_pose_1s', 'particle_pose_3s', 'gaussian_pose_3s']

    file_paths = glob.glob(data_path + "/*_pred.json")


    print(set_name)

    calc_losses()
    # plot_auc_metric(timestamp)
    # plot_single_auc_metric(timestamp)
    #plot_all_single_auc_metric(timestamp)
    print_table_auc_all()
    print_table_position_quaternion()
