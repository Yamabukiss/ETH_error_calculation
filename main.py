""" created by yamabuki 2023/7/28 """

"""
   It's for calculating the errors from mech-mind ETH calibration

   Robot, Calibration result coordinate system:
       z
      |
      |
      |______ y
     /
    /
   x

   Camera coordinate system:

   y ______
         /|
        / |
       x  z
"""

import numpy as np
import math


def transformPointToRobotBase(t_camera_to_robot, _camera_point):
    point_camera_homogeneous = np.append(_camera_point, 1)

    point_robot_homogeneous = np.dot(t_camera_to_robot, point_camera_homogeneous)

    point_robot = point_robot_homogeneous[:3] / point_robot_homogeneous[3]

    return point_robot


def quaternionToRotationMatrix(_quaternion):
    q0, q1, q2, q3 = _quaternion
    rotation_matrix = np.array([
        [1 - 2 * q2 ** 2 - 2 * q3 ** 2, 2 * q1 * q2 - 2 * q0 * q3, 2 * q1 * q3 + 2 * q0 * q2],
        [2 * q1 * q2 + 2 * q0 * q3, 1 - 2 * q1 ** 2 - 2 * q3 ** 2, 2 * q2 * q3 - 2 * q0 * q1],
        [2 * q1 * q3 - 2 * q0 * q2, 2 * q2 * q3 + 2 * q0 * q1, 1 - 2 * q1 ** 2 - 2 * q2 ** 2]
    ])
    return rotation_matrix


def poseToTransformationMatrix(_translation, _quaternion):
    rotation_matrix = quaternionToRotationMatrix(_quaternion)
    _transformation_matrix = np.eye(4)
    _transformation_matrix[:3, :3] = rotation_matrix
    _transformation_matrix[:3, 3] = _translation
    return _transformation_matrix


if __name__ == '__main__':
    try:
        with open("./ExtrinsicParameters.txt", "r") as f:
            text = f.readlines()[1]
            extrinsic_trans = text.split(",")[:3]
            extrinsic_quaternions = text.split(",")[3:]
            translation = np.array(extrinsic_trans).astype("float")
            quaternion = np.array(extrinsic_quaternions).astype("float")
            f.close()
    except:
        print("[ERROR] you have no ExtrinsicParameters.txt in the same path with this program")
        input("[INFO] Press Enter to exit or just close the window")
        exit()

    transformation_matrix = poseToTransformationMatrix(translation, quaternion)

    print("[INFO] The point should be the end of tool")

    camera_point_str = input("[INFO] Input the translation of point(m) in camera coordinate system (split with space "
                             "key):\n")
    camera_point_lst = camera_point_str.split(" ")

    if len(camera_point_lst) != 3:
        print("[ERROR] You should only input 3 args")
        input("[INFO] Press Enter to exit or just close the window")
        exit()

    camera_point = np.array(camera_point_lst).astype("float")

    eval_base_point = transformPointToRobotBase(transformation_matrix, camera_point)
    print("[INFO] The tool end in base from ETH calibration: ", eval_base_point)

    gt_base_trans_str = input("[INFO] Input the translation of tool end(mm) in base (split with space key):\n")
    gt_base_trans_lst = gt_base_trans_str.split(" ")

    if len(gt_base_trans_lst) != 3:
        print("[ERROR] You should only input 3 args")
        input("[INFO] Press Enter to exit or just close the window")
        exit()

    gt_base_trans = np.array(gt_base_trans_lst).astype("float") / 1000.

    print("[INFO] The tool end in base from robot base: ", gt_base_trans)

    error_lst = []
    distance = 0
    for i in range(3):
        error_lst.append(abs(gt_base_trans[i] * 1000 - eval_base_point[i] * 1000))
        distance += pow(gt_base_trans[i] * 1000 - eval_base_point[i] * 1000, 2)
    distance = math.sqrt(distance)

    print("[INFO] Absolute error(mm) per axis: ", error_lst)
    print("[INFO] Euclidean distance(mm)", distance)

    input("[INFO] Press Enter to exit or just close the window")
