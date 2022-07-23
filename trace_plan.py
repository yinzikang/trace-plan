import numpy as np
import numpy as py
import PyKDL as kdl
import kdl_model


def Move_Along_Array(robot, pos_array, ori_array):
    pos_num = len(pos_array)
    # ori_num = len(ori_array)
    # if pos_num != ori_num:
    #     print('error path')

    joint_last_pos = kdl.JntArray(robot.getNrOfJoints())
    joint_array = []

    for current_path_id in range(pos_num):
        set_v = pos_array[current_path_id]
        # set_r = ori_array[current_path_id]
        set_r = ori_array
        set_p = kdl.Frame(set_r, set_v)
        joint_current_pos = (kdl_model.getInverseKinematics(robot, joint_last_pos, set_p))
        joint_last_pos = joint_current_pos
        joint_array.append(joint_current_pos)

    return joint_array


def Trajectory_Generation(int_pos, end_pos, dot_num):
    trajectory = []
    for i in range(dot_num):
        trajectory.append(int_pos + (end_pos - int_pos) / (dot_num - 1) * i)
    return trajectory


# int_pos1 = kdl.Vector(0.5, 0.19145, 0.5)
# end_pos1 = kdl.Vector(0, 0.19145, 0.5)
# tra = Trajectory_Generation(int_pos1, end_pos1, 5)
# print(tra)
