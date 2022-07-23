from mujoco_py import load_model_from_path, MjSim, MjViewer
import os
import kdl_model as kdl_model
import PyKDL as kdl
import numpy as np

mj_robot = load_model_from_path("UR5_gripper/UR5gripper.xml")
# mj_robot = load_model_from_path("UR5_gripper/assets/pick_and_place.xml")
sim = MjSim(mj_robot)
viewer = MjViewer(sim)
kdl_robot = kdl_model.createChain()
print(kdl_robot.getNrOfJoints())
sim_state = sim.get_state()
duration = 30000
step = 0
while step < duration:

    # # 通过qpos进行控制1
    # sim_state.qpos[0] = 0
    # sim_state.qpos[1] = 0
    # sim_state.qpos[2] = 0
    # sim_state.qpos[3] = 0
    # sim_state.qpos[4] = 0
    # sim_state.qpos[5] = 0
    #
    # sim.set_state(sim_state)
    #
    # # mujoco正向运动学
    # print('mjc:', sim.data.get_body_xpos('ee_link')-sim.data.get_body_xpos('shoulder_link'))  # ee_link,wrist_3_link
    #
    # # kdl正向运动学
    # kdl_joint_pos = kdl.JntArray(kdl_model.joint_num)
    #
    # for j in range(kdl_model.joint_num):
    #     kdl_joint_pos[j] = sim.data.qpos[j]
    # ee_pos = kdl_model.getForwardKinematics(kdl_robot, kdl_joint_pos)
    # print(ee_pos)

    # 逆运动学实验
    # init_pos
    # [[0, -1, 0;
    # 0, 0, 1;
    # -1, 0, 0]
    # [0.51965,     0.19145,     0.39225]]

    set_r = kdl.Rotation(0, -1, 0, 0, 0, 1, -1, 0, 0)
    set_v = kdl.Vector(0, 0.19145, 0.5)
    set_p = kdl.Frame(set_r, set_v)
    desire_qpos = kdl_model.getInverseKinematics(kdl_robot, kdl.JntArray(kdl_model.joint_num), set_p)
    print('joint:', desire_qpos)
    for j in range(kdl_model.joint_num):
        sim_state.qpos[j] = desire_qpos[j]
    sim.set_state(sim_state)

    kdl_pos = kdl_model.getForwardKinematics(kdl_robot, desire_qpos)
    print('desire:', set_v)
    print('kdl:', kdl_pos)
    print('mj:', sim.data.get_body_xpos('ee_link') - sim.data.get_body_xpos('shoulder_link'), '\n')

    step += 1

    # 执行
    sim.forward()
    sim.step()
    viewer.render()

    # print('mjc:', sim.data.get_body_xpos('robot0:gripper_link') - sim.data.get_body_xpos('robot0:shoulder_pan_link'))
    # print('mjc:', sim.data.get_body_xpos('ee_link') - sim.data.get_body_xpos('shoulder_link'))
    # print('kdl:', ee_pos, '\n')

    if os.getenv('TESTING') is not None:
        break
