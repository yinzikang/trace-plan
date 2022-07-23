import numpy as np
from mujoco_py import load_model_from_path, MjSim, MjViewer
import os
import kdl_model as kdl_model
import PyKDL as kdl
import trace_plan
import matplotlib.pyplot as plt
ax = plt.axes(projection='3d')

mj_robot = load_model_from_path("UR5_gripper/UR5gripper.xml")
sim = MjSim(mj_robot)
sim_state = sim.get_state()
viewer = MjViewer(sim)

kdl_robot = kdl_model.createChain()

duration = 10000
step = 0
int_pos = kdl.Vector(0.5, 0.19145, 0)
tar_pos = kdl.Vector(0, 0.19145, 0.5)
int_ori = kdl.Rotation(0, 1, 0, 1, 0, 0, 0, 0, -1)
cart_path = trace_plan.Trajectory_Generation(int_pos, tar_pos, duration)
joint_path = trace_plan.Move_Along_Array(kdl_robot, cart_path, int_ori)
ee_pos = np.empty(shape=(duration, 3))
# for i in range(kdl_model.joint_num):
#     sim_state.qpos[i] = 0

while step < duration:

    for i in range(kdl_model.joint_num):
        sim_state.qpos[i] = joint_path[step][i]

    # sim_state.qvel[0] = 1
    # # sim.set_joint_qvel[0] = 1
    sim.set_state(sim_state)



    # 执行

    sim.forward()
    sim.step()
    viewer.render()

    ee_pos[step] = sim.data.get_body_xpos('ee_link') - sim.data.get_body_xpos('shoulder_link')
    print(ee_pos[step])
    step += 1

    if os.getenv('TESTING') is not None:
        break

# np.zeros(10000)
ax.scatter3D(ee_pos[:, 0], ee_pos[:, 1], ee_pos[:, 2], cmap='Blues')
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.set_zlim(0, 1)
plt.show()
