import numpy as np
from mujoco_py import load_model_from_path, MjSim, MjViewer
import os
# import mujoco_env

model = load_model_from_path("UR5_gripper/UR5gripper.xml")
sim = MjSim(model)
viewer = MjViewer(sim)

sim_state = sim.get_state()
duration = 10000
step = 0
while step < duration:

    # 通过qpos进行控制1
    # sim_state.qpos[3] = -1
    # sim.set_state(sim_state)

    # 通过qpos进行控制2
    sim_state.qvel[0] = 100
    sim.set_state(sim_state)

    # 通过ctrl进行控制
    # sim.data.ctrl[0] = -5
    # sim_state = sim.get_state()
    # if sim_state.qpos[0] < -1.5707:
    #     sim_state.qpos[0] = 1.5707
    #     sim.set_state(sim_state)

    # 通过mocap_pos进行控制
    # sim.data.set_mocap_pos("mocap", np.array([0, 0, 0.5]))

    # 执行
    sim.forward()
    sim.step()
    viewer.render()

    # 显示结果
    print(sim_state.qvel[2])
    print(sim.data.get_body_xpos('ee_link'))  # ee_link,box_link
    step += 1

    if os.getenv('TESTING') is not None:
        break
