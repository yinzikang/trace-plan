from mujoco_py import load_model_from_path, MjSim


class my_ur5_env():
    def __init__(self, env, args):
        # super(lab_env, self).__init__(env)
        # 导入xml文档
        self.model = load_model_from_path("UR5_gripper/UR5gripper.xml")
        # 调用MjSim构建一个basic simulation
        self.sim = MjSim(model=self.model)

    def get_state(self, *args):
        self.sim.get_state()
        # 如果定义了相机
        # self.sim.data.get_camera_xpos('[camera name]')

    def reset(self, *args):
        self.sim.reset()

    def step(self, *args):
        self.sim.step()
