import ur5_env


my_env = ur5_env.my_ur5_env()
status = my_env.get_state()
duration = 30000
step = 0
while step < duration:
    status.qpod[0] = 1
    my_env.sim.set_state(status)
    my_env.step()

