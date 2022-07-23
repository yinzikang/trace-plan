# trace_plan介绍

## 1. 文件结构

UR5_gripper&emsp;UR5机器人stl，urdf文件 \
kdl_model.py&emsp;用于导入kdl中的UR5模型，以及前向、逆向运动学 \
test.py&emsp;轨迹规划测试程序\
trace_plan.py&emsp;轨迹规划内容，控制机器人按路径运动，路径生成\
readme.md&emsp;帮助文档

## 2. py文件详解

### 2.1.kdl_model.py

createChain() \
&emsp;利用urdf文件中机械臂、关节位姿信息，搭建kdl机器人模型，用于正逆运动学求解。主要搭建方式为设置joint与frame，然后搭建最小模块Segment，再将Segment相add组成chain并返回。注意搭建时的参考坐标系并不是urdf中的世界坐标系

getForwardKinematics(robot, joint_pos) \
&emsp;对于kdl机器人robot，输入关节角度，输出末端执行器位姿

getInverseKinematics(robot, joint_init_pos, cart_pos_ori) \
&emsp;对于kdl机器人robot，输入末端执行器位姿，输出关节角度

### 2.2.test.py

&emsp;利用xml文件加载mujoco中机器人，初始化相关sim配置 \
&emsp;利用kdl_model.py加载kdl中机器人 \
&emsp;设置起点终点位姿及总点数 \
&emsp;Trajectory_Generation生成笛卡尔空间轨迹 \
&emsp;Move_Along_Array笛卡尔空间轨迹转换到关节空间 \
&emsp;仿真并输出ee_link关于shoulder_link位置\
&emsp;matplot做出末端轨迹运动曲线

### 2.3.trace_plan.py

Move_Along_Array(robot, pos_array, ori_array) \
&emsp;对于kdl机器人robot，输入末端位置（3X3）变化轨迹，以及末端姿态变化轨迹（当前只支持姿态不变的轨迹规划） \
对于轨迹中的每一个末端位姿，利用逆运动学求解出各关节角度并返回，即笛卡尔空间至关节空间

Trajectory_Generation(int_pos, end_pos, dot_num) \
&emsp;对于输入的起点位姿与终点位姿，以及轨迹总共点数，输出末端位置变化直线轨迹
