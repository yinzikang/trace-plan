import PyKDL as kdl
import numpy as np

joint_num = 6


def createChain():
    jnts = []
    frms = []

    # __init__(self: PyKDL.Joint, arg0: PyKDL.Joint)
    # constructor for joint along x, y or z axis, at origin of reference frame
    jnts.append(kdl.Joint(kdl.Joint.RotZ))
    jnts.append(kdl.Joint(kdl.Joint.RotY))
    jnts.append(kdl.Joint(kdl.Joint.RotY))
    jnts.append(kdl.Joint(kdl.Joint.RotY))
    jnts.append(kdl.Joint(kdl.Joint.RotZ))
    jnts.append(kdl.Joint(kdl.Joint.RotY))

    # Rotx() give the value of the appropriate rotation matrix back.
    frm1 = kdl.Frame(kdl.Rotation.RotY(np.pi/2), kdl.Vector(0, 0.13585, 0))
    frm2 = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, -0.1197, 0.425))
    frm3 = kdl.Frame(kdl.Rotation.RotY(np.pi/2), kdl.Vector(0, 0, 0.39225))
    frm4 = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0.093, 0))
    frm5 = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0, 0.09465))
    frm6 = kdl.Frame(kdl.Rotation.RotZ(np.pi/2), kdl.Vector(0, 0.0823, 0))
    # Quaternion(-0.707107, 0, 0, 0.707107)
    frms.append(frm1)
    frms.append(frm2)
    frms.append(frm3)
    frms.append(frm4)
    frms.append(frm5)
    frms.append(frm6)

    rbt = kdl.Chain()

    link = []
    for i in range(joint_num):
        link.append(kdl.Segment(jnts[i], frms[i]))
        rbt.addSegment(link[i])
    return rbt


def getForwardKinematics(robot, joint_pos):
    fk = kdl.ChainFkSolverPos_recursive(robot)
    cart_pos_ori = kdl.Frame()
    fk.JntToCart(joint_pos, cart_pos_ori)
    return cart_pos_ori


# q1 = [-1.6056,-1.0,-3.141,0.0,-3.141,-2.16,-3.141]
# q2 = [1.6056,0.0,3.141,1.0,-3.141,2.16,3.141]
# qmin = kdl.JntArray(7)
# qmax = kdl.JntArray(7)
# for i in range(7):
#     qmin[i] = q1[i]
#     qmax[i] = q2[i]

def getInverseKinematics(robot, joint_init_pos, cart_pos_ori):
    ik = kdl.ChainIkSolverPos_LMA(robot, maxiter=1500)
    joint_pos = kdl.JntArray(joint_num)
    ik.CartToJnt(joint_init_pos, cart_pos_ori, joint_pos)
    return joint_pos
