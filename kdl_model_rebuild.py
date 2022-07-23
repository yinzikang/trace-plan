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
    frm1 = kdl.Frame(kdl.Rotation.RotX(0), kdl.Vector(0, 0.13585, 0))
    frm2 = kdl.Frame(kdl.Rotation.RotX(0), kdl.Vector(0.425, -0.1197, 0))
    frm3 = kdl.Frame(kdl.Rotation.RotX(0), kdl.Vector(0.39225, 0, 0))
    frm4 = kdl.Frame(kdl.Rotation.RotX(0), kdl.Vector(0, 0.093, 0))
    frm5 = kdl.Frame(kdl.Rotation.RotX(0), kdl.Vector(0, 0, -0.09465))
    frm6 = kdl.Frame(kdl.Rotation.RotX(0), kdl.Vector(0, 0, 0.0823))

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


def getForwardKinematics(rbt, q):
    fk = kdl.ChainFkSolverPos_recursive(rbt)
    p = kdl.Frame()
    fk.JntToCart(q, p)
    return p


# q1 = [-1.6056,-1.0,-3.141,0.0,-3.141,-2.16,-3.141]
# q2 = [1.6056,0.0,3.141,1.0,-3.141,2.16,3.141]
# qmin = kdl.JntArray(7)
# qmax = kdl.JntArray(7)
# for i in range(7):
#     qmin[i] = q1[i]
#     qmax[i] = q2[i]

def getInverseKinematics(rbt, q_init, p):
    ik = kdl.ChainIkSolverPos_LMA(rbt, maxiter=1500)
    q = kdl.JntArray(joint_num)
    ik.CartToJnt(q_init, p, q)
    return q
