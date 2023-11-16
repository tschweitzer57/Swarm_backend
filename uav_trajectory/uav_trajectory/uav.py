import numpy as np
import random
import math

# rotation matrix from quaternion
def quatRot(q):
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    r00 = 1 - 2*qy**2 - 2*qz**2
    r01 = 2*qx*qy - 2*qz*qw
    r02 = 2*qx*qz + 2*qy*qw

    r10 = 2*qx*qy + 2*qz*qw
    r11 = 1 - 2*qx**2 - 2*qz**2
    r12 = 2*qy*qz - 2*qx*qw

    r20 = 2*qx*qz + 2*qy*qw
    r21 = 2*qy*qz + 2*qx*qw
    r22 = 1 - 2*qx**2 - 2*qy**2

    qRot = np.array([r00, r01, r02],[r10, r11, r12],[r20, r21, r22])

    return qRot

# transformation euler to quaternion
def euler2quat(yaw, pitch, roll):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qw, qx, qy, qz]

# rotation matrix from euler angles
def eulRotZYX(psi, theta, phi):

    r00 = np.cos(theta) * np.cos(psi)
    r01 = np.sin(phi) * np.sin(theta) * np.cos(psi) - np.cos(phi)*np.sin(phi)
    r02 = np.cos(phi) * np.sin(theta) * np.cos(psi) - np.sin(phi)*np.sin(psi)

    r10 = np.cos(theta) * np.sin(psi)
    r11 = np.sin(phi) * np.sin(theta) * np.sin(psi) - np.cos(phi)*np.cos(psi)
    r12 = np.cos(phi) * np.sin(theta) * np.sin(psi) - np.sin(phi)*np.cos(psi)

    r20 = -np.sin(theta)
    r21 = np.sin(phi) * np.cos(theta)
    r22 = np.cos(phi) * np.cos(theta)

    Rot = np.array([[r00, r01, r02],[r10, r11, r12],[r20, r21, r22]])

    return Rot

def quatRotZYX(psi, theta, phi)
    q = euler2quat(psi, theta, phi)
    return quatRot(q)

#euler norm
def euler_norm(pointa, pointb):
    dist = np.sqrt((pointa[0] - pointb[0])**2 + (pointa[1] - pointb[1])**2 + (pointa[2] - pointb[2])**2)
    return dist

def dcmEulerAngles(R):
    sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])








