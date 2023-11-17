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

    qRot = np.array([[r00, r01, r02],[r10, r11, r12],[r20, r21, r22]])

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

def quatRotZYX(psi, theta, phi):
    q = euler2quat(psi, theta, phi)
    return quatRot(q)

#euler norm
def euler_norm(pointa, pointb):
    dist = np.sqrt((pointa[0] - pointb[0])**2 + (pointa[1] - pointb[1])**2 + (pointa[2] - pointb[2])**2)
    return dist

def dcmEulerAngles(R):
  sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
  singular = sy < 1e-6
  if  not singular :
      phi = math.atan2(R[2,1] , R[2,2])
      theta = math.atan2(-R[2,0], sy)
      psi = math.atan2(R[1,0], R[0,0])
  else :
      phi = math.atan2(-R[1,2], R[1,1])
      theta = math.atan2(-R[2,0], sy)
      psi = 0
  return np.array([phi, theta, psi])

def trans2pose(uav_pose):
  x = uav_pose.Tr[0]
  y = uav_pose.Tr[1]
  z = uav_pose.Tr[2]
  psi = dcm2EulerAngles(uav_pose.Rot)[2]
  return np.array([x,y,z,psi])

# Transformation matrix class
class T:
  def __init__(self, x=0, y=0, z=0, psi=0, theta=0, phi=0, gen='default', transformationMatrix=np.array([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])):
    if(gen == 'default'):
      self.Tr = np.array([x, y, z])
      self.Rot = quatRotZYX(psi, theta, phi)
      self.T = np.array([
          [self.Rot[0,0], self.Rot[0,1], self.Rot[0,2], self.Tr[0]],
          [self.Rot[1,0], self.Rot[1,1], self.Rot[1,2], self.Tr[1]],
          [self.Rot[2,0], self.Rot[2,1], self.Rot[2,2], self.Tr[2]],
          [0,             0,             0,             1]
      ])
    elif(gen == 'tr'):
      self.T = np.array([
          [transformationMatrix[0,0], transformationMatrix[0,1], transformationMatrix[0,2], transformationMatrix[0,3]],
          [transformationMatrix[1,0], transformationMatrix[1,1], transformationMatrix[1,2], transformationMatrix[1,3]],
          [transformationMatrix[2,0], transformationMatrix[2,1], transformationMatrix[2,2], transformationMatrix[2,3]],
          [transformationMatrix[3,0], transformationMatrix[3,1], transformationMatrix[3,2], transformationMatrix[3,3]]
      ])
      self.Tr = np.array([transformationMatrix[0,3], transformationMatrix[1,3], transformationMatrix[2,3]])
      self.Rot = np.array([
          [transformationMatrix[0,0], transformationMatrix[0,1], transformationMatrix[0,2]],
          [transformationMatrix[1,0], transformationMatrix[1,1], transformationMatrix[1,2]],
          [transformationMatrix[2,0], transformationMatrix[2,1], transformationMatrix[2,2]]
      ])

  def inv(self):
    Rt = np.transpose(self.Rot)
    Tt = np.dot(-Rt,self.Tr)
    inv = np.array([
        [Rt[0,0], Rt[0,1], Rt[0,2], Tt[0]],
        [Rt[1,0], Rt[1,1], Rt[1,2], Tt[1]],
        [Rt[2,0], Rt[2,1], Rt[2,2], Tt[2]],
        [0,       0,       0,       1],
    ])
    return inv

  def set(self, matrix):
    self.Tr = np.array([matrix[0,3], matrix[1,3], matrix[2,3]])
    self.Rot = np.array([
        [matrix[0,0], matrix[0,1], matrix[0,2]],
        [matrix[1,0], matrix[1,1], matrix[1,2]],
        [matrix[2,0], matrix[2,1], matrix[2,2]]
    ])
    self.T = np.array([
        [self.Rot[0,0], self.Rot[0,1], self.Rot[0,2], self.Tr[0]],
        [self.Rot[1,0], self.Rot[1,1], self.Rot[1,2], self.Tr[1]],
        [self.Rot[2,0], self.Rot[2,1], self.Rot[2,2], self.Tr[2]],
        [0,             0,             0,             1],
    ])

  def addnoise(self, noise):
    self.Tr = np.array([self.Tr[0] + noise*np.random.normal(),
                        self.Tr[1] + noise*np.random.normal(),
                        self.Tr[2] + noise*np.random.normal()])
    angles = dcm2EulerAngles(self.Rot)
    self.Rot = quatRotZYX(angles[2]+ noise*0.01*np.random.normal(),
                          angles[1]+ noise*0.01*np.random.normal(),
                          angles[0]+ noise*0.01*np.random.normal())
    self.T = np.array([
          [self.Rot[0,0], self.Rot[0,1], self.Rot[0,2], self.Tr[0]],
          [self.Rot[1,0], self.Rot[1,1], self.Rot[1,2], self.Tr[1]],
          [self.Rot[2,0], self.Rot[2,1], self.Rot[2,2], self.Tr[2]],
          [0,             0,             0,             1]
      ])
    
# UAV class (contain ground truth and measurements)
class UAV:
  def __init__(self, initialPose = [0,0,0]):
    self.origin_x = initialPose[0]
    self.origin_y = initialPose[1]
    self.origin_z = initialPose[2]
    self.groundtruth_x = []
    self.groundtruth_y = []
    self.groundtruth_z = []
    self.vio_pose = []
    self.z_VIO = []
    self.z_UWB = []
    self.z_VD = []
    self.z_LC = []

  def generate_3DoF_trajectory(self, diameter = 20, max_height = 7, start_angle = 0):
    height = max_height - 5

    # take off
    takeoffTime = 5
    Nsamples = int(takeoffTime/0.01)

    x = self.origin_x
    y = self.origin_y
    z = self.origin_z

    x_takeoff = np.linspace(x,(diameter/2) * np.sin(start_angle) + x,Nsamples)
    y_takeoff = np.linspace(y,(diameter/2) * np.cos(start_angle) + y,Nsamples)
    z_takeoff = np.linspace(z,height,Nsamples)

    # flight
    flightTime = 90
    Nsamples = int(flightTime/0.01)

    theta = np.linspace(start_angle, start_angle + 20*np.pi, Nsamples)
    x_flight = (diameter/2) * np.sin(theta) + x * np.ones(int(Nsamples),)
    y_flight = (diameter/2) * np.cos(theta) + y * np.ones(int(Nsamples),)
    z_flight = np.linspace(height,max_height,int(Nsamples))

    # landing
    landingTime = 5
    Nsamples = int(landingTime/0.01)
    print("Nombre de points :", Nsamples)

    x_landing = x_flight[-1] * np.ones(int(Nsamples),)
    y_landing = y_flight[-1] * np.ones(int(Nsamples),)
    z_landing = np.linspace(max_height, z, int(Nsamples))

    #Concatenate flight periods
    self.groundtruth_x = np.concatenate([x_takeoff, x_flight, x_landing])
    self.groundtruth_y = np.concatenate([y_takeoff, y_flight, y_landing])
    self.groundtruth_z = np.concatenate([z_takeoff, z_flight, z_landing])

  def generate_VIO(self, noise=0):
    self.z_VIO = []
    self.vio_pose = []

    Pinit = T(0,0,0,0)
    Pnew = T(self.groundtruth_x[0],self.groundtruth_y[0],self.groundtruth_z[0],0)
    self.z_VIO.append(Pinit)
    self.vio_pose.append(Pnew)

    for i in range(1,len(self.groundtruth_x)):
      Pold = Pnew
      Pnew = T(self.groundtruth_x[i]+noise*np.random.normal(),
               self.groundtruth_y[i]+noise*np.random.normal(),
               self.groundtruth_z[i]+noise*np.random.normal(),
               0+noise*0.01*np.random.normal())
      self.vio_pose.append(Pnew)
      dp = np.dot(Pold.inv(),Pnew.T)
      self.z_VIO.append(dp)

    print("Successfully generated VIO data !")


  def generate_UWB(self, uav2, uav3, uav4, noise=0):
    self.z_UWB = []

    for i in range(0,len(uav2.groundtruth_x)):
      self_pose = [self.groundtruth_x[i],self.groundtruth_y[i],self.groundtruth_z[i]]
      uav2_pose = [uav2.groundtruth_x[i],uav2.groundtruth_y[i],uav2.groundtruth_z[i]]
      uav3_pose = [uav3.groundtruth_x[i],uav3.groundtruth_y[i],uav3.groundtruth_z[i]]
      uav4_pose = [uav4.groundtruth_x[i],uav4.groundtruth_y[i],uav4.groundtruth_z[i]]

      uwb_measure = {"distance to B": (euler_norm(self_pose, uav2_pose)+noise*np.random.normal()),
                     "distance to C": (euler_norm(self_pose, uav3_pose)+noise*np.random.normal()),
                     "distance to D": (euler_norm(self_pose, uav4_pose)+noise*np.random.normal())}
      self.z_UWB.append(uwb_measure)

    print("Successfully generated UWB data !")

  def generate_VD(self, uav2, uav3, uav4, noise=0):
    self.z_VD = []

    for i in range(0,len(self.groundtruth_x)):
      vd_measure = {"B detected": None,
                    "C detected": None,
                    "D detected": None}

      if (np.random.normal()>1.5):
        drone = random.randint(1,3)

        if (np.random.normal()>1) or (drone == 1): #Drone B detected
          xb = uav2.groundtruth_x[i] + noise*np.random.normal()
          yb = uav2.groundtruth_y[i] + noise*np.random.normal()
          zb = uav2.groundtruth_z[i] + noise*np.random.normal()
          psib = 0 + noise*0.01*np.random.normal()
          Pb = T(xb, yb, zb, psib)

          vd_measure["B detected"] = np.dot(self.vio_pose[i].inv(),Pb.T)

        if (np.random.normal()>1) or (drone == 2): #Drone C detected
          xc = uav3.groundtruth_x[i] + noise*np.random.normal()
          yc = uav3.groundtruth_y[i] + noise*np.random.normal()
          zc = uav3.groundtruth_z[i] + noise*np.random.normal()
          psic = 0 + noise*0.01*np.random.normal()
          Pc = T(xc, yc, zc, psic)

          vd_measure["C detected"] = np.dot(self.vio_pose[i].inv(),Pc.T)

        if (np.random.normal()>1) or (drone == 3): #Drone D detected
          xd = uav4.groundtruth_x[i] + noise*np.random.normal()
          yd = uav4.groundtruth_y[i] + noise*np.random.normal()
          zd = uav4.groundtruth_z[i] + noise*np.random.normal()
          psid = 0 + noise*0.01*np.random.normal()
          Pd = T(xd, yd, zd, psid)

          vd_measure["D detected"] = np.dot(self.vio_pose[i].inv(),Pd.T)

      self.z_VD.append(vd_measure)

    print("Successfully generated VD data !")

  def generate_LC(self, uav2, uav3, uav4, noise=0):
    self.z_LC = []
    nb_drones = 4

    for i in range(0,len(self.groundtruth_x)):
      lc_measure = {"lc detected AKF": None,
                    "lc detected BKF": None,
                    "lc detected CKF": None,
                    "lc detected DKF": None,
                    "lc measure AKF": None,
                    "lc measure BKF": None,
                    "lc measure CKF": None,
                    "lc measure DKF": None,
                    "lc time AKF": None,
                    "lc time BKF": None,
                    "lc time CKF": None,
                    "lc time DKF": None,}

      for d in range(1,nb_drones+1):
        if (np.random.normal()>2.5 and i>4):
          matchedKF = random.randint(1,4)
          time = random.randint(0,i-1)

          Pmatched = self.get_pose(matchedKF, time, uav2, uav3, uav4)
          Prec = self.get_pose(d, i, uav2, uav3, uav4)

          measure = T(gen='tr', transformationMatrix=np.dot(Pmatched.inv(), Prec.T))
          measure.addnoise(noise)

          lc_measure[self.lc_tag(d, "detected")]= self.lc_tag(d, "matched", matched_drone = matchedKF)
          lc_measure[self.lc_tag(d, "measure")]=measure.T
          lc_measure[self.lc_tag(d, "time")]=time

      self.z_LC.append(lc_measure)

    print("Successfully generated LC data !")

  def lc_tag(self, droneRec, tag, matched_drone = 0):

    if (tag == "time"):
      if(droneRec == 1):
        tag = "lc time AKF"
      elif(droneRec == 2):
        tag = "lc time BKF"
      elif(droneRec == 3):
        tag = "lc time CKF"
      elif(droneRec == 4):
        tag = "lc time DKF"

    elif(tag == "measure"):
      if(droneRec == 1):
        tag = "lc measure AKF"
      elif(droneRec == 2):
        tag = "lc measure BKF"
      elif(droneRec == 3):
        tag = "lc measure CKF"
      elif(droneRec == 4):
        tag = "lc measure DKF"

    elif(tag == "detected"):
      if(droneRec == 1):
        tag = "lc detected AKF"
      elif(droneRec == 2):
        tag = "lc detected BKF"
      elif(droneRec == 3):
        tag = "lc detected CKF"
      elif(droneRec == 4):
        tag = "lc detected DKF"

    elif(tag == "matched"):
      if(matched_drone == 1):
        tag = "A"
      elif(matched_drone == 2):
        tag = "B"
      elif(matched_drone == 3):
        tag = "C"
      elif(matched_drone == 4):
        tag = "D"

    return tag

  def get_pose(self, drone, t, uav2, uav3, uav4):
    if(drone == 1):
      pose = self.vio_pose[t]
    elif(drone == 2):
      pose = uav2.vio_pose[t]
    elif(drone == 3):
      pose = uav3.vio_pose[t]
    elif(drone == 4):
      pose = uav4.vio_pose[t]

    return pose

#Pour l'affichage :
  def poseVector(self, axis=0):
    vector = []
    for i in range(0,len(self.vio_pose)):
      if (axis==0):
        vector.append(self.vio_pose[i].Tr[0])
      elif(axis==1):
        vector.append(self.vio_pose[i].Tr[1])
      elif(axis==2):
        vector.append(self.vio_pose[i].Tr[2])

    return vector

  def get_lcVector(self, axis=0):
    vector = []
    for i in range(0,len(self.z_LC)):

      if(self.z_LC[i]["lc detected AKF"] is not None):
        if (axis==0):
          vector.append(self.vio_pose[i].Tr[0])
        elif(axis==1):
          vector.append(self.vio_pose[i].Tr[1])
        elif(axis==2):
          vector.append(self.vio_pose[i].Tr[2])

    return vector

  def get_vdVector(self, axis=0):
    vector = []
    for i in range(0,len(self.z_VD)):

      if(((self.z_VD[i]["B detected"]) is not None) or (self.z_VD[i]["C detected"] is not None) or (self.z_VD[i]["D detected"] is not None)):
        if (axis==0):
          vector.append(self.vio_pose[i].Tr[0])
        elif(axis==1):
          vector.append(self.vio_pose[i].Tr[1])
        elif(axis==2):
          vector.append(self.vio_pose[i].Tr[2])

    return vector




