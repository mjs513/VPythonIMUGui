from struct import unpack
from binascii import unhexlify
from math import *
from array import *
from ctypes import *

euler = array('f',[0.,0.,0.])
q = array('f',[0.,0.,0.,0.])
q1 = array('f',[0.,0.,0.,0.])
acc = array('f',[0.,0.,0.])
magn = array('f',[0.,0.,0.])
gyro = array('f',[0.,0.,0.])
conj = array('f',[0.,0.,0.,0.])
ypr = array('f',[0.,0.,0.])
hq = None

#--------------------------------------------------------------------------------
def quaternionToEuler():
    global q
    global hq
    global euler

    euler[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1)         ## psi
    euler[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2])                                              ## theta
    euler[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1)       ## phi

#--------------------------------------------------------------------
#--------------------------------------------------------------------
#
# Decode an hex representation of a float to a float
#
# See:
# http://stackoverflow.com/questions/1592158/python-convert-hex-to-float/1...
# http://stackoverflow.com/questions/4315190/single-precision-big-endian-f...

def decodeFloat(s):
  #Other possible implementation. Don't know what's better

  s = s[6:8] + s[4:6] + s[2:4] + s[0:2] # reverse the byte order
  i = int(s, 16)                   # convert from hex to a Python int
  cp = pointer(c_int(i))           # make this into a c integer
  fp = cast(cp, POINTER(c_float))  # cast the int pointer to a float pointer
  return fp.contents.value         # dereference the pointer, get the float
  

#------------------------------------------------------------------------------
#
# compensate the accelerometer readings from gravity. 
# @param q the quaternion representing the orientation of a 9DOM MARG sensor array
# @param acc the readings coming from an accelerometer expressed in g
#
# @return a 3d vector representing dinamic acceleration expressed in g

def gravity_compensate(q, acc):
  
  g = [0.0, 0.0, 0.0]
  
  # get expected direction of gravity
  g[0] = 2 * (q[1] * q[3] - q[0] * q[2])
  g[1] = 2 * (q[0] * q[1] + q[2] * q[3])
  g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
  
  # compensate accelerometer readings with the expected direction of gravity
  return [acc[0] - g[0], acc[1] - g[1], acc[2] - g[2]]

def getYawPitchRollRad():
  global q
  global ypr
  
  #gx, gy, gz; // estimated gravity direction
  
  gx = 2 * (q[1]*q[3] - q[0]*q[2])
  gy = 2 * (q[0]*q[1] + q[2]*q[3])
  gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]
  
  ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1)
  ypr[1] = atan(gx / sqrt(gy*gy + gz*gz))
  ypr[2] = atan(gy / sqrt(gx*gx + gz*gz))
  
  
#------------------------------------------------------------------------------
#
# See Sebastian O.H. Madwick report 
# "An efficient orientation filter for inertial and intertial/magnetic sensor arrays"
#  Chapter 2 Quaternion representation
#
#--------------------------------------------------------------------------------
##

def quatProd(a, b):
  q = array('f',[0.,0.,0.,0.])

  q[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3]
  q[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2]
  q[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1]
  q[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]

  return q

 
 #--------------------------------------------------------------------------------
 #
 #
 ## returns a quaternion from an axis angle representation

def quatAxisAngle(axis, angle):
    global q

    halfAngle = angle / 2.0
    sinHalfAngle = sin(halfAngle)
    q[0] = cos(halfAngle);
    q[1] = -axis[0] * sinHalfAngle
    q[2] = -axis[1] * sinHalfAngle
    q[3] = -axis[2] * sinHalfAngle

 #--------------------------------------------------------------------------------
 #
 #
 ## return the quaternion conjugate of quat

def quatConjugate(quat):
  conj = array('f',[0,0,0,0])
  
  conj[0] = quat[0]
  conj[1] = -quat[1]
  conj[2] = -quat[2]
  conj[3] = -quat[3]
  return conj

#------------------------------------------------------------------------------
