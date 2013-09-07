# Tested on Razor 9DOF IMU Sensor Stick, DFRobot 10DOF IMU and FreeIMU v0.4
# Initial code code developed by Jose Julio @2009 and modified by Mike Smorto @2013
# Changes include:
#   1. Created a color coded rectange similar to that of the FreeIMU Demo Cube
#   2. Created a downward pointing arrow to assist in direction visualization for
#      z-axis
#   3. Changed font sizes
#   4. Added data display window to see the data stream
#   5. Added routines to convert quatenions to Euler angle ported from the
#      FreeIMU Demo Cube based in Processing
# 


#from __future__ import print_function, division
from __future__ import division

import serial
import string
import time

from visual import *
from struct import unpack
from binascii import unhexlify
from math import *
from array import *
from ctypes import *
from Quarterion import *

grad2rad = 3.141592/180.0

roll=0
pitch=0
yaw=0
temp = 0.
baro=0
alt=0
count = 0
burst = 32
told = 0
tnew = 0
dt = 0
alt=0
press = 0.
prod_q =None
hq = None
k = 'n'

free_q =array('f',[0.,0.,0.,0.])

sea_press = 1011.9

SerialPort = 'COM9'

#euler = array('f',[0.,0.,0.])
#q = array('f',[0.,0.,0.,0.])
#acc = array('f',[0.,0.,0.])
#magn = array('f',[0.,0.,0.])
#gyro = array('f',[0.,0.,0.])

# Main scene
scene=display(title="10DOF DFRobot IMU test")
scene.range=(1.2,1.2,1.2)

#scene.forward = (0,-1,-0.25)
scene.forward = (1,0,-0.25)
scene.up=(0,0,1)
scene.width=600
scene.y=300

# Second scene (Roll, Pitch, Yaw)
scene2 = display(title='FreeIMU v0.4.3 IMU Data',x=0, y=0, width=600, height=300,center=(0,0,0), background=(0,0,0))
scene2.range=(1,1,1)

scene3 = display(title='FreeIMU IMU Data', x=650,y=1.5, width=600, height= 400, center=(0,0,0), background=(0,0,0))
scene3.range=(10,10,10)
#Set up accelerometer display
label(pos=(-8,4.5,0),text='accx:',height=14, color=color.white,box=0)
alx=label(pos=(-5,4.5,0),text=str(acc[0]),height=14, color=color.white, box=0)
label(pos=(-8,3.0,0),text='accy:',height=14, color=color.white, box=0)
aly=label(pos=(-5,3.0,0),text=str(acc[1]),height=14, color=color.white, box=0)
label(pos=(-8,1.5,0),text='accy:',height=14, color=color.white, box=0)
alz=label(pos=(-5,1.5,0),text=str(acc[2]),height=14, color=color.white, box=0)
#Set up gyro display
label(pos=(-2,4.5,0),text='gyrox:',height=14, color=color.white, box=0)
glx=label(pos=(1.2,4.5,0),text=str(gyro[0]),height=14, color=color.white, box=0)
label(pos=(-2,3.0,0),text='gyroy:',height=14, color=color.white, box=0)
gly=label(pos=(1.2,3.0,0),text=str(gyro[1]),height=14, color=color.white, box=0)
label(pos=(-2,1.5,0),text='gyroz:',height=14, color=color.white, box=0)
glz=label(pos=(1.2,1.5,0),text=str(gyro[2]),height=14, color=color.white, box=0)
#Set up magn display
label(pos=(4,  4.5,0),text='magnx:',height=14, color=color.white, box=0)
mlx=label(pos=(6.3,4.5,0),text=str(magn[0]),height=14, color=color.white, box=0)
label(pos=(4,  3,0),text='magny:',height=14, color=color.white, box=0)
mly=label(pos=(6.3,3,0),text=str(magn[1]),height=14, color=color.white, box=0)
label(pos=(4,  1.5,0),text='magnz:',height=14, color=color.white, box=0)
mlz=label(pos=(6.3,1.5,0),text=str(magn[2]),height=14, color=color.white, box=0)

#Set up q display
label(pos=(-8,-0.5,0),text='q0:',height=14, color=color.white, box=0)
qq0=label(pos=(-5.9,-0.5,0),text=str(q[0]),height=14, color=color.white, box=0)
label(pos=(-8,-2.0,0),text='q1:',height=14, color=color.white, box=0)
qq1=label(pos=(-5.9,-2.0,0),text=str(q[1]),height=14, color=color.white, box=0)
label(pos=(-8,-3.5,0),text='q2:',height=14, color=color.white, box=0)
qq2=label(pos=(-5.9,-3.5,0),text=str(q[2]),height=14, color=color.white, box=0)
label(pos=(-8,-5,0),text='q3:',height=14, color=color.white, box=0)
qq3=label(pos=(-5.9,-5,0),text=str(q[3]),height=14, color=color.white, box=0)

#Set up temp, barometer, deltaT and altitude
label(pos=(-2,-2,0),text='Temp:',height=14, color=color.white, box=0)
ltp=label(pos=(1,-2.0,0),text=str(temp),height=14, color=color.white, box=0)
label(pos=(-2,-4,0),text='Baro:',height=14, color=color.white, box=0)
lpress = label(pos=(1,-4.0,0),text=str(baro),height=14, color=color.white, box=0)
label(pos=(4,-4,0),text='dT:',height=14, color=color.white, box=0)
ldt=label(pos=(6.5,-4.0,0),text=str(dt),height=14, color=color.white, box=0)
label(pos=(4,-2,0),text='Alt:',height=14, color=color.white, box=0)
lalt=label(pos=(6.5,-2.0,0),text=str(alt),height=14, color=color.white, box=0)

scene2.select()

#Roll, Pitch, Yaw
cil_cir_roll = cylinder(pos=(-0.5,-0.1,0),axis=(0,0,-0.01),radius=0.21,color=color.white)
cil_roll = cylinder(pos=(-0.5,-0.1,0),axis=(0.2,0,0),radius=0.01,color=color.red)
cil_roll2 = arrow(pos=(-0.5,-0.1,0),axis=(-0.2,0,0),radius=0.01,color=color.red)

cil_cir_pitch = cylinder(pos=(0.05,-0.1,0),axis=(0,0,-0.01),radius=0.21,color=color.white)
cil_pitch = arrow(pos=(0.05,-0.1,0),axis=(0.2,0,0),radius=0.01,color=color.green)
cil_pitch2 = cylinder(pos=(0.05,-0.1,0),axis=(-0.2,0,0),radius=0.01,color=color.green)

#cil_course = cylinder(pos=(0.6,0,0),axis=(0.2,0,0),radius=0.01,color=color.blue)
#cil_course2 = cylinder(pos=(0.6,0,0),axis=(-0.2,0,0),radius=0.01,color=color.blue)
cil_cir_roll = cylinder(pos=(0.6,-0.1,0),axis=(0,0,-0.01),radius=0.19,color=color.white)
arrow_course = arrow(pos=(0.6,-0.1,0),color=color.cyan,axis=(-0.2,0,0), shaftwidth=0.02, fixedwidth=1)

#Roll,Pitch,Yaw labels
label(pos=(-0.5,0.3,0),height=14,text="Roll",box=0,opacity=0)
label(pos=(0.0,0.3,0),height=14,text="Pitch",box=0,opacity=0)
label(pos=(0.61,0.3,0),height=14,text="Yaw",box=0,opacity=0)

label(pos=(0.6,0.12,0),text="N",box=0,opacity=0,color=color.yellow)
label(pos=(0.6,-0.32,0),text="S",box=0,opacity=0,color=color.yellow)
label(pos=(0.38,-0.1,0),text="W",box=0,opacity=0,color=color.yellow)
label(pos=(0.82,-0.1,0),text="E",box=0,opacity=0,color=color.yellow)
label(pos=(0.75,0.05,0),height=9,text="NE",box=0,color=color.yellow)
label(pos=(0.45,0.05,0),height=9,text="NW",box=0,color=color.yellow)
label(pos=(0.75,-0.25,0),height=9,text="SE",box=0,color=color.yellow)
label(pos=(0.45,-0.25,0),height=9,text="SW",box=0,color=color.yellow)

L1 = label(pos=(-0.5,0.22,0),height=14,text="R",box=0,opacity=0)
L2 = label(pos=(0.0,0.22,0),height=14,text="P",box=0,opacity=0)
L3 = label(pos=(0.6,0.22,0),height=14,text="Y",box=0,opacity=0)

# Main scene objects
scene.select()

# Reference axis (x,y,z)
x_arrow=arrow(color=color.red,axis=(1,0,0), shaftwidth=0.02, fixedwidth=1)
y_arrow=arrow(color=color.yellow,axis=(0,-1,0), shaftwidth=0.02 , fixedwidth=1)
z_arrow=arrow(color=color.green,axis=(0,0,-1), shaftwidth=0.02, fixedwidth=1)
b_z_axis = arrow(color=color.white, axis=(0,0,1),shaftwidth=0.02, fixedwidth=1)

# labels
label(pos=(0,0,0.8),text="10DOF DFRobot IMU test",box=0,opacity=0)
label(pos=(0.5,0,0.8),text="Point FreeIMU's X axis to your monitor then press \"h\"",box=0,opacity=0)
label(pos=(1,0.05,0.8),text="Disable home position by pressing \"n\"",box=0,opacity=0)
input_home = label(pos=(1,-0.8,0.8),text='h', box=0,opacity=0) 

label(pos=(1,0,0),text="X",box=0,opacity=0)
label(pos=(0,-1,0),text="Y",box=0,opacity=0)
label(pos=(0,0,-1),text="Z",box=0,opacity=0)

# IMU object
#platform = box(length=1, height=0.05, width=1, color=color.red)
platform = frame()
p_t=box(frame=platform, pos=(0,0.05,0), length=1, height=0.01, width=.5, color=color.red)
p_b=box(frame=platform,pos=(0,-0.05,0), length=1, height=0.01, width=.5, color=color.blue)
p_ls=box(frame=platform,pos=(0,0,.25), length=1, height=0.1, width=.01, color=color.green)
p_rs=box(frame=platform,pos=(0,0,-.25), length=1, height=0.1, width=.01, color=color.cyan)
p_fs=box(frame=platform,pos=(.495,0,0), length=.01, height=0.1, width=0.5, color=color.yellow)
p_bs=box(frame=platform,pos=(-0.495,0,0), length=.01, height=0.1, width=0.5, color=color.orange)

p_line = box(length=1,height=0.13,width=0.1,color=color.yellow)
plat_arrow = arrow(color=color.green,axis=(1,0,0), shaftwidth=0.06, fixedwidth=1)

# configure the serial connections (the parameters differs on the device you are connecting to)
p = serial.Serial(port=SerialPort,baudrate=38400, timeout = 1)

time.sleep(1)

p.flushInput()    # clean input buffer

p.write("v")
line = p.readline()
time.sleep(5)

while 1 :
    rate(200)

    p.write("z" + str(burst))

    line = p.readline()
    #print line
  
    inputStringArr = string.split(line,",")    # Fields split

    if (len(inputStringArr) > 16):
        try:           
            q1[0] = decodeFloat(inputStringArr[0])
            q1[1] = decodeFloat(inputStringArr[1])
            q1[2] = decodeFloat(inputStringArr[2])
            q1[3] = decodeFloat(inputStringArr[3])
            acc[0] = decodeFloat(inputStringArr[4])
            acc[1] = decodeFloat(inputStringArr[5])
            acc[2] = decodeFloat(inputStringArr[6])
            gyro[0] = decodeFloat(inputStringArr[7])
            gyro[1] = decodeFloat(inputStringArr[8])
            gyro[2] = decodeFloat(inputStringArr[9])
            magn[0] = decodeFloat(inputStringArr[10])
            magn[1] = decodeFloat(inputStringArr[11])	
            magn[2] = decodeFloat(inputStringArr[12])
            temp = decodeFloat(inputStringArr[13])
            press = decodeFloat(inputStringArr[14])
            tnew = decodeFloat(inputStringArr[15])
            dt = tnew - told
            told = tnew
            alt = ((pow((sea_press / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
        except:
            print "Invalid line"
            continue

    #Update data display
    alx.text = str("{:.4f}".format(acc[0]))
    aly.text = str("{:.4f}".format(acc[1]))
    alz.text = str("{:.4f}".format(acc[2]))
    glx.text = str("{:.4f}".format(gyro[0]))
    gly.text = str("{:.4f}".format(gyro[1]))
    glz.text = str("{:.4f}".format(gyro[2]))     
    mlx.text = str("{:.4f}".format(magn[0]))
    mly.text = str("{:.4f}".format(magn[1]))
    mlz.text = str("{:.4f}".format(magn[2]))
    qq0.text = str("{:.4f}".format(q1[0]))
    qq1.text = str("{:.4f}".format(q1[1]))
    qq2.text = str("{:.4f}".format(q1[2]))
    qq3.text = str("{:.4f}".format(q1[3]))
    ltp.text = str("{:.2f}".format(temp))
    lpress.text = str("{:.2f}".format(press))
    ldt.text = str("{:.2f}".format(dt))
    lalt.text = str("{:.2f}".format(alt))

    #out_tmp=str(q1[0]) + "," + str(q1[1]) + "," + str(q1[2]) + "," + str(q1[3]) + "\n"
    #fout.write(out_tmp)
        
    if scene.kb.keys: # event waiting to be processed?
        k = scene.kb.getkey() # get keyboard info
        if len(k) == 1:
            if k == 'h':
                hq = quatConjugate(q1)
            if k == 'n':
                hq = None
            if k == 'e':
                break
            else:
                pass
    
    if (k == 'n'):                                ## use home quaternion
       q[0] = q1[0]
       q[1] = q1[1]
       q[2] = q1[2]
       q[3] = q1[3]
       quaternionToEuler()
       #getYawPitchRollRad()
    elif (k == 'h'):
       q[0] = q1[0]
       q[1] = q1[1]
       q[2] = q1[2]
       q[3] = q1[3]
       prod_q = quatProd(hq, q)
       q[0] = prod_q[0]
       q[1] = prod_q[1]
       q[2] = prod_q[2]
       q[3] = prod_q[3]       
       quaternionToEuler();
       #getYawPitchRollRad()        


    #rotateZ(-Euler[2]); // phi - roll
    #rotateX(-Euler[1]); // theta - pitch
    #rotateY(-Euler[0]); // psi - yaw

    roll = euler[2]
    pitch = euler[1]
    yaw = -euler[0]
    #roll = ypr[1]
    #pitch = ypr[2]
    #yaw = ypr[0]      

    axis=(cos(pitch)*cos(yaw),-cos(pitch)*sin(yaw),sin(pitch)) 
    up=(sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw),sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw),-cos(roll)*cos(pitch))

    
    platform.axis=axis
    platform.up=up
    platform.length=1.0
    platform.width=0.65
    
    plat_arrow.axis=axis
    plat_arrow.up=up
    plat_arrow.length=0.8
    
    p_line.axis=axis
    p_line.up=up
    b_z_axis.axis = 0.5*vector(up)
    
    cil_roll.axis=(-0.2*cos(roll),0.2*sin(roll),0)
    cil_roll2.axis=(0.2*cos(roll),-0.2*sin(roll),0)
    
    cil_pitch.axis=(0.2*cos(pitch),0.2*sin(pitch),0)
    cil_pitch2.axis=(-0.2*cos(pitch),-0.2*sin(pitch),0)
    arrow_course.axis=(0.2*sin(yaw),0.2*cos(yaw),0)
    
    #L1.text = str(float(euler[1])/grad2rad)
    L1.text = str("%3.3f" % (euler[1]/grad2rad))
    L2.text = str("%3.3f" % (euler[2]/grad2rad))
    L3.text = str("%3.3f" % (-euler[0]/grad2rad))
    #L1.text = str("%3.3f" % (yaw/grad2rad))
    #L2.text = str("%3.3f" % (pitch/grad2rad))
    #L3.text = str("%3.3f" % (roll/grad2rad))

    #time.sleep(.05)

p.close
        

