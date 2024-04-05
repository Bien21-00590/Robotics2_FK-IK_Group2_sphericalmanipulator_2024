import roboticstoolbox as rtb
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
from roboticstoolbox.backends.PyPlot import PyPlot
import math
import spatialmath
from spatialmath import SE3


## SPHERICAL MANIPULATOR MODEL

# link lengths in mm
a1 = float(input("a1 = "))
a2 = float(input("a2 = "))
a3 = float(input("a3 = "))



##  PRISMATIC LINK LIMIT mm - m
lml = float(input("lml = "))


## CREATE LINKS
## Robot_variable = DHRobot([RevoluteDH(d,r,alpha,offset=theta,qlim)])
## Robot_variable = DHRobot([PrismaticDH(d=0,r,alpha,offset=d,qlim)])


Spherical = DHRobot([
    RevoluteDH(a1/100,0,(90/180)*np.pi,(0/180)*np.pi,qlim=[-np.pi/2,np.pi/2]),
    RevoluteDH(0,0,(90/180)*np.pi,(90/180)*np.pi,qlim=[-np.pi/2,np.pi/2]),
    PrismaticDH(0,0,(0/180)*np.pi,(a3+a2)/100,qlim=[0,lml/100]),
], name="Spherical") 

print(Spherical)

Spherical.teach([0,0,0])

