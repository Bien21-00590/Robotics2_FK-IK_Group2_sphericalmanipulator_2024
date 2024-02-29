import numpy as np
import math

# link lengths in mm
a1 = float(input("a1 = "))
a2 = float(input("a2 = "))
a3 = float(input("a3 = "))

# joint variables: is mm if f, is degrees if theta
T1 = float(input("T1 = ")) #20 mm
T2 = float(input("T2 = ")) #30 deg
d3 = float(input("d3 = ")) #-90 deg

# degree to radian
T1 = (T1/180.0)*np.pi
T2 = (T2/180.0)*np.pi

# Parametric Table (theta, alpha, r, d)
PT = [[T1,(90.0/180.0)*np.pi,0,a1],
      [T2 + (90.0/180.0)*np.pi,(90.0/180.0)*np.pi,0,0],
      [0,0,0,a2 + a3 + d3]]


# HTM formulae
i = 0
H0_1 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
        [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
        [0,0,0,1]]

i = 1
H1_2 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
        [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
        [0,0,0,1]]

i = 2
H2_3 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
        [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
        [0,0,0,1]]

H0_1 = np.matrix(np.around(H0_1,3))
print("H0_1= ")
print(H0_1)

H1_2 = np.matrix(np.around(H1_2,3))
print("H1_2= ")
print(H1_2)

H2_3 = np.matrix(np.around(H2_3,3))
print("H2_3= ")
print(H2_3)

H0_2 = np.dot(H0_1,H1_2)
H0_3 = np.dot(H0_2,H2_3)
print("H0_3= ")
print(np.matrix(np.around(H0_3,3)))
