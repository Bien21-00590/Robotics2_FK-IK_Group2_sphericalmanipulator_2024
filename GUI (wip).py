from tkinter import *
from tkinter import messagebox
from tkinter import PhotoImage
import numpy as np
import math
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import spatialmath 
from spatialmath import SE3
import matplotlib

#-----------------------------------------------------------------#

## Create GUI with title

mygui = Tk()
mygui.title("Spherical Manipulator Calculator")
mygui.resizable(True,True) ## (width, height)
mygui.configure(bg ="purple")

## reset functions
def reset():
    a1_E.delete(0, END)
    a2_E.delete(0, END)
    a3_E.delete(0, END)
    T1_E.delete(0, END)
    T2_E.delete(0, END)
    d3_E.delete(0, END)
    X_E.delete(0, END)
    Y_E.delete(0, END)
    Z_E.delete(0, END)

##-----------------------------------------------------------------##

## 1 FORWARD KINEMATICS DEFINE BUTTON

def f_k():

    # link lengths in mm
    a1 = float(a1_E.get())
    a2 = float(a2_E.get())
    a3 = float(a3_E.get())

    # joint variables: is mm if f, is degrees if theta
    T1 = float(T1_E.get()) #20 mm
    T2 = float(T2_E.get()) #30 deg
    d3 = float(d3_E.get()) #-90 deg

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
    H1_2 = np.matrix(np.around(H1_2,3))
    H2_3 = np.matrix(np.around(H2_3,3))
    H0_2 = np.dot(H0_1,H1_2)
    H0_3 = np.dot(H0_2,H2_3)
    H0_3 = np.array(H0_3)


    X0_3 = H0_3[0,3]
    X_E.delete(0,END)
    X_E.insert(0,np.around(X0_3,3))

    Y0_3 = H0_3[1,3] 
    Y_E.delete(0,END)
    Y_E.insert(0,np.around(Y0_3,3))


    Z0_3 = H0_3[2,3]
    Z_E.delete(0,END)
    Z_E.insert(0,np.around(Z0_3,3))


##-----------------------------------------------------------------##

## 2 INVERSE KINEMATICS DEFINE BUTTON
    
def i_k():
    ### inverse kinematics - SPHERICAL

    # LINK LENGTHS
    a1 = float(a1_E.get())
    a2 = float(a2_E.get())
    a3 = float(a3_E.get())

    # JOINT VARIABLE
    x0_3 = float(X_E.get())
    y0_3 = float(Y_E.get())
    z0_3 = float(Z_E.get())


    ## INVERSE KINEMATICS SOLUTION USING GRAPHICAL METHOD

    T1 = np.arctan(y0_3 / x0_3)

    r1 = np.sqrt(x0_3**2 + y0_3**2)

    r2 = z0_3 - a1

    T2 = np.arctan(r2 / r1)

    d3 = np.sqrt(r1**2 + r2**2) - a2 - a3

   
    T1 = T1*(180/np.pi)
    T1_E.delete(0,END)
    T1_E.insert(0,T1)

    T2 = T2*(180/np.pi)
    T2_E.delete(0,END)
    T2_E.insert(0,T2)

    d3_E.delete(0,END)
    d3_E.insert(0,d3)

##-----------------------------------------------------------------##











##-----------------------------------------------------------------##
    
## MAIN GRAPHICAL USER INTERPHASE (GUI)

## link lenghts and joint variable
    
FI = LabelFrame(mygui,text = "Link frames and Joint Variables", font=(5))
FI.grid(row = 0 , column = 0)

#link lenghts and joint variable
a1 = Label(FI,text=("a1 = "), font=(10),bg="white")## fg = "--" for text color
a1_E = Entry(FI, width=5, font=10)
cm1 = Label(FI,text=("cm"), font=(10),bg="white")   

a2 = Label(FI,text=("a2 = "), font=(10),bg="white")
a2_E = Entry(FI, width=5, font=10)
cm2 = Label(FI,text=("cm"), font=(10),bg="white") 

a3 = Label(FI,text=("a3 = "), font=(10),bg="white")
a3_E = Entry(FI, width=5, font=10)
cm3 = Label(FI,text=("cm"), font=(10),bg="white") 

T1 = Label(FI,text=("θ1 = "), font=(10),bg="white")
T1_E = Entry(FI, width=5, font=10)
deg1 = Label(FI,text=("deg"), font=(10),bg="white") 

T2 = Label(FI,text=("θ2 = "), font=(10),bg="white")
T2_E = Entry(FI, width=5, font=10)
deg2 = Label(FI,text=("deg"), font=(10),bg="white") 

d3 = Label(FI,text=("d3 = "), font=(10),bg="white")
d3_E = Entry(FI, width=5, font=10)
cm5 = Label(FI,text=("cm"), font=(10),bg="white") 



a1.grid(row=0,column=0)
a1_E.grid(row=0,column=1)
cm1.grid(row=0,column=2)

a2.grid(row=1,column=0)
a2_E.grid(row=1,column=1)
cm2.grid(row=1,column=2)

a3.grid(row=2,column=0)
a3_E.grid(row=2,column=1)
cm3.grid(row=2,column=2)


T1.grid(row=0,column=3)
T1_E.grid(row=0,column=4)
deg1.grid(row=0,column=5)

T2.grid(row=1,column=3)
T2_E.grid(row=1,column=4)
deg2.grid(row=1,column=5)

d3.grid(row=2,column=3)
d3_E.grid(row=2,column=4)
cm5.grid(row=2,column=5)


## Button frame (forward/reset/inverse)

BF = LabelFrame(mygui,text = "Forward and Inverse", font=(5))
BF.grid(row = 1 , column = 0)

# Buttons

FK = Button(BF, text = "Forward ↓", font = 10, bg = "blue", command=f_k)
RST = Button(BF, text = "Reset ↕", font = 10, bg = "green", command=reset)
IK = Button(BF, text = "Inverse ↑", font = 10, bg = "red", command=i_k)

FK.grid(row=0,column=0)
RST.grid(row=0,column=1)
IK.grid(row=0,column=2)

## position vector

PV = LabelFrame(mygui,text = "Position Vector", font=(5))
PV.grid(row =2 , column = 0)


X = Label(PV,text=("X = "), font=(10),bg="white")## fg = "--" for text color
X_E = Entry(PV, width=5, font=10)
cm6 = Label(PV,text=("cm"), font=(10),bg="white")   

Y = Label(PV,text=("Y = "), font=(10),bg="white")
Y_E = Entry(PV, width=5, font=10)
cm7 = Label(PV,text=("cm"), font=(10),bg="white") 

Z = Label(PV,text=("Z = "), font=(10),bg="white")
Z_E = Entry(PV, width=5, font=10)
cm8 = Label(PV,text=("cm"), font=(10),bg="white") 



X.grid(row=0,column=0)
X_E.grid(row=0,column=1)
cm6.grid(row=0,column=2)

Y.grid(row=1,column=0)
Y_E.grid(row=1,column=1)
cm7.grid(row=1,column=2)

Z.grid(row=2,column=0)
Z_E.grid(row=2,column=1)
cm8.grid(row=2,column=2)

##-----------------------------------------------------------------##

mygui.mainloop() ## always at last




