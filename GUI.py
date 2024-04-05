import tkinter as tk
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
from PIL import ImageTk, Image
from tkinter import font as tkFont




#-----------------------------------------------------------------#

## Create GUI with title


mygui = Tk()
mygui.title("Spherical Manipulator Calculator")
mygui.resizable(False,False) 
mygui.geometry("800x600") ## (width, height)
mygui.configure(bg ="#424949")

titlefont = tkFont.Font(family = "Ubuntu", weight="bold", size=13)


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

def warning(): 
    messagebox.showinfo("Value Undefined","Unable to compute.")
    condition = True

##-----------------------------------------------------------------##

## 1 FORWARD KINEMATICS BUTTON

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


    # HTM formula
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


## CREATE LINKS
## Robot_variable = DHRobot([RevoluteDH(d,r,alpha,offset=theta,qlim)])
## Robot_variable = DHRobot([PrismaticDH(d=0,r,alpha,offset=d,qlim)])

    Spherical = DHRobot([
    RevoluteDH(a1/100,0,(90/180)*np.pi,(0/180)*np.pi,qlim=[(-90/180)*np.pi/2,(90/180)*np.pi/2]),
    RevoluteDH(0,0,(90/180)*np.pi,(90/180)*np.pi,qlim=[(-90/180)*np.pi/2,(90/180)*np.pi/2]),
    PrismaticDH(0,0,(0/180)*np.pi,(a3+a2)/100,qlim=[0,10]),
    ], name="Spherical") 


    q1 = np.array([T1, T2, d3/100])

    print(Spherical)

    Spherical.plot(q1, limits=[-1,1,-1,1,0,1],block=True,)




##-----------------------------------------------------------------##

## 2 INVERSE KINEMATICS BUTTON
    
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

    if x0_3 == 0 and y0_3==0 or x0_3 == 0:
        warning()

    T1 = np.arctan(y0_3 / x0_3)

    r1 = np.sqrt(x0_3**2 + y0_3**2)

    r2 = z0_3 - a1

    T2 = np.arctan(r2 / r1)


    d3 = (np.sqrt(r1**2 + r2**2) - a2 - a3)
    
   
        ## convert radian to degrees
    TH1 = T1*(180/np.pi)
    TH2 = T2*(180/np.pi)

  
    T1_E.delete(0,END)
    T1_E.insert(0,np.around(TH1,3))


    T2_E.delete(0,END)
    T2_E.insert(0,np.around(TH2,3))

    d3_E.delete(0,END)
    d3_E.insert(0,np.around(d3,3))



## CREATE LINKS
## Robot_variable = DHRobot([RevoluteDH(d,r,alpha,offset=theta,qlim)])
## Robot_variable = DHRobot([PrismaticDH(d=0,r,alpha,offset=d,qlim)])


    Spherical = DHRobot([
    RevoluteDH(a1/100,0,(90/180)*np.pi,(0/180)*np.pi,qlim=[(-90/180)*np.pi/2,(90/180)*np.pi/2]),
    RevoluteDH(0,0,(90/180)*np.pi,(90/180)*np.pi,qlim=[(-90/180)*np.pi/2,(90/180)*np.pi/2]),
    PrismaticDH(0,0,(0/180)*np.pi,(a3+a2)/100,qlim=[0,10]),
    ], name="Spherical") 


    q1 = np.array([T1, T2, d3/100])

    print(Spherical)

    Spherical.plot(q1, limits=[-1,1,-1,1,0,1],block=True,)



##---------------------------------------------------------------------------------------------------------------------------------------##




## MAIN GRAPHICAL USER INTERFACE (GUI)

ttlfont1 = tkFont.Font(family="Stencil STD", weight="bold", size=28)
ttlfont2 = tkFont.Font(family="Curlz MT", weight="bold", size=19)

ttl1 = Label(mygui, text ="Spherical", font="Terminal 26 bold", bg =mygui.cget("bg"), fg = "#FF2D00")
ttl2 = Label(mygui, text ="Manipulator", font="Terminal 26 bold", bg =mygui.cget("bg"), fg = "#C5A9A3")
ttl1.place(x = 190, y = 20)
ttl2.place(x = 145, y = 60)

ttl3 = Label(mygui, text ="Forward and ", font="Terminal 18 bold", bg =mygui.cget("bg"), fg = "#DEDEDE")
ttl4 = Label(mygui, text ="Inverese Kinematics", font="Terminal 18 bold", bg =mygui.cget("bg"), fg = "#DEDEDE")
ttl5 = Label(mygui, text ="Calculator", font="Terminal 18 bold", bg =mygui.cget("bg"), fg = "#DEDEDE")
ttl3.place(x = 415, y = 20)
ttl4.place(x = 415, y = 49)
ttl5.place(x = 415, y = 78)

div = Canvas(mygui, width= 7, height=100, highlightthickness=0)
div.pack(anchor="n", pady = 15)

div.create_line(3,0,3, 120,  width=7, fill="#101010")




## Links And Joint Variable Label
    
FI = LabelFrame(mygui,text = "Link frames and Joint Variables", font=titlefont ,bg="white", padx=5, pady= 5, border=13, borderwidth=12, relief="groove", labelanchor="n")
FI.place(x = 30, y = 150)

#link lenghts and joint variable
a1 = Label(FI,text=("a1 = "), font=(10),bg="white", padx=5 ,pady=3)## fg = "--" for text color
a1_E = Entry(FI, width=5, font=10)
cm1 = Label(FI,text=("cm"), font=(10),bg="white", padx=5, pady=3)

a2 = Label(FI,text=("a2 = "), font=(10),bg="white", padx=5, pady=3)
a2_E = Entry(FI, width=5, font=10)
cm2 = Label(FI,text=("cm"), font=(10),bg="white", padx=5, pady=3)

a3 = Label(FI,text=("a3 = "), font=(10),bg="white", padx=5, pady=3)
a3_E = Entry(FI, width=5, font=10)
cm3 = Label(FI,text=("cm"), font=(10),bg="white", padx=5, pady=3) 

T1 = Label(FI,text=("θ1 = "), font=(10),bg="white", padx=5, pady=3)
T1_E = Entry(FI, width=5, font=10)
deg1 = Label(FI,text=("deg"), font=(10),bg="white", padx=5, pady=3)

T2 = Label(FI,text=("θ2 = "), font=(10),bg="white", padx=5, pady=3)
T2_E = Entry(FI, width=5, font=10)
deg2 = Label(FI,text=("deg"), font=(10),bg="white", padx=5, pady=3)

d3 = Label(FI,text=("d3 = "), font=(10),bg="white", padx=5, pady=3)
d3_E = Entry(FI, width=5, font=10)
cm5 = Label(FI,text=("cm"), font=(10),bg="white", padx=5, pady = 3)



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



## Position Vector Label Frame

PV = LabelFrame(mygui,text ="Position Vector",font=titlefont ,bg="white", padx=5, pady= 5, border=13, borderwidth=10, relief="groove", labelanchor="n", highlightbackground="black", highlightcolor="blue")
PV.place(x = 95, y = 300)


X = Label(PV,text=("X = "), font=(10),bg="gray", )## fg = "--" for text color
X_E = Entry(PV, width=5, font=10)
cm6 = Label(PV,text=("cm"), font=(10),bg="gray")   

Y = Label(PV,text=("Y = "), font=(10),bg="gray")
Y_E = Entry(PV, width=5, font=10)
cm7 = Label(PV,text=("cm"), font=(10),bg="gray") 

Z = Label(PV,text=("Z = "), font=(10),bg="gray")
Z_E = Entry(PV, width=5, font=10)
cm8 = Label(PV,text=("cm"), font=(10),bg="gray") 



X.grid(row=0,column=0, padx=3, ipadx=5, pady = 3)
X_E.grid(row=0,column=1, padx=3, ipadx=5, pady = 3)
cm6.grid(row=0,column=2, padx=3, ipadx=5, pady = 3)

Y.grid(row=1,column=0, padx=3, ipadx=5, pady = 3)
Y_E.grid(row=1,column=1, padx=3, ipadx=5, pady = 3)
cm7.grid(row=1,column=2, padx=3, ipadx=5, pady = 3)

Z.grid(row=2,column=0, padx=3, ipadx=5, pady = 3)
Z_E.grid(row=2,column=1,padx=3, ipadx=5, pady = 3)
cm8.grid(row=2,column=2, padx=3, ipadx=5, pady = 3)


## Button Label Frame(forward/reset/inverse)

BF = LabelFrame(mygui,text = "Forward and Inverse",font=titlefont ,bg="white", padx=5, pady= 5, border=10, borderwidth=13, relief="groove", labelanchor="n")
BF.place(x = 30, y = 460)

# Buttons

FK = Button(BF, text = "Forward ↓", font = 10, bg = "white", command=f_k, padx=5, pady=5, highlightthickness=5,highlightbackground="#ff4040",)
RST = Button(BF, text = "Reset ↕", font = 10, bg = "white", command=reset, padx=5, pady=5, highlightthickness=5,highlightbackground="#273746")
IK = Button(BF, text = "Inverse ↑", font = 10, bg = "white" , command=i_k, padx=5, pady=5, highlightthickness=5,highlightbackground="#FF4040")


FK.pack(side = LEFT , padx=3, pady=3)
RST.pack(side=LEFT, padx=3, pady=3)
IK.pack(side=LEFT , padx=3, pady=3)

## Image Canvas

cv1 = Canvas()
cv1.place(x= 460, y= 120)

model1 = ImageTk.PhotoImage(Image.open("/home/gian/Desktop/Robo_3202/img1.png").resize((250,220)))
img_1 = Label(cv1, image=model1)
img_1.pack(fill="both")

cv2 = Canvas()
cv2.place(x= 460, y= 370)

model2 = ImageTk.PhotoImage(Image.open("/home/gian/Desktop/Robo_3202/img2.png").resize((250,200)))
img_2 = Label(cv2, image=model2)
img_2.pack(fill="both")



mygui.mainloop() ## always at last


