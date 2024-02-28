### inverse kinematics - SPHERICAL

import numpy as np


#link lengths

a1 = float(input("a1 = "))
a2 = float(input("a2 = "))
a3 = float(input("a3 = "))


#joint variable

x0_3 = float(input("x0_3 = "))
y0_3 = float(input("y0_3 = "))
z0_3 = float(input("z0_3 = "))


## inverse knematics solution using graphical method

T1 = np.arctan(y0_3 / x0_3)

r1 = np.sqrt(x0_3**2 + y0_3**2)

r2 = z0_3 - a1

T2 = np.arctan(r2 / r1)

d3 = np.sqrt(r1**2 + r2**2) - a2 - a3

#-----------------------------------------------------#

d1 =  z0_3- a2 - a3

# displaying the joint variable



print("T1 = " , np.around(T1,3))
print("T2 = " , np.around(T2,3))
print("d3 = " , np.around(d3,3))



















