### Inverse Kinematics - SPHERICAL

import numpy as np


# Link Lengths

a1 = float(input("a1 = "))
a2 = float(input("a2 = "))
a3 = float(input("a3 = "))


# Joint Variable

x0_3 = float(input("x0_3 = "))
y0_3 = float(input("y0_3 = "))
z0_3 = float(input("z0_3 = "))


## Inverse Kinematics using graphical method solutions

T1 = np.arctan(y0_3 / x0_3) * (180/np.pi)

r1 = np.sqrt(x0_3**2 + y0_3**2)

r2 = z0_3 - a1

T2 = np.arctan(r2 / r1) * (180/np.pi)

d3 = np.sqrt(r1**2 + r2**2) - a2 - a3

#-----------------------------------------------------#

# Displaying the Joint Variables T1, T2, d3

print("T1 = " , np.around(T1,3))
print("T2 = " , np.around(T2,3))
print("d3 = " , np.around(d3,3))



















