# Forward and Inverse Kinematics of a Spherica Manipulator
### Members:
- PL :
- PE :
- PS :
- PQ : Almonte, Ray Ivan C.
- PP :

##  **Abstract of the Project**

## Introduction
In the times of today, modernized industries have utilized the usage of robots to achieve tasks at a certain and desired efficiency for the sake of various aspects such as mass production or quality assurance but ultimately leads to product fabrications. Robots of today have many applications for a variety of tasks, however, certain robots can only do certain tasks and for this project, The Spherical Manipulator or Robot will be the main focus. The Spherical Manipulator is a manipulator that is made up of two rotary and one prismatic joint and, from the name itself, uses its links to perform spherical motions from a stationary position or fixed standpoint. To analyze the movement of the said manipulator, we use kinematic equations which have two different approaches. The two different approaches are Forward Kinematics and Inverse Kinematics which are tackled in depth in the Forward and Inverse Kinematics section.

## Degrees of Freedom Description

The Degrees of Freedom refer to a system’s flexibility which in other words, is how freely the manipulator can move and interact with its surroundings. There are two main kinds of Mechanical Manipulators which are the Spatial and the Planar manipulators. Planar manipulators are usually limited to an ideal 3 degrees of freedom whilst Spatial Manipulators have an ideal of 6 degrees of freedom. This is because Planar manipulators can only ideally move at the 3 axes X, Y, and Z being translational movements while Spatial manipulators can, not only move at the three aforementioned axes but also rotate at each which adds up to ideally 6 having 3 translational movements, and 3 rotational movements. The Spherical manipulator is a spatial manipulator meaning that it has up to 6 degrees of freedom. However, this is not always the case as some manipulators can be described as Underactuated, meaning that they have less than the ideal degrees of freedom but they can also be described as Redundant meaning that they have more than the ideal degrees of freedom. Manipulators that don’t have this issue and have the right number of degrees of freedom are called Ideal Manipulators which in this case for the project that focuses on the Spherical Manipulator, it would have 6 degrees of freedom. To find the degrees of freedom of whichever manipulator, we use Grubler’s Criterion. [insert formula]. Since the project’s focus is the Spherical Manipulator, we use Grubler’s Criterion for Spatial Manipulators and solve it as follows: [Insert Computation]

This gives us the answer that our Spherical Manipulator has [insert M value] Degrees of Freedom and is [a/an] [Under-Actuated, Redundant, Ideal] Manipulator

## Kinematic Diagram

Kinematics is the study or the science of a system’s motion with disregard to other forces that may affect it and this becomes easier to figure out when we use a kinematic diagram. Kinematic diagrams give a view of the manipulator with its joints and links connected when their values are set to 0 or when their values are variables. In the case of the Spherical Manipulator, its kinematic diagram looks like this: [insert kinematic diagram]

## DH Frame Assignment

The Kinematic Diagram helps us isolate the parts of the manipulator so we can have a better view of its general structure but it is incomplete. To complete this, we use the Denavit-Hartenberg Notation which is used to analyze and design the manipulator and is used to solve for the Forward Kinematics. We will also need to assign the Frames, which in this case is a coordinate system that the manipulator needs to keep track of its supposed location and action. After assigning frames to the manipulator, we then follow the DH (Denavit-Hartenberg) Frame rules which are as follows: [Insert DH Frame Rules]

Applying these rules to our Spherical Manipulator, it should look like this:

[Insert Rule 1 application to Spherical Manipulator]

[Insert Rule 2 application to Spherical Manipulator]

Note: Notice that here, we are unable to accomplish Rule No. 2 which is why we will move to Rule No. 3 to fix this issue

[Insert Rule 3 application to Spherical Manipulator]

[Insert Rule 4 application to Spherical Manipulator]

Now that we have resolved the issue and made it through all the Rules, the application of the DH Frame Rules has been achieved and the Kinematic Diagram for the Spherical Manipulator is complete.

## Homogeneous Transformation Matrix

The Homogeneous Transformation Matrix is an essential part of this project as this would be the next step after making our Kinematic Diagram but before we can make the Homogeneous transformation matrix, we must first get the Rotation matrix which helps us indicate the rotational movement of a manipulator in the form of a 3 by 3 matrix that consists of the relative rotations between frames and another essential part of the Homogeneous Transformation Matrix is the Position Vector which then indicates the displacement of the manipulator in the form of a 1 by 3 matrix that consists of its X, Y, and Z positions. In the flowing solutions, we will get the Rotation matrix of the Spherical manipulator.

[Insert Rotation Matrix process]

After these solutions, we will then get the Position vector of the Spherical Manipulator which is shown in the following solution.

[Insert Position Vector process]

With this, we can now build our Homogeneous Transformation Matrix with the following process.

[Insert HTM process and result in H0_3]

And thus, our Homogeneous Transformation Matrix is complete.
