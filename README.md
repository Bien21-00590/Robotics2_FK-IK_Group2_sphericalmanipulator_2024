# Forward and Inverse Kinematics of a Spherical Manipulator
### Members:
- PL :
- PE : Capuno, Raphael Juno L.
- PS : Arnante, Juan Bienvinido P.
- PQ : Almonte, Ray Ivan C.
- PP :

##  Abstract of the Project

## Introduction
In the times of today, modernized industries have utilized the usage of robots to achieve tasks at a certain and desired efficiency for the sake of various aspects such as mass production or quality assurance but ultimately leads to product fabrications. Robots of today have many applications for a variety of tasks, however, certain robots can only do certain tasks and for this project, **The Spherical Manipulator** or Robot will be the main focus. _**The Spherical Manipulator**_ is a manipulator that is made up of two rotary and one prismatic joint and, from the name itself, uses its links to perform spherical motions from a stationary position or fixed standpoint. To analyze the movement of the said manipulator, we use kinematic equations which have two different approaches. The two different approaches are Forward Kinematics and Inverse Kinematics which are tackled in depth in the Forward and Inverse Kinematics section.

## Degrees of Freedom

The _**Degrees of Freedom**_ refer to a system’s flexibility which in other words, is how freely the manipulator can move and interact with its surroundings. There are two main kinds of Mechanical Manipulators which are the Spatial and the Planar manipulators. Planar manipulators are usually limited to an ideal 3 degrees of freedom whilst Spatial Manipulators have an ideal of 6 degrees of freedom. This is because Planar manipulators can only ideally move at the 3 axes X, Y, and Z being translational movements while Spatial manipulators can, not only move at the three aforementioned axes but also rotate at each which adds up to ideally 6 having 3 translational movements, and 3 rotational movements. The Spherical manipulator is a spatial manipulator meaning that it has up to 6 degrees of freedom. However, this is not always the case as some manipulators can be described as Underactuated, meaning that they have less than the ideal degrees of freedom but they can also be described as Redundant meaning that they have more than the ideal degrees of freedom. Manipulators that don’t have this issue and have the right number of degrees of freedom are called Ideal Manipulators which in this case for the project that focuses on the Spherical Manipulator, it would have 6 degrees of freedom. To find the degrees of freedom of whichever manipulator, we use _**Grubler’s Criterion**_. 

[insert formula]. 

Since the project’s focus is the Spherical Manipulator, we use Grubler’s Criterion for Spatial Manipulators and solve it as follows: 

![DOF Computation](https://github.com/Bien21-00590/Robotics2_FK-IK_Group2_SphericalManipulator_2024/assets/157681561/fefa867e-e71d-4bac-bca9-64932f16e2c9)


This gives us the conclusion that therfore, our Spherical Manipulator has 3 Degrees of Freedom and is an Under-Actuated Spatial Manipulator

## Kinematic Diagram

_**Kinematics**_ is the study or the science of a system’s motion with disregard to other forces that may affect it and this becomes easier to figure out when we use a kinematic diagram. _**Kinematic diagrams give a view of the manipulator with its joints and links connected when their values are set to 0 or when their values are variables. In the case of the Spherical Manipulator, its kinematic diagram looks like this:

![Spherical Manipulator](https://github.com/Bien21-00590/Robotics2_FK-IK_Group2_SphericalManipulator_2024/assets/157681561/77f46ee7-4a8b-4d90-9df6-e365b466135b)


## DH Frame Assignment

_**The Kinematic Diagram**_ helps us isolate the parts of the manipulator so we can have a better view of its general structure but it is incomplete. To complete this, we use the _**Denavit-Hartenberg Notation**_ which is used to analyze and design the manipulator and is used to solve for the _**Forward Kinematics. We will also need to assign the Frames, which in this case is a coordinate system that the manipulator needs to keep track of its supposed location and action. After assigning frames to the manipulator, we then follow the DH (Denavit-Hartenberg) Frame rules which are as follows:

[Insert DH Frame Rules]

Applying these rules to our Spherical Manipulator, it should look like this:

_**Rule 1 Application**_:

![z1_1_Rule 1](https://github.com/Bien21-00590/Robotics2_FK-IK_Group2_SphericalManipulator_2024/assets/157681561/c8becb02-3561-46be-b440-0aecfd97c59a)

_**Rule 2 Application**_:
![z1_2_Rule 2](https://github.com/Bien21-00590/Robotics2_FK-IK_Group2_SphericalManipulator_2024/assets/157681561/f26e992f-d8a2-48ca-b7f0-106f0def36e6)


> Note: Notice that here, we are unable to accomplish Rule No. 2 which is why we will move to Rule No. 3 to fix this issue

_**Rule 3 Application**_:
![Rules_3_Rule 3](https://github.com/Bien21-00590/Robotics2_FK-IK_Group2_SphericalManipulator_2024/assets/157681561/df83ff50-2921-4b74-bd2c-ed8fbe3066a6)


_**Rule 4 Application**_:
![Rules_4_Rule 4](https://github.com/Bien21-00590/Robotics2_FK-IK_Group2_SphericalManipulator_2024/assets/157681561/4f5b0950-0bf1-4fb4-9b57-e2ec77c9bb9f)


Now that we have resolved the issue and made it through all the Rules, the application of the DH Frame Rules has been achieved and the Kinematic Diagram for the Spherical Manipulator is complete.

## DH Parametric Table

After the DH Frame Rules, we can now make what is called the DH Parametric table. _**The DH Parametric Table**_ will help significantly in making our _**Homogeneous Transformation Matrix**_ which will be explained later on. The DH parametric table has columns consisting of parameters and rows related to how many frames there are minus 1. The table will dictate the values of each Homogenous Transformation Matrix from the pair of frames up to the last. The DH Parametric table has a set of parameters that are to be met which are the following for each column:

![DH Parameters](https://github.com/Bien21-00590/Robotics2_FK-IK_Group2_SphericalManipulator_2024/assets/157681561/0096a411-165e-41fa-9d87-ab650cfc3c8f)


By following each of the DH Table parameters, we The DH Parametric table for the Spherical Manipulator will result in such:

![Parametric Table](https://github.com/Bien21-00590/Robotics2_FK-IK_Group2_SphericalManipulator_2024/assets/157681561/5132017b-3ac5-4673-99ca-d1b9bb99e2de)


## Homogeneous Transformation Matrix

_**The Homogeneous Transformation Matrix**_ (HTM) is an essential part of this project because it is the proper combination of the rotation matrixes and position vectors of the Spherical manipulator. HTM describes the rotation and position of the manipulator and this would be the next step after making our Kinematic Diagram. There are two ways of getting the HTM wherein one would be to get the rotation matrixes and the position vectors of each frame with its reference or the more efficient way would be to utilize the DH parametric table that has been made. For the sake of efficiency, the latter will be used. There is a standard for making the HTM such as labeling the HTM. HTMs have superscripts that mean their current frame and subscripts that mean their reference frame and it is as follows:

![HTM Template](https://github.com/Bien21-00590/Robotics2_FK-IK_Group2_SphericalManipulator_2024/assets/157681561/806c5480-e0f4-4e44-94a5-649b98538668)

![HTM Template 2](https://github.com/Bien21-00590/Robotics2_FK-IK_Group2_SphericalManipulator_2024/assets/157681561/52e2dacf-9458-40e6-89d5-3febf58736aa)

Following the standard and with substitution, the HTM should form as such:

![HTM H01](https://github.com/Bien21-00590/Robotics2_FK-IK_Group2_SphericalManipulator_2024/assets/157681561/8d17f52c-8bb4-4f34-9c8b-3efadb73e1dc)

![HTM H12](https://github.com/Bien21-00590/Robotics2_FK-IK_Group2_SphericalManipulator_2024/assets/157681561/ee181190-f801-4d21-b60f-985bba03802f)

![HTM H23](https://github.com/Bien21-00590/Robotics2_FK-IK_Group2_SphericalManipulator_2024/assets/157681561/c8209179-8798-4ec4-83c0-a97c3e5c5939)

After getting the HTMs from frame 0 to frame 3, the next step would be to multiply the HTMs altogether to get the combined HTMs that will be essential for the following sections of computation. The Final HTM is as such:

![HTM H03](https://github.com/Bien21-00590/Robotics2_FK-IK_Group2_SphericalManipulator_2024/assets/157681561/a5b32893-e06d-4fa8-ae14-b20c2c2be614)


Now that we have the final HTM, we should be all set for the Forward Kinematics of our Spherical Manipulator.


## Inverse Kinematics

_**Inverse Kinematics**_ is figuring out joint configurations that produce a desired end-effector position and orientation in robotics. It involves figuring out the joint angles needed to position the end-effector or a robot arm for example, at a particular spot and angle in space. However, inverse kinematics determines the end-effector's position based on joint angles, compared to forward kinematics' calculation procedure. It’s important to have the necessary calculations in tasks like motion planning where having the exact control over the orientation and location of the end-effector is required. For the sake of this project, the graphical method of obtaining the Inverse Kinematics will be used.
 For the Graphical method and the calculation of the inverse kinematics of the Spherical Manipulator, the following will be needed:
 - Link lengths
 - Joint variables
 - Kinematic diagram
 - Pythagorean theorem equations

The following equations were derived as a result from the Graphical method and will all be used in a Python Program to get the values necessary for the Inverse Kinematics of the Spherical Manipulator

![Inverse Kinematics](https://github.com/Bien21-00590/Robotics2_FK-IK_Group2_SphericalManipulator_2024/assets/157681561/70a5d21a-a7f1-47ab-b4f1-467eb312ac9a)


## Forward and Inverse Kinematics GUI Calculator

This midterm project is directed toward a calculator with a user interface that has features connected to the Forward and Inverse kinematics of the Spherical Manipulator of this project. As such, this program was made with the Python programming language and has several things that are important to note for information essential to understanding how the program works. With that stated, here are a few things to know about the code to better understand it.

[Insert Libraries used and each library containment]

Features of the GUI include: 
[Insert the GUI features]


