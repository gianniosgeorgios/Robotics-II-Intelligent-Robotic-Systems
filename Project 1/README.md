# Lab1: Κinematic control of a redundant manipulator

![Screenshot from 2021-03-19 00-00-05](https://user-images.githubusercontent.com/50829499/111703286-1c5edf80-8846-11eb-8331-29fb89acc6c8.png)

## Abstract

The main purpose of this excerise was to implement an algorithm accordic to this, the end effector of a robotic arm will perform a periodic straight movemet but also avoids moving obstacles (red and green columns in the above image). Click here to watch [demo](https://user-images.githubusercontent.com/50829499/111825598-4ff64400-88f0-11eb-92ef-dbccfc999726.mp4)



## Robotic Arm 

The Robotic Arm we study is [xArm-7](https://www.youtube.com/watch?v=xaOWXSACNXs). It is a seven degrees of freedom arm, which brings human-like flexibility, and it is perfect for emerging industries such as AI research, service automation and filming, etc.

## Tools 

To implement this project we needed:

* **ROS environment**: The Robot Operating System ([ROS](https://www.google.com/search?channel=fs&client=ubuntu&q=ROS)) is an open-source framework that helps researchers and developers build and reuse code between robotics applications
* **Gazebo**: [Gazebo](http://gazebosim.org/tutorials?tut=ros_overview) is an open-source 3D robotics simulator

## Theorytical Analysis 

### Step 1: Finding Transformation Matrix for previous to next joint

![Screenshot from 2021-03-18 23-59-54](https://user-images.githubusercontent.com/50829499/111703337-2da7ec00-8846-11eb-90d5-f33260ca6138.png)

From Denavit–Hartenberg parameters (each line of above matrix), we compute Transformation Matrix for previous to next joint (7 tolal). This became easier inserting 4 parameters `Rot(x), Tra(x), Tra(z), Rot(z)` to MATLAB function icluded to `Transfer.m` file.

### Step 2: Finding Jacobian Matrix for robotic arm

Multyplyng consecutively these tranformation matrixes, we compute **Homogeneous Transformation Matrix** from start to end joint. Keeping only the last column that corresponds to `pe_x, pe_y, pe_z` variables, we are ready to compute Jacobian using the form:

![Screenshot from 2021-03-18 23-59-42](https://user-images.githubusercontent.com/50829499/111703311-24b71a80-8846-11eb-8e42-41d0dcf07c15.png)

This became easier using matlab code `Jacobian.m`

The kinematic analysis (Jacobian & Homogeneous Transformation Matrices computation) has been inluded to `kinematics.py` file. This code is usefull for `controller.py`,which is responsible for Path Planning and Two Different Tasks.

### Step 3: Path Planning

We define a 3rd order polynomial that describes the position of end efector each time, according to standards. The movement breaks to two tasks

#### Task 1: Straight movemet of End Effector

The general equation that describes this task is: 

![Screenshot from 2021-03-18 23-43-57](https://user-images.githubusercontent.com/50829499/111701964-40b9bc80-8844-11eb-8be7-a43a6c2be1c5.png)


#### Task 2: Αvoiding moving obstacles

The general equation that describes this task is: 

![Screenshot from 2021-03-18 23-44-30](https://user-images.githubusercontent.com/50829499/111701875-2089fd80-8844-11eb-958d-df93645d99d3.png)

*For further information,about these tasks, check `Report.pdf` *

## Simulation 

As I said before the analysis about controlles has been included to `controller.py` file. 

Starting Gazebo simulation we notice that the end efector performs a straight movement (constant values y-z coordinates and "smooth" periodic movement of x-cordinate (described by cosine function)): 

![Screenshot from 2021-03-18 23-54-13](https://user-images.githubusercontent.com/50829499/111702780-77dc9d80-8845-11eb-91ef-8ba2202a7716.png)
