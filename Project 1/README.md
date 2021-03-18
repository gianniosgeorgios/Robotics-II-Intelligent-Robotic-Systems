# Lab1: Κinematic control of a redundant manipulator

## Abstract

The main purpose of this excerise was to implement the algorithm accordic to this, the end effector of a robotic arm will perform a periodic straight movemet but also avoids moving obstacles.

## Robotic Arm 

The Robotic Arm we study is xArm-7. It is a seven degrees of freedom arm, which brings human-like flexibility, and it is perfect for emerging industries such as AI research, service automation and filming, etc.

## Tools 

To implement this project we needed:

* **ROS environment**: The Robot Operating System (ROS) is an open-source framework that helps researchers and developers build and reuse code between robotics applications
* **Gazebo**: Gazebo is an open-source 3D robotics simulator

## Theorytical Analysis 

### Step 1: Finding Transformation Matrix for previous to next joint

From Denavit–Hartenberg parameters that describe the current robotic Arm, we compute Transformation Matrix for previous to next joint (7 tolal). Using MATLAB code Transfer.m we compute D-H Homogeneous Transformation Matrix given 4 parameters (Rot(x), Tra(x), Tra(z), Rot(z).

### Step 2: Finding Jacobian Matrix for robotic arm

Multyplyng consecutively these tranformation matrixes, we compute Homogeneous Transformation Matrix from start to end joint. Keeping only the last column that corresponds to pe_x, pe_y, pe_z variables we are ready to compute Jacobian using the form:

### Step 3: Path Planning

We define a 3rd order polynomial that describes the position of end efector each time, according to standards. The movement breaks to two tasks

#### Task 1: Straight movemet of End Effector

The general equation that describes this task is: 

#### Task 2: Αvoiding moving obstacles

The general equation that describes this task is: 

## Simulation 
