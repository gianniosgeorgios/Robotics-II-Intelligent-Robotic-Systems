# Lab2: Wall following performance of a mobile autonomus robot

![Screenshot from 2021-03-19 14-56-02](https://user-images.githubusercontent.com/50829499/111783933-6fc34300-88c3-11eb-82e5-ba4e012e1564.png)

## Abstract 

The main purpose of this lab excerise was to implement wall following algorithm for an autonomus mobile robot. This kind of movement is based only to sensor fusion.

## Mobile Robot 

The mobile robot that we studied was differential-drive and is equipped with:
* Two wheels with diameter equal to 20cm
* Five ultrasonic sensors for distance measurement
* One IMU  (Inertial Measurement  Unit) 9 degrees' of freedom for linear- angular velocity/acceleration measurement


## Algorithm Description

The algorithm is simply transitions between three or even better two states:

### State 1: Wall Approach (One time, from start position)

![Screenshot from 2021-03-19 14-56-27](https://user-images.githubusercontent.com/50829499/111783975-810c4f80-88c3-11eb-9fd0-3e55f4b934f7.png)

As long as we measure the value of front sensor (>= 0.5 cm) , we have:

* Constant linear velocity
* Zero angular velocity


### State 2: Turn to a corner 

![Screenshot from 2021-03-19 14-56-33](https://user-images.githubusercontent.com/50829499/111783993-87023080-88c3-11eb-8ccd-165068f7277f.png)

As long as we don't reach some target distances (described below), we have:

* Zero linear velocity
* Constant angular velocity

![Screenshot from 2021-03-19 15-00-39](https://user-images.githubusercontent.com/50829499/111784280-ebbd8b00-88c3-11eb-8899-f10429814947.png)

### State 3: Wall Following 

![Screenshot from 2021-03-19 14-56-40](https://user-images.githubusercontent.com/50829499/111784014-8d90a800-88c3-11eb-9761-ea4d8ea57ee0.png)


Using an PD controller, we achieve straight forward movement alongside the wall. The PD controller is described by the below diagram

![Screenshot from 2021-03-19 14-56-48](https://user-images.githubusercontent.com/50829499/111784053-9a150080-88c3-11eb-913f-b061a842e790.png)


## Simulation 

It is obvious from below plots that during the wall following the mobile robot moves alongside the wall with little (negligible) variations. As we see the distances that are measured from sensors are constant:

![Screenshot from 2021-03-19 14-57-00](https://user-images.githubusercontent.com/50829499/111784209-cb8dcc00-88c3-11eb-917c-567b0e4b9b6c.png)





