## Lab4: Reactive Methods 
In this lab, better reactive methods such as **Follow the Gap** and other variants are discussed. Among the variants, I chosed to implement the **Disparities Method**, which is developed by the UNC Champion Team and explained in [this video](https://www.youtube.com/watch?v=ctTJHueaTcY). 

The lab Handouts are obtained from 


To Run the Program, make sure you have installed the F1 Tenth simulator from [here](https://f1tenth.readthedocs.io/en/stable/going_forward/simulator/sim_install.html)

Copy this repo into your workspace src folder and run the following in your bash 
```bash 
cd <your_ros_workspace>
catkin_make 
roslaunch tiong_hee_roslab tiong_hee_lab4.launch 
```


***

## Implementation

As described in the video above,**Disparities method** works by detecting disparities and extending the boundaries of the wall by 1/2 car's width wherever a disparity is detected. This can effectively prevent collosion from the vanila **Follow the Gap**.  

### Disparities Detection
Done by comparing the difference of neighbouring lidar reading to a threshold value. If the difference is larger than the threshold, a disparity will registered. 

### Obstacle Register
For every disparity registered, the "Obstacle" will be extended 1/2 car's width. This is done by setting the lidar data covering the extended obstacles to the range where disparity is detected.  

### Object Orientation
As same with **Follow the Gap**, the car always orientate towards the furthest point sensed from the lidar. As disparities are all detected, the car can pass through corners safely. 

### Anti Collosion Correction 
As the car is set to run as fast as possible, it tends to collide with the outer side track boundary when passing through corner. To solve this, I used a concept similiar to AEB in lab 2. Whenever a collosion is sense to be imminent, a correction angle will be added to steer the car to safety. 