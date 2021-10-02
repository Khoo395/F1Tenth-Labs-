## Lab4: Reactive Methods 
In this lab, better reactive methods such as **Follow the Gap** and other variants are discussed. Among the variants, I chosed to implement the **Disparities Method**, which is developed by the UNC Champion Team and explained in [this video](https://www.youtube.com/watch?v=ctTJHueaTcY). 


To Run the Program, make sure you have installed the F1 Tenth simulator from [here](https://f1tenth.readthedocs.io/en/stable/going_forward/simulator/sim_install.html)

As this Assignment runs on the Levine_blocked map, You will need to change the map by doing the following: 
1. Navigate to f1tenth_simulator/launch and open "simulator.launch" using any text editor 
2. At Line 7, change "levine.yaml" to "levine_blocked.yaml" and save the file 

Copy this repo into your workspace src folder and run the following in your bash 
```bash 
cd <your_ros_workspace>
catkin_make 
roslaunch tiong_hee_roslab tiong_hee_lab4.launch 
```



https://user-images.githubusercontent.com/74847078/135709374-eb17a76c-fe8e-4a33-8e4e-026695211333.mp4


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
