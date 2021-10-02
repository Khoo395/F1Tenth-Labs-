## Lab2: AEB Impelementation
Automatic Emergency breaking, or **AEB**, is a system that triggers emergency breaking when the possiblity of a collosion is sensed.

To Run the Program, make sure you have installed the F1 Tenth simulator from [here](https://f1tenth.readthedocs.io/en/stable/going_forward/simulator/sim_install.html)

Copy this repo into your workspace src folder  and run the following in your bash 
```bash 
cd <your_ros_workspace>
catkin_make 
roslaunch tiong_hee_roslab tiong_hee_lab2.launch 
```

Press K to control the car using standard WASD control




***
 In this assignment, **AEB** is characterised by **TTC**, which is the Time to collosion if the vehicle follows the current direction. 


Calculations of TTC are based on the Obstacle position from the lidar data and car speed from the odometry messages using simple geometry. If the TTC calculated is found to be smaller than a user-defined threshold value, AEB will kick in to protect the vehicle from collision. 

The challenging part of this assignment is to find a TTC value that doesn't affect driving but also ensures safety/. 

