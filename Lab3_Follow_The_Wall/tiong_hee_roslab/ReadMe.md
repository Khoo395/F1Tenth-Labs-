## Lab3: Follow the Wall Method 
In this lab, we started to program the car to navigate autonomously using a very naive method - Follow the wall . The idea is the car will be on the correct path as long as it maintains a certain distance with one side of the track boundary.


To Run the Program, make sure you have installed the F1 Tenth simulator from [here](https://f1tenth.readthedocs.io/en/stable/going_forward/simulator/sim_install.html)

Copy this repo into your workspace src folder and run the following in your bash 
```bash 
cd <your_ros_workspace>
catkin_make 
roslaunch tiong_hee_roslab tiong_hee_lab3.launch 
```


***

## Implementation

The idea of "following the wall" is implemented along with a PID controller. This allows the car to respond quickly to changes and reaches the steady state effectively. 

The distance of the car is calculated through the lidar data and simple trigonometry. A detailed derivation can be found in the lab Handout. 

The PID controller is implemented through an error term, which represents the distance between the desired path and current car location. 

### Proportional Term
Implementation of the P-term is relatively easy, as it will be simply proportional to the negative of error term. 

### Integral Term 
Encountered problems when implementing, as the constant sum of errors makes the motion of the car unpredictable and crashes easily. As a result, this is not implemented. 

### Differential Term
This term is implemented by observing the change of the error term with respect to time, and is meant to prevent overshooting. However, in practise, this is very hard to implement due to the inconsistent loop run time which differs upto a factor of hundreds. As a result, the value also fluctuates following the scale and is not usable.

To solve this problem, I set a minimum time block of 100ms, and the differential term will only be calculated for each period of this time block. This ensures the stability but sacrificed quick response ability. 