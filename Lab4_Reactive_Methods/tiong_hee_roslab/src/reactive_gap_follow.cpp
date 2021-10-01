#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <memory>
// TODO: include ROS msg type headers and libraries

class Controller{
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    
    ros::Subscriber LidarSub; 
    ros::Subscriber OdomSub; 
    ros::Publisher AckermannPub; 
    ackermann_msgs::AckermannDriveStamped ackermann;
    double threshold = 1.0; 
    double car_width = 0.8;
    double k_speed = 3.5; 
    double k_speed2 = 1.7 ; 
    double k_angular_speed = 0.02; 
    double  min_ttc = 0.15; 
    float speed_x;
    int CAA; // Collision Avoid Angle 
    
    void GetSpeedCallback(nav_msgs::OdometryConstPtr odom_msg_ptr)
    {
        speed_x = odom_msg_ptr->twist.twist.linear.x; 
    }

        void Ttcalculation(sensor_msgs::LaserScanConstPtr scan_msg_ptr)
    {
        float distance;
        float velocity; 
        float current_angle =0; 
        float ttc; 
        

        for(int i=270; i < 405; i++){
            distance = scan_msg_ptr->ranges[i];
            velocity = speed_x * cos(current_angle);
            ttc = abs(distance/(velocity+0.00001)); 
            current_angle = i *0.005823;
            if(ttc < min_ttc)
            {
                CAA = 30; 
                //ROS_INFO("TTC Left: %f", ttc); 
                break;
            }
        }

        for(int i=675; i < 810; i++){
            distance = scan_msg_ptr->ranges[i];
            velocity = speed_x * cos(current_angle);
            ttc = abs(distance/(velocity+0.00001)); 
            current_angle = i *0.005823;
            if(ttc < min_ttc)
            {
                //ROS_INFO("TTC right: %f", ttc); 
                CAA = -30; 
                break;
            }
        }
        
    }

    void FindCliff(sensor_msgs::LaserScanConstPtr scan_msg_ptr)
    {  
        Ttcalculation(scan_msg_ptr);
        int furthest_point = 0;
        double furthest_distance = 0; 
        std::vector <int> restricted_Points; 
        std::vector <int> Cliffs;
        std::vector <int> Obstacle_Direction;
        std::vector <double> Cliff_dist; 
        double ranges [1080];
        for(int i = 0; i<1080; i++)
        {  // if(scan_msg_ptr->ranges[i] < 35){
            ranges[i] = scan_msg_ptr->ranges[i];
          //  }
           // else
           // {ranges[i] = 2;} // Not needed for levine blocked 
        }   
            
        for(int i = 135; i<945; i++)
        {
            double current_distance = ranges[i];
          /*  if (current_distance > furthest_distance)
            {
                furthest_point = i; 
                furthest_distance = scan_msg_ptr->ranges[i];
            }*/
            if (abs(current_distance - ranges[i-1]) > threshold)
            {//ROS_INFO("Cliff Detected at %d", i);
                if(current_distance > ranges[i-1]){
                    Cliffs.push_back(i-1);
               
                    Cliff_dist.push_back(ranges[i-1]);
                  
                    Obstacle_Direction.push_back(2); // 2 represent right, 

                }
                else{
                    Cliffs.push_back(i); 
             
                    Cliff_dist.push_back(ranges[i]);
             
                    Obstacle_Direction.push_back(1); // 1 represent left, 
                }

            }
        }

        for(int i = 0; i < Cliffs.size(); i++)
        {  //ROS_INFO("Test");
            int n_readings; 
            n_readings = ceil(0.5*car_width/ (Cliff_dist[i] * 0.005823));
            if(Obstacle_Direction[i] == 1){ 
                for(int j=0; j<n_readings; j++){
                    if(Cliffs[i]+ j < 1080){
                    ranges[Cliffs[i]+ j ] = 0; 
                    }//Cliff_dist[i]
                    //ROS_INFO("range_no: %d, range: %f", Cliffs[i]+ j , ranges[Cliffs[i]+ j ]);
                }
            }
            if(Obstacle_Direction[i] == 2){ 
                for(int j=0; j<n_readings; j++){
                    if (Cliffs[i] - j > 0){
                    ranges[Cliffs[i]- j ] = 0; 
                    }//Cliff_dist[i]
                    //ROS_INFO("range_no: %d, range: %f", Cliffs[i]- j , ranges[Cliffs[i]- j ]);
                }
            
            }
            else{//ROS_INFO("SKIP: %d", Obstacle_Direction[i]);
            }

            
        }
        for(int i = 135;i<945;i++){
                double current_distance = ranges[i];
            if (current_distance > furthest_distance)
            {
                furthest_point = i; 
                furthest_distance = ranges[i];
            }
            }
        
      //ROS_INFO("furthest point %d, furthest_range: %f",furthest_point, furthest_distance);
        
        PidController(furthest_point,furthest_distance, ranges); 

        }

    void PidController(int furthest_point, double furthest_distance, double ranges[1080]){ 
        //ackermann.drive.steering_angle = k_angular_speed*(furthest_point- 540) * pow(furthest_distance, -0.5); 
        ackermann.drive.steering_angle = (furthest_point- 540 + CAA)* 0.005823; 
        CAA = 0; 
       //  if (ackermann.drive.steering_angle < (-M_PI/8)){ackermann.drive.steering_angle = -M_PI/8;}; //Not needed for Levine blocked 
      if (abs(ackermann.drive.steering_angle < M_PI/5 )){ackermann.drive.steering_angle =  ackermann.drive.steering_angle/ 5 ;}
      if(furthest_distance > 30 && ackermann.drive.steering_angle < M_PI/10){ackermann.drive.speed = 10; }
        else{ackermann.drive.speed = k_speed* abs(k_speed2 - ackermann.drive.steering_angle);} 
        ackermann.header.frame_id = "laser"; 
        ackermann.header.stamp = ros::Time::now();
        AckermannPub.publish(ackermann); 
        //ROS_INFO("furthest point : %d, furthest_distance : %f, steering: %f", furthest_point, furthest_distance, ackermann.drive.steering_angle);
    }
    
    // TODO: create ROS subscribers and publishers

public:
    Controller() {
        n = ros::NodeHandle();

        LidarSub = n.subscribe("scan", 1000, &Controller::FindCliff, this);
        AckermannPub = n.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1000);
        OdomSub = n.subscribe("odom", 10, &Controller::GetSpeedCallback, this);
        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages
        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        */

        // TODO: create ROS subscribers and publishers
        
    }


};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "Controller");
    Controller cn;
    ros::spin();
    return 0;
}