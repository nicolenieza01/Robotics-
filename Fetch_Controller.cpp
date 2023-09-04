#include "Fetch_Controller.hpp"

Fetch_Controller::Fetch_Controller(ros::NodeHandle &nh)
{
    nh_ = nh;
    subscriber_ = nh_.subscribe("/base_scan", 100, &Fetch_Controller::Laser_Scan_Callback, this);
    //TODO: initialize a subscriber that is set to the channel "/base_scan". Set its callback function to be Laser_Scan_Callback



    publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    //TODO: initialize a publisher that is set to the channel "/cmd_vel"


}

void Fetch_Controller::Laser_Scan_Callback(const sensor_msgs::LaserScan::ConstPtr &msg_laser_scan)
{

    int nicole = msg_laser_scan->ranges.size()/ 2; 
    
   int end = nicole + 60;
   int  start = nicole - 60;
   int min = 2147483646;
    
        while (start < end)
        {
            if (msg_laser_scan->ranges[start] >= min)
            {
                min = min;
                
            }
            else {
                min = msg_laser_scan->ranges[start];
            }
            
            start++;
        }




    if (min <= 1.0)
        {
            //turn left
            geometry_msgs::Twist cmd_vel;
            cmd_vel.angular.z = 1.0;
            publisher_.publish(cmd_vel);
            
           
        }
        // forward at .5
        else
        {
            
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0.5;
            publisher_.publish(cmd_vel);
            
        }
    /*TODO: 
    Given the incoming laser scan message, find the minimium distance of the front facing scans
    Hint: The laser scan measuring directly in front of the robot will be the scan at the middle of the array laser scans. 
    So for finding the minimum, we will ONLY consider the 120 laser scans in the middle of the array of laser scans. 
    If the minimum scan in this direction is greater than 1m, drive forward. 
    Otherwise, turn left. 
    */

}
