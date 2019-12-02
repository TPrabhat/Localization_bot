#include "ros/ros.h"
#include "chase_it/DriveToTarget.h"
#include <sensor_msgs/LaserScan.h>


// This callback function continuously executes and reads the scan data
void scan_callback(const sensor_msgs::LaserScan scan)
{
        for(int i = 0; i < 100; i++) 
        {
	  ROS_INFO_STREAM("Range min : " + std::to_string(scan.range_min));
	  ROS_INFO_STREAM("RANGE MAX : " + std::to_string(scan.range_max));
          ROS_INFO_STREAM("Angle min : " + std::to_string(scan.angle_min));
	  ROS_INFO_STREAM("Angle MAX : " + std::to_string(scan.angle_max));
	  ROS_INFO_STREAM("Angle increment : " + std::to_string(scan.angle_increment));
          ROS_INFO_STREAM("Time increment : " + std::to_string(scan.time_increment));
	  ROS_INFO_STREAM("Scan time : " + std::to_string(scan.scan_time));
          for(int i = 0; i < sizeof(scan.ranges)/sizeof(scan.ranges[0]); i++)
          {
	    ROS_INFO_STREAM("range size : " + std::to_string(scan.ranges[i]));
          }
        }
}


int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "scan");
    ros::NodeHandle n;

    ros::Subscriber sub1 = n.subscribe("/scan", 10, scan_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

if(width <= (img.step)/3)
          {
            //move left
      //      ROS_INFO_STREAM("Moving left");
            drive_robot(0.5, -0.4);
            closing_in = true;
            break;
          }
          else if(width >= (2*img.step)/3)
          {
            // move right
     //       ROS_INFO_STREAM("Moving right");
            drive_robot(0.5, 0.4);
            closing_in = true;
            break;
          }
          else
          {
            // move fwd
     //       ROS_INFO_STREAM("Moving straight");
            drive_robot(1.0, 0.0);
            closing_in = true;
            break;
          }

	  velocity = 2.63 - (1.9*height)/600;
          direction = (-1.0 + (2.0/2400)*width)*((img.height-height)/200);
          closing_in = true;
	  drive_robot(velocity, direction);
          break;


