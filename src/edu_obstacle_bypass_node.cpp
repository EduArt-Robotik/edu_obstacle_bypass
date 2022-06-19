/**
 * @author Antonello Pastore
 * @date 15.06.2022
 * @brief ROS Node to manoeuvre the robot around an obstacle as soon as it is detected.
 **/
#include "ros/ros.h"                // ROS source header
#include "sensor_msgs/LaserScan.h"  // Message type for receiving scan data
#include <geometry_msgs/Twist.h>    // Message type for sending cmd_vel
#include <math.h>                   // For common mathematical operations

class ScanListener 
{
  public:
    double range_ahead;

    void scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg);
};

void ScanListener::scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  range_ahead = scan_msg->ranges[0];
  // range_ahead = scan_msg->ranges[msg.ranges.size() / 2];
}

int main(int argc, char **argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "edu_obstacle_bypass_node");

  // Create a ROS node handle
  ros::NodeHandle n;

  // Create a ScanListener object
  ScanListener scan_listener;

  // Create a publisher for cmd_vel messages
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/vel/teleop", 1);

  // Create a subscriber for scan data
  ros::Subscriber scan_sub = n.subscribe("/front/scan", 1000, &ScanListener::scanCallback, &scan_listener);

  ros::Time state_change_time = ros::Time::now();                       // Initialize the time of the last state change
  ros::Rate loop_rate(10);                                              // Set the loop rate to 10 Hz

  scan_listener.range_ahead = 0.0;                                      // Initialize the range ahead to 0.0
  float min_range_ahead = 1.2;                                          // Set the minimum range ahead in meters
  const float DEG_TO_RAD = M_PI / 180.0;                                // Convert degrees to radians
  bool obstacle_detected = false;                                       // Initialize the obstacle detected flag to false

  float angular_velocity = 0.5;                                         // Set the angular velocity in rad/s
  float linear_velocity = 0.5;                                          // Set the linear velocity in 0.0 m/s

  float min_spin_deg = 90.00 * DEG_TO_RAD;                              // Set the minimum spin angle in degrees
  float max_spin_deg = 160.00 * DEG_TO_RAD;                             // Set the maximum spin angle in degrees

  float min_spin_duration = min_spin_duration / angular_velocity;       // Set the minimum spin duration in seconds
  float max_spin_duration = max_spin_duration / angular_velocity;       // Set the maximum spin duration in seconds
  float spin_duration = 0.0;                                            // Initialize the spin duration to 0.0 seconds
  float spin_direction = 1.0;                                           // Initialize the spin direction to 1 (clockwise)

  while (ros::ok())
  {
    // Check if the obstacle is detected
    if (!obstacle_detected)
    {
      std::cout << "Driving forward. Range ahead [m]: " << scan_listener.range_ahead << std::endl;
      if (scan_listener.range_ahead < min_range_ahead )
      {
        obstacle_detected = true;
        // Set the spin_duration to a random value between min_spin_duration and max_spin_duration [s]
        spin_duration = ((float)rand() / (float)RAND_MAX * (max_spin_duration - min_spin_duration)) + min_spin_duration;
        state_change_time = ros::Time::now() + ros::Duration(spin_duration);
      }      
    }
    else
    {
      std::cout << "Obstacle detected & turning. Range ahead [m]: " << scan_listener.range_ahead << std::endl;
      if (ros::Time::now() > state_change_time)
      {
        obstacle_detected = false;
        state_change_time = ros::Time::now();
      }
    }
    
    geometry_msgs::Twist msg; // Create a message of type geometry_msgs/Twist to send cmd_vel data to the robot

    if (!obstacle_detected)
    {
      msg.linear.x = linear_velocity;
      msg.angular.z = 0.0;
    }
    else
    {
      msg.linear.x = 0.0;
      msg.angular.z = angular_velocity;
    }

    vel_pub.publish(msg); // Publish the message
    ros::spinOnce();      // Spin once to handle callbacks
    loop_rate.sleep();    // Sleep for the rest of the cycle
  }
}