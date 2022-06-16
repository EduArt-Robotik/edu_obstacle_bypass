/**
 * @author Antonello Pastore
 * @date 15.06.2022
 * @brief ROS Node to manoeuvre the robot around an obstacle as soon as it is detected.
 **/
#include "ros/ros.h"                // ROS source header
#include "sensor_msgs/LaserScan.h"  // Message type for receiving scan data
#include <geometry_msgs/Twist.h>    // Message type for sending cmd_vel
#include <math.h>                   // For common mathematical operations

ros::Publisher vel_pub;     // Initialize the Publisher
ros::Subscriber scan_sub;   // Initialize the Subscriber

/**
 * @brief Callback function for the laser scan data
 * @param msg Laser scan data.
 */
void scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  std::vector<float> scan = scan_msg->ranges;   // Collecting the meassured ranges from the arriving scan data
  geometry_msgs::Twist msg;                     // Initialization of the turning angle

  int front_index = 1;                          // Initialize the Index of the measurement in front of the robot
  float front_range = 0.5;                      // Initialize the maximum range [m] before the obstacle for triggering the manoeuvre
  
  /**
    * The publish() function is how you send messages. The parameter
    * is the message object. The type of this object must agree with the type
    * given as a template parameter to the advertise() call.
    */

  if (scan[front_index] < front_range)
  {
    int dir = 1;                                                      // Initialize the direction of the manoeuvre
    float ang_vel = 0.3 * dir;                                        // Initialize the angular velocity [rad/s]
    float target_angle = (((double) rand() / (RAND_MAX)) * M_PI_2);   // Initialize the target angle [rad]
    float time = target_angle / ang_vel;                              // Initialize the publishing time [s]
    std::cout << "pub_time: " << pub_time << std::endl;
    std::cout << "scan[front_index]: " << scan[front_index] << std::endl;

    msg.linear.x = 0;
    msg.angular.z = ang_vel;
    
    ros::Time beginTime = ros::Time::now();
    ros::Duration pub_time = ros::Duration(time); // [s]
    ros::Time endTime = beginTime + pub_time;
    while(ros::Time::now() < endTime )
    {
        vel_pub.publish(msg);
        ros::Duration(0.1).sleep();
    }
  }
  else
  {
    msg.linear.x = 0.2;
    msg.angular.z = 0;
    vel_pub.publish(msg);
  }
}

int main(int argc, char **argv)
{
  // This must be called before anything else ROS-related
  ros::init(argc, argv, "edu_obstacle_bypass_node");

  // Create a ROS node handle
  ros::NodeHandle n;

  /** PUBLISHER
  * The advertise() function is how you tell ROS that you want to
  * publish on a given topic name: the generated twist-msg to move your robot.
  * Tell the master that we are going to be publishing every 10ms the message of type geometry_msgs/Twist on the topic /cmd_vel.
  */
  vel_pub = n.advertise<geometry_msgs::Twist>("/vel/teleop", 1);

  /** SUBSCRIBER
  * The subscribe() call is how you tell ROS that you want to receive messages
  * on a given topic.  Messages are passed to a callback function, here
  * called  scanCallback to determine the target angle.
  */
  scan_sub = n.subscribe("front/scan", 1000, scanCallback);

  // Calling the Callbacks (once)
  ros::spin();
}