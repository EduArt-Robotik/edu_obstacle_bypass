# edu_obstacle_bypass
ROS Node to manoeuvre the robot around an obstacle as soon as it is detected.

![First Approach](img/test.gif)

### The Lidars Orientation

Using the Slamtec Lidar RPILidar A2M8, the orientation of its coordinate system is assumed by the following figure. Thus, obstacles in the robots's frontfacing direction should be represented by scan[0].

![Lidar Orientation](img/rpilidar_orientation.png)