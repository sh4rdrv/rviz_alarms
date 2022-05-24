#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "ros_alarms_topic");

  ros::NodeHandle n;

  ros::Publisher ros_alarms_pub = n.advertise<std_msgs::Bool>("ros_alarms_topic", 1);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::Bool msg;

    msg.data = false;

    ros_alarms_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}