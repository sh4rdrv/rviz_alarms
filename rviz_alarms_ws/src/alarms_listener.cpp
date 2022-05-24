#include <stdio.h>

#include "teleop_panel.cpp"
#include "std_msgs/Bool.h"

namespace rviz_plugin_tutorials
{

void TeleopPanel::callbackAlarms(const std_msgs::Bool::ConstPtr& msg)
{
  std::cout<<msg->data;
}


int main(int argc, char **argv){
  ros::init(argc, argv, "alarms_listener");
  
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/ros_alarms_topic", 1, &TeleopPanel::callbackAlarms);
  ros::spin();


  return 0;
}

}