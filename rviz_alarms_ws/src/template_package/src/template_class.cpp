#include <template_package/template_class.h>

TemplateClass::TemplateClass()
{
  /* I decided to use a pointer to my ros node no I need to allocate memory
   * also,  I told it to use the the node's namespace (specified by ~ just like linux directories)
   * basically all that means is all broadcast topics will be sent as "node_name/topic_name" unless
   * otherwise specified. */
  ros_node_ptr_.reset(new ros::NodeHandle("~"));
}

void TemplateClass::spin_once(){
  ROS_INFO("I don't do anything yet.  Jake is going to make me send a message");

  // after we spin once we need to tell ros to do the same
  ros::spinOnce();
}

void TemplateClass::spin(){
  ros::Rate loop_rate(1); // in hz
  while (ros::ok())
  {
    spin_once();
    loop_rate.sleep();
  }
}
