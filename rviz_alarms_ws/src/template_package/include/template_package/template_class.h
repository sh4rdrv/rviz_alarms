#ifndef TEMPLATE_CLASS_H
#define TEMPLATE_CLASS_H

#include <ros/ros.h>

class TemplateClass
{
public:
  TemplateClass();
  /*!
   * \brief spin is degined to run the node continuously
   */
  void spin();
  /*!
   * \brief spin_once is intended to run the loop just once
   */
  void spin_once();

protected:
  ros::NodeHandlePtr ros_node_ptr_;

};

#endif // TEMPLATE_CLASS_H
