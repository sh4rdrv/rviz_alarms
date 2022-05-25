/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H

#ifndef Q_MOC_RUN
#include <QColor>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>

# include <ros/ros.h>

# include <rviz/panel.h>
#include "std_msgs/Bool.h"

#endif

class QLineEdit;

namespace rviz_plugin_tutorials
{

class TeleopPanel: public rviz::Panel
{

Q_OBJECT

public:

  TeleopPanel( QWidget* parent = 0);

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

public Q_SLOTS:

  // void setTopic( const QString& topic );

protected Q_SLOTS:

  void updateTopic();

protected:

  bool sub_flag;
  QVBoxLayout* layout = new QVBoxLayout;
  QVBoxLayout* bottom_layout = new QVBoxLayout;
  QLabel *textlabel = new QLabel();

  // QTextEdit *bigEditor = new QTextEdit; 

  // QLineEdit* output_topic_editor_;
  // QLabel* input_topic_editor_;

  void paintEvent(QPaintEvent *event);

  QString output_topic_;

  ros::Publisher ros_alarms_pub;
  ros::Subscriber alarms_sub;

  ros::NodeHandle nh_;

  void callbackAlarms(const std_msgs::Bool::ConstPtr& msg);

};

} // end namespace rviz_plugin_tutorials

#endif // TELEOP_PANEL_H
