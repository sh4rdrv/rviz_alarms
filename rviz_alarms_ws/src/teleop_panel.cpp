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

#include <stdio.h>
#include<cstring>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QRect>
#include <QPen>
#include <QComboBox>
#include <QPushButton>
#include <QTextEdit>
#include <QGroupBox>
#include <QPalette>
#include <QToolBar>
#include <QBrush>
#include <QAbstractButton>
#include <QPaintEvent>
#include <QString>
#include <QScrollArea>

#include <geometry_msgs/Twist.h>

#include "teleop_panel.h"
#include "std_msgs/Bool.h"
#include "rviz_plugin_tutorials/alarm.h"


namespace rviz_plugin_tutorials
{

// extern bool sub_flag;
extern bool alarm_status[10];

QLabel *textlabel;

const int car_num = 14;
const int left_lidar_top = 10;
const int left_lidar_bottom = 30;
const int right_lidar_top = 35;
const int right_lidar_bottom = 55;
const int radar_top = 60;
const int radar_bottom = 80;
const int extension_top = 105;
const int extension_bottom = 125;
const int intercar_angle_top = 130;
const int intercar_angle_bottom = 150;
const int steering_angle_top = 155;
const int steering_angle_bottom = 175;
const int steering_front_top = 180;
const int steering_front_bottom = 200;
const int steering_rear_top = 205;
const int steering_rear_bottom = 225;


TeleopPanel::TeleopPanel( QWidget* parent)
  : rviz::Panel( parent )
{

  QHBoxLayout* topic_layout = new QHBoxLayout;

  QHBoxLayout* radar_layout = new QHBoxLayout;

  QHBoxLayout* right_lidar_layout = new QHBoxLayout;

  QHBoxLayout* extension_layout = new QHBoxLayout;


  QVBoxLayout* layout = new QVBoxLayout;

  QLabel *hline = new QLabel(" ");
  hline->setFrameStyle(QFrame::HLine | QFrame::Raised);
  hline->setLineWidth(2);

  QLabel *hline2 = new QLabel(" ");
  hline2->setFrameStyle(QFrame::HLine | QFrame::Raised);
  hline2->setLineWidth(2);

  ros_alarms_pub = nh_.advertise<std_msgs::Bool>("bool_value_topic", 1);

  textlabel->setFrameStyle(QFrame::Panel | QFrame::Sunken);
  textlabel->setWordWrap(true);
  textlabel->setLineWidth(3);
  textlabel->setAlignment(Qt::AlignBottom);

  
  layout->addLayout( topic_layout );
  layout->addLayout(right_lidar_layout);
  layout->addLayout( radar_layout );
  layout->addLayout( extension_layout );
  layout->addLayout( bottom_layout );
  setLayout( layout );


  //Check
  QTimer* output_timer = new QTimer( this );
  // Next we make signal/slot connections.
  connect( output_timer, SIGNAL( timeout() ), this, SLOT(  updateTopic() ));
  // Start the timer.
  output_timer->start( 100 );


  layout->update();
}


void TeleopPanel::updateTopic()
{
  alarms_sub = nh_.subscribe("/ros_alarms_topic", 1, &TeleopPanel::callbackAlarms,this);
}

void TeleopPanel::callbackAlarms(const rviz_plugin_tutorials::alarm::ConstPtr& msg)
{
  rviz_plugin_tutorials::alarm send_data = *msg;
  // sub_flag = msg->data;
  for(int i=0;i<10;i++){
    alarm_status[i] = msg->data[i];
  }
  
  // ROS_INFO("%d", alarm_status);
  ros_alarms_pub.publish(send_data);
}


void TeleopPanel::paintEvent(QPaintEvent *event)
{

  QPainter painter(this);

  QBrush comms_ok = QBrush(Qt::green, Qt::SolidPattern);
  QBrush comms_bad = QBrush(Qt::yellow, Qt::SolidPattern);

  painter.setRenderHint(QPainter::Antialiasing, true);  //Message box init
  painter.setPen(QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap));


  //Parameters
  int setleft[car_num], setright[car_num];
  int cnt = 0;
  while(cnt < car_num){
    setleft[cnt] = 125 + cnt*25;
    setright[cnt] = 145 + cnt*25;
    cnt++; 
  }

  //Left Lidars

  painter.drawText(20,left_lidar_top+10,"Left Lidars");

  QRect left_lidar[car_num];

  for (int i=0;i<car_num;i++){
    left_lidar[i].setTop(left_lidar_top);
    left_lidar[i].setLeft(setleft[i]);
    left_lidar[i].setBottom(left_lidar_bottom);
    left_lidar[i].setRight(setright[i]);
  }

  for(int i=0;i<car_num;i++){
    std::string c = std::to_string(i+1);
    QString str = QString::fromStdString(c);
    if(alarm_status[i] == true){
      painter.setBrush(comms_ok);
      painter.drawRect(left_lidar[i]);
      painter.drawText(left_lidar[i],Qt::AlignCenter,str);
    }
    else {
      painter.setBrush(comms_bad);
      painter.drawRect(left_lidar[i]);
      painter.drawText(left_lidar[i],Qt::AlignCenter, str);
    }
  }
  
  //Right Lidars

  painter.drawText(20,right_lidar_top+10,"Right Lidars");

  QRect right_lidar[car_num];

  for (int i=0;i<car_num;i++){
    right_lidar[i].setTop(right_lidar_top);
    right_lidar[i].setLeft(setleft[i]);
    right_lidar[i].setBottom(right_lidar_bottom);
    right_lidar[i].setRight(setright[i]);
  }

  for(int i=0;i<car_num;i++){
    std::string c = std::to_string(i+1);
    QString str = QString::fromStdString(c);
    if(alarm_status[i] == true){
      painter.setBrush(comms_ok);
      painter.drawRect(right_lidar[i]);
      painter.drawText(right_lidar[i],Qt::AlignCenter,str);
    }
    else {
      painter.setBrush(comms_bad);
      painter.drawRect(right_lidar[i]);
      painter.drawText(right_lidar[i],Qt::AlignCenter, str);
    }
  }


  // Radars

  painter.drawText(20,radar_top+10,"Radars");

  QRect radars[car_num];

  for (int i=0;i<car_num;i++){
    radars[i].setTop(radar_top);
    radars[i].setLeft(setleft[i]);
    radars[i].setBottom(radar_bottom);
    radars[i].setRight(setright[i]);
  }

  for(int i=0;i<car_num;i++){
    std::string c = std::to_string(i+1);
    QString str = QString::fromStdString(c);
    if(alarm_status[i] == true){
      painter.setBrush(comms_ok);
      painter.drawRect(radars[i]);
      painter.drawText(radars[i],Qt::AlignCenter,str);
    }
    else {
      painter.setBrush(comms_bad);
      painter.drawRect(radars[i]);
      painter.drawText(radars[i],Qt::AlignCenter, str);
    }
  }

  // Extension

  painter.drawText(20,extension_top+10,"Extension ");

  QRect extension[car_num];

  for (int i=0;i<car_num;i++){
    extension[i].setTop(extension_top);
    extension[i].setLeft(setleft[i]);
    extension[i].setBottom(extension_bottom);
    extension[i].setRight(setright[i]);
  }

  for(int i=0;i<car_num;i++){
    std::string c = std::to_string(i+1);
    QString str = QString::fromStdString(c);
    if(alarm_status[i] == true){
      painter.setBrush(comms_ok);
      painter.drawRect(extension[i]);
      painter.drawText(extension[i],Qt::AlignCenter,str);
    }
    else {
      painter.setBrush(comms_bad);
      painter.drawRect(extension[i]);
      painter.drawText(extension[i],Qt::AlignCenter, str);
    }
  }


  // Intercar angle

  painter.drawText(20,intercar_angle_top+10,"Intercar Angle ");

  QRect intercar_angle[car_num];

  for (int i=0;i<car_num;i++){
    intercar_angle[i].setTop(intercar_angle_top);
    intercar_angle[i].setLeft(setleft[i]);
    intercar_angle[i].setBottom(intercar_angle_bottom);
    intercar_angle[i].setRight(setright[i]);
  }

  for(int i=0;i<car_num;i++){
    std::string c = std::to_string(i+1);
    QString str = QString::fromStdString(c);
    if(alarm_status[i] == true){
      painter.setBrush(comms_ok);
      painter.drawRect(intercar_angle[i]);
      painter.drawText(intercar_angle[i],Qt::AlignCenter,str);
    }
    else {
      painter.setBrush(comms_bad);
      painter.drawRect(intercar_angle[i]);
      painter.drawText(intercar_angle[i],Qt::AlignCenter, str);
    }
  }


  // Steering angle

  painter.drawText(20,steering_angle_top+10,"Steering Angle ");

  QRect steering_angle[car_num];

  for (int i=0;i<car_num-1;i++){
    steering_angle[i].setTop(steering_angle_top);
    steering_angle[i].setLeft(setleft[i]);
    steering_angle[i].setBottom(steering_angle_bottom);
    steering_angle[i].setRight(setright[i]);
  }

  for(int i=0;i<car_num;i++){
    std::string c = std::to_string(i+1);
    QString str = QString::fromStdString(c);
    if(alarm_status[i] == true){
      painter.setBrush(comms_ok);
      painter.drawRect(steering_angle[i]);
      painter.drawText(steering_angle[i],Qt::AlignCenter,str);
    }
    else {
      painter.setBrush(comms_bad);
      painter.drawRect(steering_angle[i]);
      painter.drawText(steering_angle[i],Qt::AlignCenter, str);
    }
  }

  // Steering front angle

  painter.drawText(20,steering_front_top+10,"Front Steering ");

  QRect steering_front;

  steering_front.setTop(steering_front_top);
  steering_front.setLeft(setleft[0]);
  steering_front.setBottom(steering_front_bottom);
  steering_front.setRight(setright[0]);

  std::string c = std::to_string(car_num);
  QString str = QString::fromStdString(c);
  if(alarm_status[0] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(steering_front);
    painter.drawText(steering_front,Qt::AlignCenter,str);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(steering_front);
    painter.drawText(steering_front,Qt::AlignCenter, str);
  }

    // Steering rear angle

  painter.drawText(20,steering_rear_top+10,"Rear Steering ");

  QRect steering_rear;

  steering_rear.setTop(steering_rear_top);
  steering_rear.setLeft(setleft[0]);
  steering_rear.setBottom(steering_rear_bottom);
  steering_rear.setRight(setright[0]);

  if(alarm_status[0] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(steering_rear);
    painter.drawText(steering_rear,Qt::AlignCenter,str);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(steering_rear);
    painter.drawText(steering_rear,Qt::AlignCenter, str);
  }


  //Message Box - not used for now

  painter.drawText(20,300,"MESSAGES ");
  QString temp_str = (" ");
  temp_str.append("\nNo new notifications");
  textlabel->setText(temp_str);
  textlabel->setAlignment(Qt::AlignBottom | Qt::AlignLeft);
  textlabel->setGeometry(10,320, 350, 30);
  bottom_layout->addWidget( textlabel );

  update();
 
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void TeleopPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void TeleopPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    // input_topic_editor_->setText( topic );
    updateTopic();
  }
}


} // end namespace rviz_plugin_tutorials

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::TeleopPanel,rviz::Panel )
// END_TUTORIAL


