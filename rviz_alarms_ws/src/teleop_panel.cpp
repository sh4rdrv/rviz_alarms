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

#include <geometry_msgs/Twist.h>

#include "teleop_panel.h"
#include "std_msgs/Bool.h"
#include "rviz_plugin_tutorials/alarm.h"


namespace rviz_plugin_tutorials
{

// extern bool sub_flag;
extern bool alarm_status[10];

QLabel *textlabel;


TeleopPanel::TeleopPanel( QWidget* parent)
  : rviz::Panel( parent )
{

  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Left Lidars:" ));

  QHBoxLayout* radar_layout = new QHBoxLayout;

  QHBoxLayout* right_lidar_layout = new QHBoxLayout;

  QVBoxLayout* layout = new QVBoxLayout;

  QLabel *hline = new QLabel(" ");
  hline->setFrameStyle(QFrame::HLine | QFrame::Raised);
  hline->setLineWidth(2);

  //LIDAR Status
  QLabel *lidar_label1 = new QLabel(" 1 ");
  QLabel *lidar_label2 = new QLabel(" 2 ");
  QLabel *lidar_label3 = new QLabel(" 3 ");
  QLabel *lidar_label4 = new QLabel(" 4 ");
  QLabel *lidar_label5 = new QLabel(" 5 ");
  QLabel *lidar_label6 = new QLabel(" 6 ");
  QLabel *lidar_label7 = new QLabel(" 7 ");
  QLabel *lidar_label8 = new QLabel(" 8 ");
  QLabel *lidar_label9 = new QLabel(" 9 ");
  QLabel *lidar_label10 = new QLabel(" 10 ");

  lidar_label1->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  lidar_label1->setLineWidth(1);
  lidar_label1->setFixedSize(20 , 20);
  lidar_label2->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  lidar_label2->setLineWidth(2);
  lidar_label2->setFixedSize(20 , 20);
  lidar_label3->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  lidar_label3->setLineWidth(2);
  lidar_label3->setFixedSize(20 , 20);
  lidar_label4->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  lidar_label4->setLineWidth(2);
  lidar_label4->setFixedSize(20 , 20);
  lidar_label5->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  lidar_label5->setLineWidth(2);
  lidar_label5->setFixedSize(20 , 20);
  lidar_label6->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  lidar_label6->setLineWidth(2);
  lidar_label6->setFixedSize(20 , 20);
  lidar_label7->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  lidar_label7->setLineWidth(2);
  lidar_label7->setFixedSize(20 , 20);
  lidar_label8->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  lidar_label8->setLineWidth(2);
  lidar_label8->setFixedSize(20 , 20);
  lidar_label9->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  lidar_label9->setLineWidth(2);
  lidar_label9->setFixedSize(20 , 20);
  lidar_label10->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  lidar_label10->setLineWidth(2);
  lidar_label10->setFixedSize(20 , 20);

  topic_layout->addWidget(lidar_label1);
  topic_layout->addWidget(lidar_label2);
  topic_layout->addWidget(lidar_label3);
  topic_layout->addWidget(lidar_label4);
  topic_layout->addWidget(lidar_label5);
  topic_layout->addWidget(lidar_label6);
  topic_layout->addWidget(lidar_label7);
  topic_layout->addWidget(lidar_label8);
  topic_layout->addWidget(lidar_label9);
  topic_layout->addWidget(lidar_label10);


  //Right Lidar Status
  right_lidar_layout->addWidget( new QLabel( "Right Lidars:" ));
  QLabel *right_lidar_label1 = new QLabel(" 1 ");
  QLabel *right_lidar_label2 = new QLabel(" 2 ");
  QLabel *right_lidar_label3 = new QLabel(" 3 ");
  QLabel *right_lidar_label4 = new QLabel(" 4 ");
  QLabel *right_lidar_label5 = new QLabel(" 5 ");
  QLabel *right_lidar_label6 = new QLabel(" 6 ");
  QLabel *right_lidar_label7 = new QLabel(" 7 ");
  QLabel *right_lidar_label8 = new QLabel(" 8 ");
  QLabel *right_lidar_label9 = new QLabel(" 9 ");
  QLabel *right_lidar_label10 = new QLabel(" 10 ");

  right_lidar_label1->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  right_lidar_label1->setLineWidth(1);
  right_lidar_label1->setFixedSize(20 , 20);
  right_lidar_label2->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  right_lidar_label2->setLineWidth(2);
  right_lidar_label2->setFixedSize(20 , 20);
  right_lidar_label3->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  right_lidar_label3->setLineWidth(2);
  right_lidar_label3->setFixedSize(20 , 20);
  right_lidar_label4->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  right_lidar_label4->setLineWidth(2);
  right_lidar_label4->setFixedSize(20 , 20);
  right_lidar_label5->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  right_lidar_label5->setLineWidth(2);
  right_lidar_label5->setFixedSize(20 , 20);
  right_lidar_label6->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  right_lidar_label6->setLineWidth(2);
  right_lidar_label6->setFixedSize(20 , 20);
  right_lidar_label7->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  right_lidar_label7->setLineWidth(2);
  right_lidar_label7->setFixedSize(20 , 20);
  right_lidar_label8->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  right_lidar_label8->setLineWidth(2);
  right_lidar_label8->setFixedSize(20 , 20);
  right_lidar_label9->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  right_lidar_label9->setLineWidth(2);
  right_lidar_label9->setFixedSize(20 , 20);
  right_lidar_label10->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  right_lidar_label10->setLineWidth(2);
  right_lidar_label10->setFixedSize(20 , 20);


  right_lidar_layout->addWidget(right_lidar_label1);
  right_lidar_layout->addWidget(right_lidar_label2);
  right_lidar_layout->addWidget(right_lidar_label3);
  right_lidar_layout->addWidget(right_lidar_label4);
  right_lidar_layout->addWidget(right_lidar_label5);
  right_lidar_layout->addWidget(right_lidar_label6);
  right_lidar_layout->addWidget(right_lidar_label7);
  right_lidar_layout->addWidget(right_lidar_label8);
  right_lidar_layout->addWidget(right_lidar_label9);
  right_lidar_layout->addWidget(right_lidar_label10);


  //RADAR Status
  radar_layout->addWidget( new QLabel( "Radar Alarms:" ));

  QLabel *radar_label1 = new QLabel(" 1 ");
  QLabel *radar_label2 = new QLabel(" 2 ");
  QLabel *radar_label3 = new QLabel(" 3 ");
  QLabel *radar_label4 = new QLabel(" 4 ");
  QLabel *radar_label5 = new QLabel(" 5 ");
  QLabel *radar_label6 = new QLabel(" 6 ");
  QLabel *radar_label7 = new QLabel(" 7 ");
  QLabel *radar_label8 = new QLabel(" 8 ");
  QLabel *radar_label9 = new QLabel(" 9 ");
  QLabel *radar_label10 = new QLabel(" 10 ");

  radar_label1->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  radar_label1->setLineWidth(1);
  radar_label1->setFixedSize(20 , 20);
  radar_label2->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  radar_label2->setLineWidth(2);
  radar_label2->setFixedSize(20 , 20);
  radar_label3->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  radar_label3->setLineWidth(2);
  radar_label3->setFixedSize(20 , 20);
  radar_label4->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  radar_label4->setLineWidth(2);
  radar_label4->setFixedSize(20 , 20);
  radar_label5->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  radar_label5->setLineWidth(2);
  radar_label5->setFixedSize(20 , 20);
  radar_label6->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  radar_label6->setLineWidth(2);
  radar_label6->setFixedSize(20 , 20);
  radar_label7->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  radar_label7->setLineWidth(2);
  radar_label7->setFixedSize(20 , 20);
  radar_label8->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  radar_label8->setLineWidth(2);
  radar_label8->setFixedSize(20 , 20);
  radar_label9->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  radar_label9->setLineWidth(2);
  radar_label9->setFixedSize(20 , 20);
  radar_label10->setFrameStyle(QFrame::StyledPanel | QFrame::Plain);
  radar_label10->setLineWidth(2);
  radar_label10->setFixedSize(20 , 20);


  radar_layout->addWidget(radar_label1);
  radar_layout->addWidget(radar_label2);
  radar_layout->addWidget(radar_label3);
  radar_layout->addWidget(radar_label4);
  radar_layout->addWidget(radar_label5);
  radar_layout->addWidget(radar_label6);
  radar_layout->addWidget(radar_label7);
  radar_layout->addWidget(radar_label8);
  radar_layout->addWidget(radar_label9);
  radar_layout->addWidget(radar_label10);


  ros_alarms_pub = nh_.advertise<std_msgs::Bool>("bool_value_topic", 1);

  bottom_layout->addWidget( new QLabel( "Messages" ));
  bottom_layout->addWidget(hline);
  
  layout->addLayout( topic_layout );
  layout->addLayout(right_lidar_layout);
  layout->addLayout( radar_layout );
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

// void TeleopPanel::callbackAlarms(const std_msgs::Bool::ConstPtr& msg)
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


  //Left Lidars

  QRect lidar_1_left_rect;
  QRect lidar_2_left_rect;
  QRect lidar_3_left_rect;
  QRect lidar_4_left_rect;
  QRect lidar_5_left_rect;
  QRect lidar_6_left_rect;
  QRect lidar_7_left_rect;
  QRect lidar_8_left_rect;
  QRect lidar_9_left_rect;
  QRect lidar_10_left_rect;

  lidar_1_left_rect.setTop(10);
  lidar_1_left_rect.setLeft(108);
  lidar_1_left_rect.setBottom(30);
  lidar_1_left_rect.setRight(128);

  lidar_2_left_rect.setTop(10);
  lidar_2_left_rect.setLeft(135);
  lidar_2_left_rect.setBottom(30);
  lidar_2_left_rect.setRight(155);

  lidar_3_left_rect.setTop(10);
  lidar_3_left_rect.setLeft(160);
  lidar_3_left_rect.setBottom(30);
  lidar_3_left_rect.setRight(180);

  lidar_4_left_rect.setTop(10);
  lidar_4_left_rect.setLeft(185);
  lidar_4_left_rect.setBottom(30);
  lidar_4_left_rect.setRight(205);

  lidar_5_left_rect.setTop(10);
  lidar_5_left_rect.setLeft(212);
  lidar_5_left_rect.setBottom(30);
  lidar_5_left_rect.setRight(232); 

  lidar_6_left_rect.setTop(10);
  lidar_6_left_rect.setLeft(238);
  lidar_6_left_rect.setBottom(30);
  lidar_6_left_rect.setRight(258);  

  lidar_7_left_rect.setTop(10);
  lidar_7_left_rect.setLeft(265);
  lidar_7_left_rect.setBottom(30);
  lidar_7_left_rect.setRight(285); 

  lidar_8_left_rect.setTop(10);
  lidar_8_left_rect.setLeft(292);
  lidar_8_left_rect.setBottom(30);
  lidar_8_left_rect.setRight(312); 

  lidar_9_left_rect.setTop(10);
  lidar_9_left_rect.setLeft(319);
  lidar_9_left_rect.setBottom(30);
  lidar_9_left_rect.setRight(339); 

  lidar_10_left_rect.setTop(10);
  lidar_10_left_rect.setLeft(346);
  lidar_10_left_rect.setBottom(30);
  lidar_10_left_rect.setRight(366); 

  if(alarm_status[0] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_1_left_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_1_left_rect);
  }

  if(alarm_status[1] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_2_left_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_2_left_rect);
  }

  if(alarm_status[2] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_3_left_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_3_left_rect);
  }

  if(alarm_status[3] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_4_left_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_4_left_rect);
  }

  if(alarm_status[4] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_5_left_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_5_left_rect);
  }

  if(alarm_status[5] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_6_left_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_6_left_rect);
  }

  if(alarm_status[6] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_7_left_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_7_left_rect);
  }

  if(alarm_status[7] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_8_left_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_8_left_rect);
  }

  if(alarm_status[8] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_9_left_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_9_left_rect);
  }

  if(alarm_status[9] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_10_left_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_10_left_rect);
  }

  //Right Lidars

  QRect lidar_1_right_rect;
  QRect lidar_2_right_rect;
  QRect lidar_3_right_rect;
  QRect lidar_4_right_rect;
  QRect lidar_5_right_rect;
  QRect lidar_6_right_rect;
  QRect lidar_7_right_rect;
  QRect lidar_8_right_rect;
  QRect lidar_9_right_rect;
  QRect lidar_10_right_rect;

  lidar_1_right_rect.setTop(35);
  lidar_1_right_rect.setLeft(108);
  lidar_1_right_rect.setBottom(55);
  lidar_1_right_rect.setRight(128);

  lidar_2_right_rect.setTop(35);
  lidar_2_right_rect.setLeft(135);
  lidar_2_right_rect.setBottom(55);
  lidar_2_right_rect.setRight(155);

  lidar_3_right_rect.setTop(35);
  lidar_3_right_rect.setLeft(160);
  lidar_3_right_rect.setBottom(55);
  lidar_3_right_rect.setRight(180);

  lidar_4_right_rect.setTop(35);
  lidar_4_right_rect.setLeft(185);
  lidar_4_right_rect.setBottom(55);
  lidar_4_right_rect.setRight(205);

  lidar_5_right_rect.setTop(35);
  lidar_5_right_rect.setLeft(212);
  lidar_5_right_rect.setBottom(55);
  lidar_5_right_rect.setRight(232); 

  lidar_6_right_rect.setTop(35);
  lidar_6_right_rect.setLeft(238);
  lidar_6_right_rect.setBottom(55);
  lidar_6_right_rect.setRight(258);  

  lidar_7_right_rect.setTop(35);
  lidar_7_right_rect.setLeft(265);
  lidar_7_right_rect.setBottom(55);
  lidar_7_right_rect.setRight(285); 

  lidar_8_right_rect.setTop(35);
  lidar_8_right_rect.setLeft(292);
  lidar_8_right_rect.setBottom(55);
  lidar_8_right_rect.setRight(312); 

  lidar_9_right_rect.setTop(35);
  lidar_9_right_rect.setLeft(319);
  lidar_9_right_rect.setBottom(55);
  lidar_9_right_rect.setRight(339); 

  lidar_10_right_rect.setTop(35);
  lidar_10_right_rect.setLeft(346);
  lidar_10_right_rect.setBottom(55);
  lidar_10_right_rect.setRight(366); 

  if(alarm_status[0] == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_1_right_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_1_right_rect);
  }

  if(alarm_status[1] == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_2_right_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_2_right_rect);
  }

  if(alarm_status[2] == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_3_right_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_3_right_rect);
  }

  if(alarm_status[3] == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_4_right_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_4_right_rect);
  }

  if(alarm_status[4] == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_5_right_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_5_right_rect);
  }

  if(alarm_status[5] == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_6_right_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_6_right_rect);
  }

  if(alarm_status[6] == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_7_right_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_7_right_rect);
  }

  if(alarm_status[7] == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_8_right_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_8_right_rect);
  }

  if(alarm_status[8] == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_9_right_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_9_right_rect);
  }

  if(alarm_status[9] == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_10_right_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_10_right_rect);
  }


// Radars

  QRect radar_1_rect;
  QRect radar_2_rect;
  QRect radar_3_rect;
  QRect radar_4_rect;
  QRect radar_5_rect;
  QRect radar_6_rect;
  QRect radar_7_rect;
  QRect radar_8_rect;
  QRect radar_9_rect;
  QRect radar_10_rect;

  radar_1_rect.setTop(60);
  radar_1_rect.setLeft(108);
  radar_1_rect.setBottom(80);
  radar_1_rect.setRight(128);

  radar_2_rect.setTop(60);
  radar_2_rect.setLeft(135);
  radar_2_rect.setBottom(80);
  radar_2_rect.setRight(155);

  radar_3_rect.setTop(60);
  radar_3_rect.setLeft(160);
  radar_3_rect.setBottom(80);
  radar_3_rect.setRight(180);

  radar_4_rect.setTop(60);
  radar_4_rect.setLeft(185);
  radar_4_rect.setBottom(80);
  radar_4_rect.setRight(205);

  radar_5_rect.setTop(60);
  radar_5_rect.setLeft(212);
  radar_5_rect.setBottom(80);
  radar_5_rect.setRight(232); 

  radar_6_rect.setTop(60);
  radar_6_rect.setLeft(238);
  radar_6_rect.setBottom(80);
  radar_6_rect.setRight(258);  

  radar_7_rect.setTop(60);
  radar_7_rect.setLeft(265);
  radar_7_rect.setBottom(80);
  radar_7_rect.setRight(285); 

  radar_8_rect.setTop(60);
  radar_8_rect.setLeft(292);
  radar_8_rect.setBottom(80);
  radar_8_rect.setRight(312); 

  radar_9_rect.setTop(60);
  radar_9_rect.setLeft(319);
  radar_9_rect.setBottom(80);
  radar_9_rect.setRight(339); 

  radar_10_rect.setTop(60);
  radar_10_rect.setLeft(346);
  radar_10_rect.setBottom(80);
  radar_10_rect.setRight(366); 

  if(alarm_status[0] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(radar_1_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_1_rect);
  }

  if(alarm_status[1] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(radar_2_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_2_rect);
  }

  if(alarm_status[2] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(radar_3_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_3_rect);
  }

  if(alarm_status[3] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(radar_4_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_4_rect);
  }

  if(alarm_status[4] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(radar_5_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_5_rect);
  }

  if(alarm_status[5] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(radar_6_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_6_rect);
  }

  if(alarm_status[6] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(radar_7_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_7_rect);
  }

  if(alarm_status[7] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(radar_8_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_8_rect);
  }

  if(alarm_status[8] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(radar_9_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_9_rect);
  }

  if(alarm_status[9] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(radar_10_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_10_rect);
  }

  //Message Box
  textlabel->setFrameStyle(QFrame::Panel | QFrame::Sunken);
  textlabel->setWordWrap(true);
  if(alarm_status[0] == false){ textlabel->setText("Left Lidar 1 down"); }
  if(alarm_status[1] == false){ textlabel->setText("Left Lidar 2 down"); }
  if(alarm_status[2] == false){ textlabel->setText("Left Lidar 3 down"); }
  if(alarm_status[3] == false){ textlabel->setText("Left Lidar 4 down"); }
  if(alarm_status[4] == false){ textlabel->setText("Left Lidar 5 down"); }
  if(alarm_status[5] == false){ textlabel->setText("Left Lidar 6 down"); }
  if(alarm_status[6] == false){ textlabel->setText("Left Lidar 7 down"); }
  if(alarm_status[7] == false){ textlabel->setText("Left Lidar 8 down"); }
  if(alarm_status[8] == false){ textlabel->setText("Left Lidar 9 down"); }
  if(alarm_status[9] == false){ textlabel->setText("Left Lidar 10 down"); }

  // }
  // else if(sub_flag == false){
  //   textlabel->setText("Lidars non responsive");
  // }
  // else{
  //   textlabel->setText("No error");
  // }

  textlabel->setAlignment(Qt::AlignBottom | Qt::AlignRight);
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


