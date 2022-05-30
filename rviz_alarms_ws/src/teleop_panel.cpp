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

const int car_num = 7;

TeleopPanel::TeleopPanel( QWidget* parent)
  : rviz::Panel( parent )
{

  QHBoxLayout* topic_layout = new QHBoxLayout;
  // topic_layout->addWidget( new QLabel( "Left Lidars:" ));

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

  //Right Lidar Status
  // right_lidar_layout->addWidget( new QLabel( "Right Lidars:" ));

  //RADAR Status
  // radar_layout->addWidget( new QLabel( "Radar Alarms:" ));

  //Extension
  // extension_layout->addWidget( new QLabel( "Extension:\n" ));
  // extension_layout->addWidget(hline2);


  ros_alarms_pub = nh_.advertise<std_msgs::Bool>("bool_value_topic", 1);

  textlabel->setFrameStyle(QFrame::Panel | QFrame::Sunken);
  textlabel->setWordWrap(true);
  textlabel->setLineWidth(3);

  // QScrollArea *scrollArea = new QScrollArea;
  // scrollArea->setBackgroundRole(QPalette::Dark);
  // scrollArea->setWidget(textlabel);

  // bottom_layout->addWidget( new QLabel( "Messages" ));
  // bottom_layout->addWidget(hline);
  // bottom_layout->addWidget( scrollArea );

  
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

  painter.drawText(20,25,"Left Lidars");

  int setleft[car_num], setright[car_num];
  int cnt = 0;
  while(cnt < car_num){
    setleft[cnt] = 110 + cnt*25;
    setright[cnt] = 130 + cnt*25;
    cnt++; 
  }

  // int setleft[10] = {108, 135, 160, 185, 212, 238, 265, 292, 319, 346};
  // int setright[10] = {128, 155, 180, 205, 232, 258, 285, 312, 339, 366};

  QRect newrect[car_num];

  for (int i=0;i<car_num;i++){
    newrect[i].setTop(10);
    newrect[i].setLeft(setleft[i]);
    newrect[i].setBottom(30);
    newrect[i].setRight(setright[i]);
  }

  for(int i=0;i<car_num;i++){
    std::string c = std::to_string(i+1);
    QString str = QString::fromStdString(c);
    if(alarm_status[i] == true){
      painter.setBrush(comms_ok);
      painter.drawRect(newrect[i]);
      painter.drawText(newrect[i],Qt::AlignCenter,str);
    }
    else {
      painter.setBrush(comms_bad);
      painter.drawRect(newrect[i]);
      painter.drawText(newrect[i],Qt::AlignCenter, str);
    }
  }
  
  //Right Lidars

  painter.drawText(20,55,"Right Lidars");


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
    painter.drawText(lidar_1_right_rect,Qt::AlignCenter,"1");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_1_right_rect);
    painter.drawText(lidar_1_right_rect,Qt::AlignCenter,"1");
  }

  if(alarm_status[1] == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_2_right_rect);
    painter.drawText(lidar_2_right_rect,Qt::AlignCenter,"2");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_2_right_rect);
    painter.drawText(lidar_2_right_rect,Qt::AlignCenter,"2");
  }

  if(alarm_status[2] == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_3_right_rect);
    painter.drawText(lidar_3_right_rect,Qt::AlignCenter,"3");
  }    

  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_3_right_rect);
    painter.drawText(lidar_3_right_rect,Qt::AlignCenter,"3");
  }

  if(alarm_status[3] == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_4_right_rect);
    painter.drawText(lidar_4_right_rect,Qt::AlignCenter,"4");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_4_right_rect);
    painter.drawText(lidar_4_right_rect,Qt::AlignCenter,"4");
  }

  if(alarm_status[4] == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_5_right_rect);
    painter.drawText(lidar_5_right_rect,Qt::AlignCenter,"5");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_5_right_rect);
    painter.drawText(lidar_5_right_rect,Qt::AlignCenter,"5");
  }

  if(alarm_status[5] == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_6_right_rect);
    painter.drawText(lidar_6_right_rect,Qt::AlignCenter,"6");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_6_right_rect);
    painter.drawText(lidar_6_right_rect,Qt::AlignCenter,"6");
  }

  if(alarm_status[6] == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_7_right_rect);
    painter.drawText(lidar_7_right_rect,Qt::AlignCenter,"7");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_7_right_rect);
    painter.drawText(lidar_7_right_rect,Qt::AlignCenter,"7");
  }

  if(alarm_status[7] == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_8_right_rect);
    painter.drawText(lidar_8_right_rect,Qt::AlignCenter,"8");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_8_right_rect);
    painter.drawText(lidar_8_right_rect,Qt::AlignCenter,"8");
  }

  if(alarm_status[8] == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_9_right_rect);
    painter.drawText(lidar_9_right_rect,Qt::AlignCenter,"9");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_9_right_rect);
    painter.drawText(lidar_9_right_rect,Qt::AlignCenter,"9");
  }

  if(alarm_status[9] == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_10_right_rect);
    painter.drawText(lidar_10_right_rect,Qt::AlignCenter,"10");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_10_right_rect);
    painter.drawText(lidar_10_right_rect,Qt::AlignCenter,"10");
  }


// Radars

  painter.drawText(20,85,"Radars");


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
    painter.drawText(radar_1_rect,Qt::AlignCenter,"1");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_1_rect);
    painter.drawText(radar_1_rect,Qt::AlignCenter,"1");
  }

  if(alarm_status[1] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(radar_2_rect);
    painter.drawText(radar_2_rect,Qt::AlignCenter,"2");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_2_rect);
    painter.drawText(radar_2_rect,Qt::AlignCenter,"2");
  }

  if(alarm_status[2] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(radar_3_rect);
    painter.drawText(radar_3_rect,Qt::AlignCenter,"3");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_3_rect);
    painter.drawText(radar_3_rect,Qt::AlignCenter,"3");

  }

  if(alarm_status[3] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(radar_4_rect);
    painter.drawText(radar_4_rect,Qt::AlignCenter,"4");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_4_rect);
    painter.drawText(radar_4_rect,Qt::AlignCenter,"4");
  }

  if(alarm_status[4] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(radar_5_rect);
    painter.drawText(radar_5_rect,Qt::AlignCenter,"5");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_5_rect);
    painter.drawText(radar_5_rect,Qt::AlignCenter,"5");
  }

  if(alarm_status[5] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(radar_6_rect);
    painter.drawText(radar_6_rect,Qt::AlignCenter,"6");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_6_rect);
    painter.drawText(radar_6_rect,Qt::AlignCenter,"6");
  }

  if(alarm_status[6] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(radar_7_rect);
    painter.drawText(radar_7_rect,Qt::AlignCenter,"7");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_7_rect);
    painter.drawText(radar_7_rect,Qt::AlignCenter,"7");
  }

  if(alarm_status[7] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(radar_8_rect);
    painter.drawText(radar_8_rect,Qt::AlignCenter,"8");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_8_rect);
    painter.drawText(radar_8_rect,Qt::AlignCenter,"8");
  }

  if(alarm_status[8] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(radar_9_rect);
    painter.drawText(radar_9_rect,Qt::AlignCenter,"9");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_9_rect);
    painter.drawText(radar_9_rect,Qt::AlignCenter,"9");
  }

  if(alarm_status[9] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(radar_10_rect);
    painter.drawText(radar_10_rect,Qt::AlignCenter,"10");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(radar_10_rect);
    painter.drawText(radar_10_rect,Qt::AlignCenter,"10");
  }




  // Extension

  painter.drawText(20,120,"Extension ");

  QRect bridge_1_ext;
  QRect bridge_2_ext;
  QRect bridge_3_ext;
  QRect bridge_4_ext;
  QRect bridge_5_ext;
  QRect bridge_6_ext;
  QRect bridge_7_ext;
  QRect bridge_8_ext;
  QRect bridge_9_ext;
  QRect bridge_10_ext;

  bridge_1_ext.setTop(105);
  bridge_1_ext.setLeft(108);
  bridge_1_ext.setBottom(125);
  bridge_1_ext.setRight(128);

  bridge_2_ext.setTop(105);
  bridge_2_ext.setLeft(135);
  bridge_2_ext.setBottom(125);
  bridge_2_ext.setRight(155);

  bridge_3_ext.setTop(105);
  bridge_3_ext.setLeft(160);
  bridge_3_ext.setBottom(125);
  bridge_3_ext.setRight(180);

  bridge_4_ext.setTop(105);
  bridge_4_ext.setLeft(185);
  bridge_4_ext.setBottom(125);
  bridge_4_ext.setRight(205);

  bridge_5_ext.setTop(105);
  bridge_5_ext.setLeft(212);
  bridge_5_ext.setBottom(125);
  bridge_5_ext.setRight(232); 

  bridge_6_ext.setTop(105);
  bridge_6_ext.setLeft(238);
  bridge_6_ext.setBottom(125);
  bridge_6_ext.setRight(258);  

  bridge_7_ext.setTop(105);
  bridge_7_ext.setLeft(265);
  bridge_7_ext.setBottom(125);
  bridge_7_ext.setRight(285); 

  bridge_8_ext.setTop(105);
  bridge_8_ext.setLeft(292);
  bridge_8_ext.setBottom(125);
  bridge_8_ext.setRight(312); 

  bridge_9_ext.setTop(105);
  bridge_9_ext.setLeft(319);
  bridge_9_ext.setBottom(125);
  bridge_9_ext.setRight(339); 

  bridge_10_ext.setTop(105);
  bridge_10_ext.setLeft(346);
  bridge_10_ext.setBottom(125);
  bridge_10_ext.setRight(366); 

  if(alarm_status[0] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(bridge_1_ext);
    painter.drawText(bridge_1_ext,Qt::AlignCenter,"1");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(bridge_1_ext);
    painter.drawText(bridge_1_ext,Qt::AlignCenter,"1");
  }

  if(alarm_status[1] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(bridge_2_ext);
    painter.drawText(bridge_2_ext,Qt::AlignCenter,"2");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(bridge_2_ext);
    painter.drawText(bridge_2_ext,Qt::AlignCenter,"2");
  }

  if(alarm_status[2] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(bridge_3_ext);
    painter.drawText(bridge_3_ext,Qt::AlignCenter,"3");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(bridge_3_ext);
    painter.drawText(bridge_3_ext,Qt::AlignCenter,"3");

  }

  if(alarm_status[3] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(bridge_4_ext);
    painter.drawText(bridge_4_ext,Qt::AlignCenter,"4");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(bridge_4_ext);
    painter.drawText(bridge_4_ext,Qt::AlignCenter,"4");
  }

  if(alarm_status[4] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(bridge_5_ext);
    painter.drawText(bridge_5_ext,Qt::AlignCenter,"5");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(bridge_5_ext);
    painter.drawText(bridge_5_ext,Qt::AlignCenter,"5");
  }

  if(alarm_status[5] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(bridge_6_ext);
    painter.drawText(bridge_6_ext,Qt::AlignCenter,"6");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(bridge_6_ext);
    painter.drawText(bridge_6_ext,Qt::AlignCenter,"6");
  }


  if(alarm_status[6] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(bridge_7_ext);
    painter.drawText(bridge_7_ext,Qt::AlignCenter,"7");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(bridge_7_ext);
    painter.drawText(bridge_7_ext,Qt::AlignCenter,"7");
  }

  if(alarm_status[7] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(bridge_8_ext);
    painter.drawText(bridge_8_ext,Qt::AlignCenter,"8");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(bridge_8_ext);
    painter.drawText(bridge_8_ext,Qt::AlignCenter,"8");
  }

  if(alarm_status[8] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(bridge_9_ext);
    painter.drawText(bridge_9_ext,Qt::AlignCenter,"9");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(bridge_9_ext);
    painter.drawText(bridge_9_ext,Qt::AlignCenter,"9");
  }

  if(alarm_status[9] == true){
    painter.setBrush(comms_ok);
    painter.drawRect(bridge_10_ext);
    painter.drawText(bridge_10_ext,Qt::AlignCenter,"10");
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(bridge_10_ext);
    painter.drawText(bridge_10_ext,Qt::AlignCenter,"10");
  }


  //Message Box
  // textlabel->setFrameStyle(QFrame::Panel | QFrame::Sunken);
  // textlabel->setWordWrap(true);
  // textlabel->setLineWidth(3);
  painter.drawText(20,160,"MESSAGES ");
  painter.drawLine(10,170, 400,170);
  QString temp_str = (" ");
  if(alarm_status[0] == false){ temp_str.append("\nLeft Lidar 1 down"); }
  if(alarm_status[1] == false){ temp_str.append("\nLeft Lidar 2 down"); }
  if(alarm_status[2] == false){ temp_str.append("\nLeft Lidar 3 down"); }
  if(alarm_status[3] == false){ temp_str.append("\nLeft Lidar 4 down"); }
  if(alarm_status[4] == false){ temp_str.append("\nLeft Lidar 5 down"); }
  if(alarm_status[5] == false){ temp_str.append("\nLeft Lidar 6 down"); }
  if(alarm_status[6] == false){ temp_str.append("\nLeft Lidar 7 down"); }
  if(alarm_status[7] == false){ temp_str.append("\nLeft Lidar 8 down"); }
  if(alarm_status[8] == false){ temp_str.append("\nLeft Lidar 9 down"); }
  if(alarm_status[9] == false){ temp_str.append("\nLeft Lidar 10 down"); }

  textlabel->setText(temp_str);


  // QScrollArea *scrollArea = new QScrollArea;
  // scrollArea->setBackgroundRole(QPalette::Dark);
  // scrollArea->setWidget(textlabel);


  textlabel->setAlignment(Qt::AlignBottom | Qt::AlignLeft);
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


