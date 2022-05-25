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

namespace rviz_plugin_tutorials
{

extern bool sub_flag;

QLabel *textlabel;


TeleopPanel::TeleopPanel( QWidget* parent)
  : rviz::Panel( parent )
{

  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Lidar Alarms:" ));

  QHBoxLayout* radar_layout = new QHBoxLayout;

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

void TeleopPanel::callbackAlarms(const std_msgs::Bool::ConstPtr& msg)
{
  std_msgs::Bool send_data = *msg;
  sub_flag = msg->data;
  ROS_INFO("%d", sub_flag);
  ros_alarms_pub.publish(send_data);
}


void TeleopPanel::paintEvent(QPaintEvent *event)
{

  QPainter painter(this);

  QBrush comms_ok = QBrush(Qt::green, Qt::SolidPattern);
  QBrush comms_bad = QBrush(Qt::yellow, Qt::SolidPattern);

  painter.setRenderHint(QPainter::Antialiasing, true);  //Message box init
  painter.setPen(QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap));

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
  lidar_5_left_rect.setLeft(160);
  lidar_5_left_rect.setBottom(30);
  lidar_5_left_rect.setRight(180);  

  if(sub_flag == true){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_1_left_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_1_left_rect);
  }

  if(sub_flag == false){
    painter.setBrush(comms_ok);
    painter.drawRect(lidar_2_left_rect);
  }
  else {
    painter.setBrush(comms_bad);
    painter.drawRect(lidar_2_left_rect);
  }


  // if(sub_flag == true){
  //   lidar_painter.setRenderHint(QPainter::Antialiasing, true);  //Message box init
  //   lidar_painter.setPen(QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap));
  //   lidar_painter.setBrush(QBrush(Qt::green, Qt::SolidPattern));
  //   // lidar_painter.drawRect(108, 10, 20, 20);
  //   // lidar_painter.drawRect(lidar_1_left_rect);
  //   // lidar_painter.drawRect(135, 10, 20, 20);
  //   // lidar_painter.drawRect(160, 10, 20, 20);
  //   lidar_painter.drawRect(185, 10, 20, 20);
  //   lidar_painter.drawRect(212, 10, 20, 20);
  //   lidar_painter.drawRect(238, 10, 20, 20);
  //   lidar_painter.drawRect(265, 10, 20, 20);
  //   lidar_painter.drawRect(292, 10, 20, 20);
  //   lidar_painter.drawRect(319, 10, 20, 20);
  //   lidar_painter.drawRect(346, 10, 20, 20);


  // }
  // else if(sub_flag == false){
  //   lidar_painter.setRenderHint(QPainter::Antialiasing, true);
  //   lidar_painter.setPen(QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap));
  //   lidar_painter.setBrush(QBrush(Qt::yellow, Qt::SolidPattern));
  //   // lidar_painter.drawRect(108, 10, 20, 20);
  //   lidar_painter.drawRect(135, 10, 20, 20);
  //   lidar_painter.drawRect(160, 10, 20, 20);
  //   lidar_painter.drawRect(185, 10, 20, 20);
  //   lidar_painter.drawRect(212, 10, 20, 20);
  //   lidar_painter.drawRect(238, 10, 20, 20);
  //   lidar_painter.drawRect(265, 10, 20, 20);
  //   lidar_painter.drawRect(292, 10, 20, 20);
  //   lidar_painter.drawRect(319, 10, 20, 20);
  //   lidar_painter.drawRect(346, 10, 20, 20);

  // } 
  // else{
  //   lidar_painter.setRenderHint(QPainter::Antialiasing, true);
  //   lidar_painter.setPen(QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap));
  //   lidar_painter.setBrush(QBrush(Qt::gray, Qt::SolidPattern));
  //   // lidar_painter.drawRect(108, 10, 20, 20);
  //   lidar_painter.drawRect(135, 10, 20, 20);
  //   lidar_painter.drawRect(160, 10, 20, 20);
  //   lidar_painter.drawRect(185, 10, 20, 20);
  //   lidar_painter.drawRect(212, 10, 20, 20);
  //   lidar_painter.drawRect(238, 10, 20, 20);
  //   lidar_painter.drawRect(265, 10, 20, 20);
  //   lidar_painter.drawRect(292, 10, 20, 20);
  //   lidar_painter.drawRect(319, 10, 20, 20);
  //   lidar_painter.drawRect(346, 10, 20, 20);
  // }


  QPainter radar_painter(this);

  if(sub_flag == false){
    radar_painter.setRenderHint(QPainter::Antialiasing, true);
    radar_painter.setPen(QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap));
    radar_painter.setBrush(QBrush(Qt::green, Qt::SolidPattern));
    radar_painter.drawRect(108, 35, 20, 20);
    radar_painter.drawRect(135, 35, 20, 20);
    radar_painter.drawRect(160, 35, 20, 20);
    radar_painter.drawRect(185, 35, 20, 20);
    radar_painter.drawRect(212, 35, 20, 20);
    radar_painter.drawRect(238, 35, 20, 20);
    radar_painter.drawRect(265, 35, 20, 20);
    radar_painter.drawRect(292, 35, 20, 20);
    radar_painter.drawRect(319, 35, 20, 20);
    radar_painter.drawRect(346, 35, 20, 20);

  }
  else if(sub_flag == true){
    radar_painter.setRenderHint(QPainter::Antialiasing, true);
    radar_painter.setPen(QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap));
    radar_painter.setBrush(QBrush(Qt::yellow, Qt::SolidPattern));
    radar_painter.drawRect(108, 35, 20, 20);
    radar_painter.drawRect(135, 35, 20, 20);
    radar_painter.drawRect(160, 35, 20, 20);
    radar_painter.drawRect(185, 35, 20, 20);
    radar_painter.drawRect(212, 35, 20, 20);
    radar_painter.drawRect(238, 35, 20, 20);
    radar_painter.drawRect(265, 35, 20, 20);
    radar_painter.drawRect(292, 35, 20, 20);
    radar_painter.drawRect(319, 35, 20, 20);
    radar_painter.drawRect(346, 35, 20, 20);

  } 
  else{
    radar_painter.setRenderHint(QPainter::Antialiasing, true);
    radar_painter.setPen(QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap));
    radar_painter.setBrush(QBrush(Qt::gray, Qt::SolidPattern));
    radar_painter.drawRect(108, 35, 20, 20);
    radar_painter.drawRect(135, 35, 20, 20);
    radar_painter.drawRect(160, 35, 20, 20);
    radar_painter.drawRect(185, 35, 20, 20);
    radar_painter.drawRect(212, 35, 20, 20);
    radar_painter.drawRect(238, 35, 20, 20);
    radar_painter.drawRect(265, 35, 20, 20);
    radar_painter.drawRect(292, 35, 20, 20);
    radar_painter.drawRect(319, 35, 20, 20);
    radar_painter.drawRect(346, 35, 20, 20);
  }

  //Message Box
  textlabel->setFrameStyle(QFrame::Panel | QFrame::Sunken);
  if(sub_flag == true){
    textlabel->setText("Radars in fault");
  }
  else if(sub_flag == false){
    textlabel->setText("Lidars non responsive");
  }
  else{
    textlabel->setText("No error");
  }

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


