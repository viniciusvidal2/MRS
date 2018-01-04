/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include<iostream>
#include "../include/guiROS/qnode.hpp"
#include <QFuture>
#include <QtConcurrentRun>

/*****************************************************************************
**
*****************************************************************************/

namespace guiROS {

/*****************************************************************************
**
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
    {

   cout << "iniciando ZERO" << endl;
   //QFuture<void> future = QtConcurrent::run(this, &QNode::init);


}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown();
      ros::waitForShutdown();
    }
	wait();
}



void QNode::init() {

  // start();
  cout << "iniciando classe guiROS" << endl;

  ros::init(init_argc,init_argv,"guiROS");


  if ( ! ros::master::check() )  {
    cout << "check ros master not good" << endl;
    return;
  }

  cout << "iniciando ros" << endl;
  ros::start();
  ros::NodeHandle n;
  chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

  this->run();

}


void QNode::run() {
  ros::Rate loop_rate(10);
  int count = 0;
  while ( ros::ok() ) {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    // Aqui viria as linhas de comando como num programa padrao do ros

    ROS_INFO("%s", msg.data.c_str());

    chatter_publisher.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  std::cout << "Sai do ROS e fecha GUI." << std::endl;
  Q_EMIT rosShutdown(); //
}

}  // namespace guiROS
