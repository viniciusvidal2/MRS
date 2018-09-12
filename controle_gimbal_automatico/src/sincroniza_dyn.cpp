#include "ros/ros.h"

#include <nav_msgs/Odometry.h>

#include <dynamixel_workbench_toolbox/dynamixel_multi_driver.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelState.h>

using namespace std;

// Mensagem sobre os dados dos motores a ser preenchida
nav_msgs::Odometry dyn_msg;

void escutarPan(const dynamixel_workbench_msgs::DynamixelStateConstPtr msg){
  dyn_msg.pose.pose.position.x  = (double)msg->present_position; // O PAN estara na posicao X da mensagem
  dyn_msg.twist.twist.angular.x = (double)msg->present_velocity; // Velocidade do PAN na orientacao X
}

void escutarTilt(const dynamixel_workbench_msgs::DynamixelStateConstPtr msg){
  dyn_msg.pose.pose.position.y  = (double)msg->present_position; // O TILT estara na posicao Y da mensagem
  dyn_msg.twist.twist.angular.y = (double)msg->present_velocity; // Velocidade do TILT na orientacao Y
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sincroniza_dyn");
  ros::NodeHandle nh;

  dyn_msg.header.frame_id = "odom"; // Por desencargo

  // Inicia subscribers dos motores
  ros::Subscriber subpandyn  = nh.subscribe("/multi_port/pan_state" , 100, &escutarPan);
  ros::Subscriber subtiltdyn = nh.subscribe("/multi_port/tilt_state", 100, &escutarTilt);
  // Publisher dos motores sincronizados
  ros::Publisher  pubdyn     = nh.advertise<nav_msgs::Odometry>("/dynamixel_sync", 100);

  ros::Rate rate(2.9); // Publicar na frequencia que chega praticamente

  while(ros::ok()){
    dyn_msg.header.stamp = ros::Time::now();
    pubdyn.publish(dyn_msg);

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
