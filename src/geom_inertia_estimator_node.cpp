
#include <ros/ros.h>
#include "geom_inertia_estimator.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/RCOut.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Header.h>
#include <geom_inertia_estimator/MotorRPM.h>


ros::Publisher pose_pub, imu_pub, rpm_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header = msg->header;
  pose_msg.pose = msg->pose;
  pose_pub.publish(pose_msg);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  imu_pub.publish(*msg);
}

void rcOutCallback(const mavros_msgs::RCOut::ConstPtr& msg) {
  geom_inertia_estimator::MotorRPM rpm_msg;
  rpm_msg.header = msg->header;
  // for (int i = 0; i < 4 && i < msg->channels.size(); ++i) {
  //   rpm_msg.rpm[i] = msg->channels[i];
  // }
  if(msg->channels[3] >= 1000 ){
    rpm_msg.rpm[3]=(((float) (msg->channels[3]-1000)/1000.f)*1000.f +100.0f); 
  }else{
    rpm_msg.rpm[3]=0.f;
  }

  if(msg->channels[1] >= 1000 ){
    rpm_msg.rpm[1]=(((float) (msg->channels[1]-1000)/1000.f)*1000.f +100.0f);
  }else{
    rpm_msg.rpm[1]=0.f;
  }

  if(msg->channels[0] >= 1000 ){
    rpm_msg.rpm[0]=(((float) (msg->channels[0]-1000)/1000.f)*1000.f +100.0f);
  }else{
    rpm_msg.rpm[0]=0.f;
  }

  if(msg->channels[2] >= 1000 ){
    rpm_msg.rpm[2]=(((float) (msg->channels[2]-1000)/1000.f)*1000.f +100.0f);
  }else{
    rpm_msg.rpm[2]=0.f;
  }
  rpm_pub.publish(rpm_msg);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "geom_inertia_estimator");
  ros::NodeHandle nh("~");

  // 发布原有估计器输入话题
  pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 10);
  imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 10);
  rpm_pub = nh.advertise<geom_inertia_estimator::MotorRPM>("rpm", 10);

  // 订阅 MAVROS 话题并桥接
  ros::NodeHandle nh_global;
  ros::Subscriber odom_sub = nh_global.subscribe("/mavros/odometry/in", 10, odomCallback);
  ros::Subscriber imu_sub = nh_global.subscribe("/mavros/imu/data_raw", 10, imuCallback);
  ros::Subscriber rc_sub = nh_global.subscribe("/mavros/rc/out", 10, rcOutCallback);

  // 原有估计器
  InertiaEstimator estimator;
  estimator.onInit(nh);
  estimator.sub_rpm_  = nh.subscribe("rpm",  10, &InertiaEstimator::rpm_callback,  &estimator);
  estimator.sub_odom_ = nh.subscribe("pose", 10, &InertiaEstimator::pose_callback, &estimator);
  estimator.sub_imu_  = nh.subscribe("imu",  10, &InertiaEstimator::imu_callback,  &estimator);

  estimator.pub_estimates_ = nh.advertise<geom_inertia_estimator::ParameterEstimates>("param_estimates", 10);

  ros::spin();
  return 0;
}
