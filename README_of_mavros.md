
2.1 remap the topic to /quadrotor/xxx in `node.launch` file:
```xml
<node pkg="mavros" type="mavros_node" name="mavros" required="$(eval not respawn_mavros)" clear_params="true" output="$(arg log_output)" respawn="$(arg respawn_mavros)">
      <param name="fcu_url" value="$(arg fcu_url)" />
      <param name="gcs_url" value="$(arg gcs_url)" />
      <param name="target_system_id" value="$(arg tgt_system)" />
      <param name="target_component_id" value="$(arg tgt_component)" />
      <param name="fcu_protocol" value="$(arg fcu_protocol)" />
      <remap from="/mavros/odometry/pose_cov" to="/quadrotor/pose"/>
      <remap from="/mavros/imu/data_raw"  to="/quadrotor/imu"/>
      <remap from="/mavros/motorpwm"  to="/quadrotor/rpm"/>

      <!-- load blacklist, config -->
      <rosparam command="load" file="$(arg pluginlists_yaml)" />
      <rosparam command="load" file="$(arg config_yaml)" />
</node>
```
2.2 Publish `/mavros/odometry/pose_cov` in `mavros_extras/src/plugins/odom.cpp`:
```cpp
#include <geometry_msgs/PoseWithCovarianceStamped.h>
...
class OdometryPlugin : public plugin::PluginBase {
public:
	void initialize(UAS &uas_) override
	{
		...

		// publishers
		...
		local_position_cov = odom_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_cov", 10);
            ...
	}
...
private:
	ros::Publisher local_position_cov;
      ...
      void handle_odom(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ODOMETRY &odom_msg)
	{
		...

		//! Transform twist covariance matrix
		r_vel.block<3, 3>(0, 0) = r_vel.block<3, 3>(3, 3) = tf_child2child_des.linear();
		cov_vel = r_vel * cov_vel * r_vel.transpose();
		Eigen::Map<Matrix6d>(odom->twist.covariance.data(), cov_vel.rows(), cov_vel.cols()) = cov_vel;

            //---------------------------------------------------
		// for I estimator
		// publish pose_cov always
		auto pose_cov = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
		pose_cov->header = odom->header;
		pose_cov->pose = odom->pose;
		local_position_cov.publish(pose_cov);
            //---------------------------------------------------

		//! Publish the data
		odom_pub.publish(odom);
	}
};

```

2.3 Add `servo_output_raw` plugin in `mavros_extras`:

2.3.1 Add item to `mavros_extras/CMakeLists.txt`:
```
add_library(mavros_extras
  src/plugins/servo_output_raw.cpp
  ...   
)
```
2.3.2 Change `mavros_extras/mavros_plugins.xml`
```xml
<?xml version="1.0"?>
<library path="lib/libmavros_extras">
  ...
  <class name="servo_output_raw" type="mavros::extra_plugins::ServoOutputRawPlugin" base_class_type="mavros::plugin::PluginBase">
     <description>Accepts pwm.</description>
  </class>
</library>
```

2.3.3 Add `mavros_extras/src/plugins/servo_output_raw.cpp`:
```cpp
/**
 * @brief servo_output_raw plugin
 * @file servo_output_raw.cpp
 * @author meng chaoheng
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2024 meng chaoheng.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/ServoOutputRaw.h>
#include <mavros_msgs/MotorRPM.h>
namespace mavros {
namespace extra_plugins {
/**
 * @brief servo_output_raw plugin.
 */
class ServoOutputRawPlugin : public plugin::PluginBase {
public:
	ServoOutputRawPlugin() : PluginBase(),
		nh("~")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		nh.param<std::string>("frame_id", frame_id, "map");
		servo_output_raw_pub = nh.advertise<mavros_msgs::ServoOutputRaw>("servo_output_raw", 10);
        motorpwm_pub = nh.advertise<mavros_msgs::MotorRPM>("motorpwm", 10);

	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&ServoOutputRawPlugin::handle_servo_output_raw),

		};
	}

private:
	ros::NodeHandle nh;
	std::string frame_id;

	ros::Publisher servo_output_raw_pub;
    ros::Publisher motorpwm_pub;

	void handle_servo_output_raw(const mavlink::mavlink_message_t *msg, mavlink::common::msg::SERVO_OUTPUT_RAW &servo_output_raw)
	{
        // std::cout << "handle_servo_output_raw" <<  std::endl;

		auto ros_msg = boost::make_shared<mavros_msgs::ServoOutputRaw>();
		ros_msg->header = m_uas->synchronized_header(frame_id, servo_output_raw.time_usec);
        ros_msg->port = servo_output_raw.port;
		ros_msg->servo1_raw = servo_output_raw.servo1_raw;
        ros_msg->servo2_raw = servo_output_raw.servo2_raw;
        ros_msg->servo3_raw = servo_output_raw.servo3_raw;
        ros_msg->servo4_raw = servo_output_raw.servo4_raw;
        ros_msg->servo5_raw = servo_output_raw.servo5_raw;
        ros_msg->servo6_raw = servo_output_raw.servo6_raw;
        ros_msg->servo7_raw = servo_output_raw.servo7_raw;
        ros_msg->servo8_raw = servo_output_raw.servo8_raw;
        ros_msg->servo9_raw = servo_output_raw.servo9_raw;
        ros_msg->servo10_raw = servo_output_raw.servo10_raw;
        ros_msg->servo11_raw = servo_output_raw.servo11_raw;
        ros_msg->servo12_raw = servo_output_raw.servo12_raw;
        ros_msg->servo13_raw = servo_output_raw.servo13_raw;
        ros_msg->servo14_raw = servo_output_raw.servo14_raw;
        ros_msg->servo15_raw = servo_output_raw.servo15_raw;
        ros_msg->servo16_raw = servo_output_raw.servo16_raw;

		servo_output_raw_pub.publish(ros_msg);

        auto pwm = boost::make_shared<mavros_msgs::MotorRPM>();
        pwm->header = m_uas->synchronized_header(frame_id, servo_output_raw.time_usec);
		// change the order for I estimator:
		//((float) (servo_output_raw.servo1_raw-1000)/1000.f) is the value send to gazebo by px4, *1000.f is input_scaling and +100.0f is the zero_position_armed of iris sdf.
		if(servo_output_raw.servo1_raw >= 1000 ){
			pwm->rpm[3]=(((float) (servo_output_raw.servo1_raw-1000)/1000.f)*1000.f +100.0f); 
		}else{
			pwm->rpm[3]=0.f;
		}

		if(servo_output_raw.servo2_raw >= 1000 ){
			pwm->rpm[1]=(((float) (servo_output_raw.servo2_raw-1000)/1000.f)*1000.f +100.0f);
		}else{
			pwm->rpm[1]=0.f;
		}

		if(servo_output_raw.servo3_raw >= 1000 ){
			pwm->rpm[0]=(((float) (servo_output_raw.servo3_raw-1000)/1000.f)*1000.f +100.0f);
		}else{
			pwm->rpm[0]=0.f;
		}

		if(servo_output_raw.servo4_raw >= 1000 ){
			pwm->rpm[2]=(((float) (servo_output_raw.servo4_raw-1000)/1000.f)*1000.f +100.0f);
		}else{
			pwm->rpm[2]=0.f;
		}

        motorpwm_pub.publish(pwm);

	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ServoOutputRawPlugin, mavros::plugin::PluginBase)

```
2.3.4 Add `mavros_msgs/msg/MotorRPM.msg` and `mavros_msgs/msg/ServoOutputRaw.msg` msg to `mavros_msgs`:
1) `MotorRPM.msg`:
```xml
Header header
uint32[4] rpm
```
2) `ServoOutputRaw.msg`:
```
# actuator output pwm

std_msgs/Header header
uint8 port

uint16 servo1_raw
uint16 servo2_raw
uint16 servo3_raw
uint16 servo4_raw
uint16 servo5_raw
uint16 servo6_raw
uint16 servo7_raw
uint16 servo8_raw
uint16 servo9_raw 
uint16 servo10_raw 
uint16 servo11_raw
uint16 servo12_raw 
uint16 servo13_raw 
uint16 servo14_raw 
uint16 servo15_raw 
uint16 servo16_raw 
```
3) `mavros_msgs/CMakeLists.txt`:
```
add_message_files(
  ...
  MotorRPM.msg
  ServoOutputRaw.msg
  ...
)
```
2.4 Rebuild the code.