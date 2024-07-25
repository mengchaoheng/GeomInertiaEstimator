# GeomInertiaEstimator
a nonlinear Kalman Filter based ROS package that allows to estimate inertia and geometric parameters of multirotors in-flight and re-estimates them online

By combining rotor speed measurements with data from an Inertial Measurement Unit (IMU) and any kind of pose sensor, an Unscented Kalman Filter (UKF) or Extended Kalman Filter (EKF) estimates inertia parameters (mass, moment of inertia, position of center of mass) and geometric parameters (position of IMU, position of pose sensor).

We facilitate the setup process and demonstrate the performance of the estimator by providing an example bag file containing the data from one of our experiments (_config/lissajous_trajectory.bag_). Furthermore, we provide a layout file (_config/PlotJuggler_Layout.xml_) for [PlotJuggler](http://wiki.ros.org/plotjuggler) to plot and analyse the estimates more easily.

  <img src="https://raw.githubusercontent.com/arplaboratory/GeomInertiaEstimator/master/config/Multirotor.svg?sanitize=true" width="100%" height="140">
  
<!--video -->

# Change log:
2024-7-26 
-------------------
* Add member variables: c, l, w.
  
* Add px4 support.
-------------------

# Reference
Please cite the following publication in case you are using the package in an academic context:

Wüest V, Kumar V, Loianno G. "**Online Estimation of Geometric and Inertia Parameters for Multirotor Aerial Vehicles**." _2019 IEEE International Conference on Robotics and Automation (ICRA)_. IEEE, 2019.
```
@inproceedings{wueest2018estimation,
  title={Online Estimation of Geometric and Inertia Parameters for Multirotor Aerial Vehicles},
  author={W{\"u}est, Valentin and Kumar, Vijay and Loianno, Giuseppe},
  booktitle={2019 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={},
  year={2019},
  organization={IEEE}
}
```
In the publication you can find details about:
* parameter definitions
* derivations of models
  * system dynamics
  * measurements
* filter implementation on _SO_(3)
* nonlinear observability analysis
* experimental results

## License
Please be aware that this code was originally implemented for research purposes and may be subject to changes and any fitness for a particular purpose is disclaimed.
To inquire about commercial licenses, please contact [Valentin Wüest](mailto:valentinwueest@gmail.com) and [Giuseppe Loianno](mailto:loiannog@nyu.edu).
```
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
```

## Installation
Clone the _GeomInertiaEstimator_ repo into your catkin workspace:
```
cd ~/catkin_ws/src/
git clone https://github.com/mengchaoheng/GeomInertiaEstimator.git
```

Build the _GeomInertiaEstimator_ package:
```
catkin_make --pkg geom_inertia_estimator --cmake-args -DCMAKE_BUILD_TYPE=Release
```

In case an error message appears, try running the last step again.

## Usage
To use the estimator, first enter the parameters of your multirotor in _config/quad_params.yaml_.

Make sure that the three topics _IMU_, _pose_ and _motor rpm_ are published. 

Then, remap these topics in _launch/estimator.launch_ and launch the estimator by executing:
```
roslaunch geom_inertia_estimator estimator.launch
```

## Example
Firstly, install _PlotJuggler_ if you have not already:
```
sudo apt-get install ros-$ROS_DISTRO-plotjuggler
sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros

```

In a terminal window, start the roscore:
```
roscore
```

In a second terminal window, start the estimator:
```
roslaunch geom_inertia_estimator estimator.launch
```

In a third one, play the example experiment bag file:
```
roscd geom_inertia_estimator/
rosbag play config/lissajous_trajectory.bag --pause
```

You can now plot the estimates using plotjuggler by executing this command in a fourth window:
```
roscd geom_inertia_estimator/
rosrun plotjuggler plotjuggler -l config/PlotJuggler_Layout_for_play_bag.xml 
```
When prompted, hit "_Yes (Both Layout and Streaming)_", "_OK_", and "_Create empty placeholders_". You can then unpause the bag play by clicking on the rosbag terminal window and hitting _SPACEBAR_. 

Or You can now plot the estimates using plotjuggler by executing:
```
roscd geom_inertia_estimator/
rosrun plotjuggler plotjuggler 
```
load `result.bag` data from the `/config`, load layout file from `config/PlotJuggler_Layout_for_result_bag.xml`.

Now, enjoy following the plots being drawn!

## Remark
If you intend to change the mathematical model of the estimator, please use the Unscented Kalman Filter (UKF) instead of the Extended Kalman Filter (EKF), as we have not yet provided the Matlab functions used to calculate the linearized state transition model.

In this paper/Project, the order of rotor is :
```
 1    X   4
      ^
      |
 Y <-- 


 2        3
```
But the order of px4 is 
```
 3    X   1
      ^
      |
 Y <-- 


 2        4
```
The Z Axis of all frame is set to Up Axis.(all is ENU instead of NED)!!! 

## PX4 example 
In order to using with PX4-Autopilot, the mavros pkg and rate of mavlink have to be change. Some detail show below. 
```
WARNING

This article has been tested against:

Ubuntu: 20.04
ROS: Noetic
PX4 Firmware: v1.14.0-beta2 (other version maybe don't send orientation_variance to mavlink stream).
mavros: v1.15.0 (build from soure)
mavlink: release/noetic/mavlink/2022.12.30-1 (All we need is adapt to px4 and mavros, we don't change this, so don't care about this version.)

Other version is easy to change for work.

```
### Installation
This [section](https://docs.px4.io/main/en/ros/mavros_installation.html) explains how to install ROS 1 with PX4. 
Then MAVROS shoulde be installed from source. If you can takeoff the quadrotor of gazebo simulation, and the mavros have been run to publish `/mavros/xxx` topic, some step have to be follow to using this estimator with px4:


1. code of PX4-Autopilot


clone the repositories to ~/PX4-Autopilot:
```
git clone https://github.com/PX4/PX4-Autopilot.git
cd ~/PX4-Autopilot
git checkout v1.14.0-deta2
git submodule update --init --recursive
```
and build it by 
```
make px4_sitl
```

The detail of what is change in the code is:

1.1 change the rate of mavlink stream in `src/modules/mavlink/mavlink_main.cpp` under `case MAVLINK_MODE_ONBOARD:`:
```cpp
configure_stream_local("HIGHRES_IMU", unlimited_rate); //need for rostopic /quadrotor/imu
configure_stream_local("ODOMETRY", unlimited_rate); // need for rostopic /quadrotor/ose
configure_stream_local("SERVO_OUTPUT_RAW_0", unlimited_rate); //need for rostopic /quadrotor/rpm
configure_stream_local("ATTITUDE", unlimited_rate); //(option)
```
1.2 set the value of Off-diagonal elements of pose_covariance to 0 instead of NAN in `src/modules/mavlink/streams/ODOMETRY.hpp`:
```cpp
// pose_covariance
			//  Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle
			//  (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.)
			for (auto &pc : msg.pose_covariance) {
				pc = 0;
			}
```
1.3 Use the correct timestamp in `src/modules/mavlink/streams/SERVO_OUTPUT_RAW.hpp`.
```cpp
msg.time_usec = act.timestamp / 1000;
```
1.4 setup params of the sdf file of iris in `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja`. Since the model of this project use constant value to calculate F/H, So we have to set the rotor parameters so that it has a rectangular distribution. 
```xml
<link name='rotor_0'>
      <pose>0.13 -0.22 0.023 0 0 0</pose> 
</link>
<link name='rotor_1'>
      <pose>-0.13 0.22 0.023 0 0 0</pose>
</link>
<link name='rotor_2'>
      <pose>0.13 0.22 0.023 0 0 0</pose>
</link>
<link name='rotor_3'>
      <pose>-0.13 -0.22 0.023 0 0 0</pose>
</link>

```
 (option): set `rotorDragCoefficient` and `rollingMomentCoefficient` to 0 for debug the algorithm.

1.5 Rebuild the code.

2. mavros

NOTE: we clone mavlink and mavros pkg in to path `~/mavros_ws/src`.

Follow the step in [build from soure](https://docs.px4.io/main/en/ros/mavros_installation.html).



After build all the mavros code, delet the origin version of mavros and clone my mavros repositories to ~/mavros_ws/src to recover the origin one by:
```
cd ~/mavros_ws/src
git clone https://github.com/mengchaoheng/GeomInertiaEstimator.git
```
and build it by 
```
cd ~/mavros_ws
catkin build
```


The detail of what is change in the code is:

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

3. GeomInertiaEstimator

clone this repositories to another path `~/catkin_ws/src` which build by `catkin_make`.
```
cd  ~/catkin_ws/src
git clone https://github.com/mengchaoheng/GeomInertiaEstimator.git
```
and build it by 
```
cd  ~/catkin_ws
catkin_make
```


The detail of what is change in the code is:

3.1 Add `onfig/PlotJuggler_Layout_xxx.xml` for plot data by PlotJuggler.

3.2 Add `config/px4_iris_params.yaml` for px4, have to reorder the rotor to adapt to px4, that is:
```
rotor1 <--> <link name='rotor_2'>
rotor2 <--> <link name='rotor_1'>
rotor3 <--> <link name='rotor_3'>
rotor4 <--> <link name='rotor_0'>

motorConstant = kf,   Unit: [N/rad^2]
momentConstant*kf= km,  negative if counter-clockwise ,  positive if clockwise
momentConstant = consts_.c 
R_BP=I
R_BI=I
```
Now the unit of rotor vel is rad/s. but it work. 

3.3 Add `launch/px4_estimator.launch` for launch.

3.4 Add params to Consts of `include/geom_inertia_estimator/geom_inertia_estimator.h`
```cpp 
struct Consts
{
  // for calc_XX
  flt c;
  flt w;
  flt l;
};
```
3.5 (option) Add record to `launch/estimator.launch`:
```xml
<launch>
...
    <group ns="$(arg model)">
      ...
      <node pkg="rosbag" type="record" name="bag_record" args="-a -O $(find geom_inertia_estimator)/config/result.bag"/> 

    </group>
</launch>

```
3.6 Using params in `src/geom_inertia_estimator.cpp`.
```cpp
void InertiaEstimator::onInit(const ros::NodeHandle &nh)
{
  // initialize often used variables as consts
  ...
  for (num_rotors = 0; num_rotors < 20; num_rotors++)
  {
    ...

    if (R.norm() == 3) // not defined rotor
      break;
    else
    {
      // TODO save individual kf/km for each rotor
      ...
      consts_.c=std::abs(k_M/k_f); // all rotor have the same value
      consts_.l=std::abs(t[0]);
      consts_.w=std::abs(t[1]);
      std::cout << "consts_.c: " << consts_.c << std::endl;
      std::cout << "consts_.l: " << consts_.l << std::endl;
      std::cout << "consts_.w: " << consts_.w << std::endl;
      ...
    }
  }
 ...
}
void InertiaEstimator::predictEKF(StateWithCov &state, Input &input)
{
  // calculate time diff to predicted state
  ...
  if (dt > 1.0e-5)
  {
    ...
    calc_EKF_F_optimized(
          ...
          consts_.w, consts_.l, consts_.c, consts_.k_f, F_temp_); // instead of 0.08f, 0.071f, 0.0095f, 4.179e-9f, H_temp);

    ...
  }
}
...

void InertiaEstimator::measUpdatePoseEKF(StateWithCov &state, MeasPose &meas)
{
  // since orientation measurement is linear, we use linear Kalman Filter update for orientation measurement
  // allocate matrices
  ...
  // calculate linearized observation model using Matlab generated function
  calc_EKF_H_odom_optimized_simple(
        ...
        consts_.w, consts_.l, consts_.c, consts_.k_f, H_temp);// instead of 0.08f, 0.071f, 0.0095f, 4.179e-9f, H_temp);

  // use with floats
  ...
}

...

void InertiaEstimator::measUpdateImuEKF(StateWithCov &state, Input &input, MeasImu &meas)
{
  // since angular velocity measurement is linear, we use linear Kalman Filter update for orientation measurement
  // allocate matrices
  ...
  // calculate linearized observation model using Matlab generated function
  calc_EKF_H_imu_optimized_simple(
        orientation(0),orientation(1),orientation(2),
        ...
        consts_.w, consts_.l, consts_.c, consts_.k_f, H_temp);// instead of 0.08f, 0.071f, 0.0095f, 4.179e-9f, H_temp);

  // use with floats
  ...
}
```
3.7 Rebuild the code.

## PX4 example usage
1. Run px4 sitl:
```
make px4_sitl gazebo
```
2. Run mavros
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

```
3. Run estimator node
```
roslaunch geom_inertia_estimator px4_estimator.launch  

```
4. Run plotjuggler node
```
rosrun plotjuggler plotjuggler -l config/PlotJuggler_Layout_for_px4.xml 

```
5. Flight the UAV by munual control or upload a mission to follow, you can see the states of estimator is converge.
