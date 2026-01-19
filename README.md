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

1. code of PX4-Autopilot


clone the repositories to ~/PX4-Autopilot:
```sh
git clone https://github.com/PX4/PX4-Autopilot.git
cd ~/PX4-Autopilot
git checkout v1.14.0-deta2 #(recommend)
git submodule update --init --recursive
```
and build it by 
```sh
make px4_sitl gazbeo 
# or
make px4_sitl gazebo-classic
```

The detail of what is change in the code of px4 is:

1.1 change the rate of mavlink stream in `src/modules/mavlink/mavlink_main.cpp` under `case MAVLINK_MODE_ONBOARD:`:
```cpp
configure_stream_local("HIGHRES_IMU", unlimited_rate); //need for rostopic /quadrotor/imu
configure_stream_local("ODOMETRY", unlimited_rate); // need for rostopic /quadrotor/ose
configure_stream_local("SERVO_OUTPUT_RAW_0", unlimited_rate); //need for rostopic /quadrotor/rpm
configure_stream_local("ATTITUDE", unlimited_rate); //(option)
```
1.2 set the value of Off-diagonal elements of pose_covariance and velocity_covariance to 0 instead of NAN in `src/modules/mavlink/streams/ODOMETRY.hpp`:
```cpp
// pose_covariance
for (auto &pc : msg.pose_covariance) {
  pc = 0;
}
// ....
// velocity_covariance
for (auto &vc : msg.velocity_covariance) {
  vc = 0;
}
```
1.3 setup params of the sdf file of iris in `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja`. Since the model of this project use constant value to calculate F/H, So we have to set the rotor parameters so that it has a rectangular distribution. 
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

1.4 Rebuild the code.

2. mavros

Follow the step in [Install MAVROS](https://docs.px4.io/main/en/ros/mavros_installation#binary-installation-debian-ubuntu).

```sh
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs

wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh

sudo bash ./install_geographiclib_datasets.sh
```

3. GeomInertiaEstimator

clone this repositories to another path `~/catkin_ws/src` which build by `catkin_make`.
```sh
cd  ~/catkin_ws/src
git clone https://github.com/mengchaoheng/GeomInertiaEstimator.git
# checkout to px4 branch 
git checkout px4
```
and build it by 
```sh
cd  ~/catkin_ws
catkin_make
```

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
