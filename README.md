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
Then MAVROS shoulde be installed from source. If you can takeoff the quadrotor of gazebo simulation, and the mavros have been run to publish `/mavros/xxx` topic, some step have to be follow to using this estimator with px4.
We have placed the details of the changes in the `README_of_xx.md` file. We have already uploaded the modified file to GitHub, and we will proceed with the deployment using these files.

Folder structure:
```sh
~/PX4-Autopilot
~/mavros_ws/src/mavlink
~/mavros_ws/src/mavros
~/catkin_ws/src/GeomInertiaEstimator # can also be placed in mavros_ws. For easier management, it is recommended to use another ROS workspace separately.
```

1. Clone PX4 and checkout to branch `inertiaestimator`.
```sh
git clone https://github.com/mengchaoheng/PX4-Autopilot.git

git checkout inertiaestimator

git submodule update --init --recursive  
```
If submodule update have error, run:
```sh
make distclean

git checkout v1.14.0-deta2
git submodule update --init --recursive

make distclean

git checkout inertiaestimator
git submodule update --init --recursive  
```

The detail of what is change in the code can be found in `README_of_PX4.md`.


1. Clone mavros

NOTE: we clone mavlink and mavros pkg in to path `~/mavros_ws/src`.

Follow the step in [build from soure](https://docs.px4.io/main/en/ros/mavros_installation.html).


After build all the code follow above, delet the origin version of **mavros** and clone my mavros repositories to ~/mavros_ws/src to recover the origin one by:
```
cd ~/mavros_ws/src
git clone https://github.com/mengchaoheng/mavros.git
git checkout inertiaestimator
```
and build it by 
```
cd ~/mavros_ws
catkin build
```

The detail of what is change in the code can be found in `README_of_mavros.md`.

1. Clone GeomInertiaEstimator

clone this repositories to another path `~/catkin_ws/src` which build by `catkin_make`.
```
cd  ~/catkin_ws/src
git clone https://github.com/mengchaoheng/GeomInertiaEstimator.git
git checkout inertiaestimator
```
and build it by 
```
cd  ~/catkin_ws
catkin_make
```


The detail of what is change in the code can be found in `README_of_GeomInertiaEstimator.md`.

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
