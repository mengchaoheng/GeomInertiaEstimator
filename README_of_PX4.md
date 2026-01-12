
1. The change of the code of PX4-Autopilot


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