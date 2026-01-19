
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
      <pose>0.22 -0.22 0.023 0 0 0</pose> 
</link>
<link name='rotor_1'>
      <pose>-0.22 0.22 0.023 0 0 0</pose>
</link>
<link name='rotor_2'>
      <pose>0.22 0.22 0.023 0 0 0</pose>
</link>
<link name='rotor_3'>
      <pose>-0.22 -0.22 0.023 0 0 0</pose>
</link>

```
 (option): set `rotorDragCoefficient` and `rollingMomentCoefficient` to 0 for debug the algorithm.

1.4 Rebuild the code.
