
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
