# Failure Detection in PX4 Stack and Landing 
>_This document assumes that you have already implemented the motor failure plugin and have setup PX4, ROS2 and  XRCE-DDS bridge_
### Integrating Failure Detection in PX4 stack
Our algorithm for detecting motor failure uses gyroscope and accelerometer data. Earlier, we had a ROS2 node that subscribed to the `/fmu/out/sensor_combined` topic on the bridge and ran the failure detection algorithm.

The same can be done in the PX4 stack by either creating our own application or module or modifying existing modules. We tried all methods, but settled by modifying existing modules. To keep the delay as minimum as possible, we modified the `SimulatorMavlink` module located at `PX4-Autopilot/src/modules/simulation/simulator_mavlink`. We estimated that this was the last module that was responsible for handling the actuator speeds that was being passed to Gazebo.

##### Modifications to `SimulatorMavlink.hpp`
The changes mainly included adding a subscriber to the `sensor_combined` uORB topic and creating some variables in the `SimulatorMavlink` class.
```cpp
//The header for subscribing to sensor_combined topic
#include <uORB/topics/sensor_combined.h>
```

```cpp
//add these variables to the public section of the SimulatorMavlink class
bool not_is_failed = true;
int failed_motor_id = 5;
```
```cpp
//add the following subscriber in the private section of the SimulatorMavlink class
uORB::Subscription _sensor_combined_sub{ORB_ID(sensor_combined)};
```

##### Modifications to `SimulatorMavlink.cpp`
Here we simply update the defined variables and implemented our algorithm. The failure is notified via the terminal where the PX4 was launched. We modiefied the `void SimulatorMavlink::actuator_controls_from_outputs(mavlink_hil_actuator_controls_t *msg)` function
```cpp
//add this snippet to the actuator_controls_from_outputs function
sensor_combined_s sensor_combined;
const float thresh = 0.6f;
if (_sensor_combined_sub.update(&sensor_combined) && not_is_failed) {
    float gyro_x = sensor_combined.gyro_rad[0];
    float gyro_y = sensor_combined.gyro_rad[1];
    float gyro_z = sensor_combined.gyro_rad[2];
    float acc_z = sensor_combined.accelerometer_m_s2[2];

    // Determine message identifier based on gyro conditions
    uint8_t new_msg = 0;
    if (gyro_x > thresh && gyro_y < -thresh)
        new_msg = 1;
    else if (gyro_x < -thresh && gyro_y > thresh)
        new_msg = 2;
    else if (gyro_x < -thresh && gyro_y < -thresh)
        new_msg = 3;
    else if (gyro_x > thresh && gyro_y > thresh)
        new_msg = 4;
    if (new_msg && acc_z > -3)
    {
        PX4_INFO(" - gyro_rad: [%.6f, %.6f, %.6f]",
		static_cast<double>(sensor_combined.gyro_rad[0]),
		static_cast<double>(sensor_combined.gyro_rad[1]),
		static_cast<double>(sensor_combined.gyro_rad[2]));
		PX4_INFO("[FAILED MOTOR ID] - %d",new_msg);
		not_is_failed=false;
		failed_motor_id=new_msg-1;
    }
}
```
At this point the Motor Failure Detection is ready. The bool `not_is_failed` can be used to switch the control algorithm when the failure occurs.


### Setting up the environment for Landing
For the remaining propellers to be able to generate enough thrust, we need to modify the upper limit of the thrust of each propeller. It can be done by either modifying the max `maxRotVelocity`, or the `motorConstant`. We are modifying the `motorConstant` for our approach. These parameters are defined in the `iris.sdf.jinja` file located at `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja`. Also to simulate the drag forces while the drone is spinning, we introduced a velocity decay to appoximate the drag forces. Finally, the `Health and arming checks` were disabled to supress the fail safe actions and prevent the drone from disarming.
##### Modifications to `iris.sdf.jinja`
Find and replace all the values of `motorConstant` with  `20.0e-06`. There should be 4 occurances of `motorConstant` tag.
```html
<motorConstant>20.0e-06</motorConstant>
```
Also add a `velocity_decay` block before the end of the `</link>` tag for `base_link`.
```html
<velocity_decay>
<angular>0.005</angular> 
</velocity_decay>
```

The complete `jinja` file can be found in the zip folder in the `Landing` subfolder, you can paste it at `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja` for implementing the necessary changes.

##### Modifications to `HealthAndArmingChecks.cpp`
This file is located at `PX4-Autopilot/src/modules/commander/HealthAndArmingChecks/HealthAndArmingChecks.cpp` and what it does is check for various health and safety parameters and act accordingly. We simply skip the checking process and allow the other parts of the code to act as normal. Simply comment out the `for` loops like so:
```cpp
bool HealthAndArmingChecks::update(bool force_reporting, bool is_arming_request)
{
	_reporter.reset();
	_reporter.prepare(_context.status().vehicle_type);
	_context.setIsArmingRequest(is_arming_request);
	// Skip the health checks loop
	// Commented out to bypass all checks
	/*
	for (unsigned i = 0; i < sizeof(_checks) / sizeof(_checks[0]); ++i) {
		if (!_checks[i]) {
			break;
		}
		_checks[i]->checkAndReport(_context, _reporter);
	}
	*/
	// Finalize the reporter without running checks
	const bool results_changed = _reporter.finalize();
	const bool reported = _reporter.report(_context.isArmed(), force_reporting);
	if (reported) {
		// Optional: Skip legacy MAVLink logging
		/*
		// LEGACY start
		_reporter._mavlink_log_pub = &_mavlink_log_pub;
		_reporter.reset();
		_reporter.prepare(_context.status().vehicle_type);
		for (unsigned i = 0; i < sizeof(_checks) / sizeof(_checks[0]); ++i) {
			if (!_checks[i]) {
				break;
			}
			_checks[i]->checkAndReport(_context, _reporter);
		}
		_reporter.finalize();
		_reporter.report(_context.isArmed(), false);
		_reporter._mavlink_log_pub = nullptr;
		// LEGACY end
		*/
		// Publish a health report indicating everything is okay
		health_report_s health_report;
		_reporter.getHealthReport(health_report);
		health_report.timestamp = hrt_absolute_time();
		_health_report_pub.publish(health_report);
	}
	// Update failsafe flags to indicate no failures
	const hrt_abstime now = hrt_absolute_time();
	if ((now > _failsafe_flags.timestamp + 500_ms) || results_changed) {
		// Set all failsafe flags to false (no failures)
		memset(&_failsafe_flags, 0, sizeof(_failsafe_flags));
		// Update the timestamp
		_failsafe_flags.timestamp = hrt_absolute_time();
		// Publish the updated failsafe flags
		_failsafe_flags_pub.publish(_failsafe_flags);
	}
	return reported;
}

void HealthAndArmingChecks::updateParams()
{
	// Optionally, you can skip updating parameters for checks
	// since checks are not performed in this modified code.
	// Comment out or remove the loop if desired.
	/*
	for (unsigned i = 0; i < sizeof(_checks) / sizeof(_checks[0]); ++i) {
		if (!_checks[i]) {
			break;
		}
		_checks[i]->updateParams();
	}
	*/
}
```
There should be three `for` loops that needs to be commneted out.
> In the recent updates to the PX4 source code, some function parameters were changed in the HealthAndArmingChecks module and may look different than the above ones, so this keep in mind if you are copying the changes. 

### Implementation  of Landing algorithm
Here we use the bool that we had generated in the Failure Detection part and extend the code in the `SimulatorMavlink` module. We introduce some helper functions for transforming vector quantities between various frames.
Following is the struct which was added to the `SimulatorMavlink.hpp` 
```cpp
struct Quaternion {
    double w, x, y, z;
    // Multiply two quaternions
    Quaternion operator*(const Quaternion& q) const {
        return {
            w * q.w - x * q.x - y * q.y - z * q.z,  // Real part
            w * q.x + x * q.w + y * q.z - z * q.y,  // i part
            w * q.y - x * q.z + y * q.w + z * q.x,  // j part
            w * q.z + x * q.y - y * q.x + z * q.w   // k part
        };
    }
    // Conjugate of quaternion
    Quaternion conjugate() const {
        return {w, -x, -y, -z};
    }
};
```
Following function does the actual transformation of the vectors.
```cpp
std::array<double, 3> rotateVector(const std::array<double, 3>& v, const Quaternion& q) {
    Quaternion v_quat = {0, v[0], v[1], v[2]}; // Vector as a pure quaternion
    Quaternion q_conj = q.conjugate();
    // Perform rotation: q * v_quat * q_conj
    Quaternion result = q * v_quat * q_conj;
    // Extract the rotated vector from the result
    return {result.x, result.y, result.z};
}
```
The core algorithm used in the landing is described in the report, but in a nutshell, the algorithm consists of multiple `P(Proportional)`, `PD(Proportional and Derivative)`, and `PID (Proportional, Intrgral and Derivative)` controllers which adjust the `actuator output` values for controlled descent.

Both the `SimulatorMavlink.cpp` and `SimulatorMavlink.hpp` files can be found in the zip folder under the `Landing` folder and should be replaced with the corresponding  `.cpp` and `.hpp` files located at `PX4-Autopilot/src/modules/simulation/simulator_mavlink/` to enable landing functionality after motor failure.

### Implementation of Partial Hovering controller
This is again a PID controller but now multiple parameters are covered under this algorithm. Like before replace the files located at `PX4-Autopilot/src/modules/simulation/simulator_mavlink/` with corresponding `SimulatorMavlink.cpp` and `SimulatorMavlink.hpp` files from the zip folder under the `Hovering` folder.
