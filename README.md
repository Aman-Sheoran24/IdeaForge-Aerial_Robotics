# Installation of Motor Failure 
### PX4 installation
Clone the official PX4 repository
```sh
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

Run the ubuntu.sh to install all the dependencies
```sh
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
### Gazebo Installation
Install Gazebo 11 using the following command:

```sh
curl -sSL http://get.gazebosim.org | sh
```
Alternatively, you can refer to the official [website](https://classic.gazebosim.org/tutorials?tut=install_ubuntu) of Gazebo for installation steps

### Setup Micro XRCE-DDS Agent & Client 
Enter the following commands in a new terminal to build the agent.
```sh
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### Install ROS2 Humble
Follow the steps given in the [Official website](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) for installing ROS2 Humble. 
Don't forget to modify the .bashrc file to automatically setup the Environment. Add the following line in the .bashrc file.
```sh
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash
```
This will reduce the number of times you need to source the ROS2 Environment every time you start a new terminal. 


### Installing MAVLINK

Clone the following repository to get the Mavlink Library:
```sh
git clone https://github.com/mavlink/c_library_v2.git
```
If you don't want the precompiled libraries, the you can generate your own from source by following the steps from the [Official Site](https://mavlink.io/en/getting_started/installation.html) of MAVLINK. 
Place the cloned folder/generated library in the following directory  `/usr/local/include/`. This will be required in the next step
### Compiling the Plugin File for Motor Failure.

Install the following dependencies by using the following commands. We are installing `protobuf`, `jinja-2` and `gstreamer`
```sh
sudo apt install -y protobuf-compiler libprotobuf-dev
pip install jinja2
sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav
```

Run the following commands from your home directory.
```sh
mkdir -p ~/src
cd src
git clone --recursive https://github.com/PX4/sitl_gazebo.git
```
Replace the script in  `gazebo_motor_failure_plugin.cpp` file at  `~/src/sitl_gazebo/src/gazebo_motor_failure_plugin.cpp` with the following script. This script is mostly same as original one with few corrections to be compatible with ROS2.
```cpp
/*
 * Copyright 2017 Nuno Marques, PX4 Pro Dev Team, Lisbon
 * Copyright 2017 Siddharth Patel, NTU Singapore
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gazebo_motor_failure_plugin.h>

namespace gazebo {

GazeboMotorFailure::GazeboMotorFailure() :
    ModelPlugin(),
    ROS_motor_num_sub_topic_(kDefaultROSMotorNumSubTopic),
    motor_failure_num_pub_topic_(kDefaultMotorFailureNumPubTopic)
{ std::cout<<"PLUGINNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN"<<std::endl;}

GazeboMotorFailure::~GazeboMotorFailure() {
  this->updateConnection_.reset();
}

void GazeboMotorFailure::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  this->namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    this->namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  motor_failure_pub_ = node_handle_->Advertise<msgs::Int>(motor_failure_num_pub_topic_, 1);

  if (_sdf->HasElement("ROSMotorNumSubTopic")) {
    this->ROS_motor_num_sub_topic_ = _sdf->GetElement("ROSMotorNumSubTopic")->Get<std::string>();
  }

  if (_sdf->HasElement("MotorFailureNumPubTopic")) {
    this->motor_failure_num_pub_topic_ = _sdf->GetElement("MotorFailureNumPubTopic")->Get<std::string>();
  }

  // ROS2 Initialization
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }

  this->ros_node_ = rclcpp::Node::make_shared("motor_failure");

  // Create a ROS2 subscription with lambda callback
  subscription = this->ros_node_->create_subscription<std_msgs::msg::Int32>(
    this->ROS_motor_num_sub_topic_, 10,
    [this](const std_msgs::msg::Int32::SharedPtr msg) {
      this->motorFailNumCallBack(msg);
    });

  std::cout << "[gazebo_motor_failure_plugin]: Subscribed to ROS topic: " << ROS_motor_num_sub_topic_ << std::endl;

  // Connect to Gazebo world update event
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
    [this](const common::UpdateInfo &info) { this->OnUpdate(info); });
}

void GazeboMotorFailure::OnUpdate(const common::UpdateInfo &info) {
  this->motor_failure_msg_.set_data(motor_Failure_Number_);
  this->motor_failure_pub_->Publish(motor_failure_msg_);
  rclcpp::spin_some(this->ros_node_);
}

void GazeboMotorFailure::motorFailNumCallBack(const std_msgs::msg::Int32::SharedPtr msg) {
  // this->motor_Failure_Number_ = msg->data;
  if (msg->data <= 0) {
    this->motor_Failure_Number_ = 0; // Reset to no failure
  } else {
    this->motor_Failure_Number_ = msg->data;
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorFailure);
}
```

Navigate to the cloned directory and create a build folder and generate the targets using the following commands:
```sh
cd ~/src/sitl_gazebo
mkdir build
cd build
cmake .. -D_MAVLINK_INCLUDE_DIR=/usr/local/include/c_library_v2/ -DBUILD_ROS2_PLUGINS=ON
```

Now depending upon your system secifications, run either of the command from the build folder created in the last step:
```sh
#If your system has limited RAM and CPU cores then run this
make 
```
```sh
#If your system has good amount of RAM & CPU cores (>=16GB with 8 or more cores) run this for faster build
make -j6
#here -jn represents the max number of parallel processes(n) that can run for compiling the plugins
```

Add the location of the compiled objects to the .bashrc file:
```sh
# Set the plugin path so Gazebo finds our plugin
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$HOME/src/sitl_gazebo/build
# Disable online model lookup since this is quite experimental and unstable
export GAZEBO_MODEL_DATABASE_URI=""
```

### Adding the Plugin to the Drone model
Open the `iris.sdf.jinja` file located at  `~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja` and add the following following lines after the last plugin tag
```HTML
<plugin name="gazebo_motor_failure_plugin" filename="libgazebo_motor_failure_plugin.so">
      <robotNamespace/>
      <ROSMotorNumSubTopic>/motor_failure_command</ROSMotorNumSubTopic>
      <MotorFailureNumPubTopic>/motor_failure_status</MotorFailureNumPubTopic>
</plugin>
```

### Setting up keyboard control and Motor Failure Detection Script
Setup the workspace:
```sh
mkdir -p ~/ros2_px4_offboard_example_ws/src
cd ~/ros2_px4_offboard_example_ws/src
```

Clone the following repositories in the `src` folder:
```sh
#This is for px4_msgs
git clone https://github.com/PX4/px4_msgs.git -b release/1.14
#This is for the keyboard  control of the drone 
git clone https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example.git
```
Also copy the `motor_failure_detection` folder to the `src` directory 
Replace the script at `~/ros2_px4_offboard_example_ws/src/ROS2_PX4_Offboard_Example/px4_offboard/px4_offboard/processes.py` with the following:

```python
#!/usr/bin/env python3
# Import the subprocess and time modules
import subprocess
import time

# List of commands to run
commands = [
    # Run the Micro XRCE-DDS Agent
    "MicroXRCEAgent udp4 -p 8888",

    # Run the PX4 SITL simulation
    "cd ~/PX4-Autopilot && make px4_sitl gazebo-classic"

    # Run QGroundControl
    # "cd ~/QGroundControl && ./QGroundControl.AppImage"
]

# Loop through each command in the list
for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    
    # Pause between each command
    time.sleep(1)
```
Build the packages:
```sh
#Assuming you are still in the /src directory 
cd ..
colcon build
```

Source the Workspace:
```sh
source install/setup.bash
```

### Running the Simulation 
Install QGroundControl  from the [official website](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) and Start it.
Now simply launch the off-Board Example:
```sh
# Make sure that you have sourced the workspace before entering this command
ros2 launch px4_offboard offboard_velocity_control.launch.py
```

Start the Motor Failure Detection Node in another terminal. 
```sh
# Make sure that you have sourced the workspace before entering this command
ros2 run motor_failure_detection motor_failure_plugin_node 
```

To fail a motor, simply run the following command on another Terminal
```sh
# Make sure that you have sourced the workspace before entering this command
# Replace n with the motor number
ros2 topic pub /motor_failure_command std_msgs/msg/Int32 "{data: n}"
```

