# Carver-Savvy : Deep Learning-based Navigation for Mobile Robots in Dynamic Environments

## A ROS2 Project for DRL autonomous navigation on mobile robots with LiDAR. Documentation: [link](https://drive.google.com/file/d/1YvqdD2zfSpZBf8gw019WrEhlrialaLND/view?usp=sharing)

<p float="left">
 <img src="media/project_overview.gif" width="800">
</p>

# **Table of contents**
* [Introduction](#introduction)
* [Installation](#installation)
* [Running the Project](#running-the-project)
  * [Training the Agent](#training-the-agent)
  * [Optional Configuration](#optional-configuration)
  * [Utilities](#utilities)
* [Physical Robot](#physical-robot)
  * [Micro-Ros-node](#micro-ros-node)
  * [ROS Bridge](#ros-bridge)
  * [DRL Agent](#drl-agent)
  * [Real world implementation suggestions](#real-world-implementation-suggestions)

# **Introduction**

This repository provides a ROS2 and PyTorch framework designed for developing and experimenting with deep reinforcement learning for autonomous navigation in mobile robots. The models are trained in simulation and evaluated in both simulated and real-world environments. The project utilizes the Carver-savvy, a differential drive mobile robot equipped with essential sensors such as a 2D lidar, IMU, and encoders. The reinforcement learning agent is trained in a simulation environment using ROS2 Gazebo. Nevertheless, the framework is flexible and can be adapted to any robot model that provides LiDAR and odometry information and can operate with linear/angular velocity messages.

The applications enabled by the current framework:

* Train, save, load, and assess a navigation agent within varied simulated environments.
* Implement a pre-existing model onto an actual robot for navigation and obstacle avoidance.
* Analyze the impact of various hyperparameters on training duration and performance.
* Develop custom Deep Reinforcement Learning (DRL) algorithm options [Algorithim in this repo: TD3, SAC.]
* Micro-ros integration for real-time communication with the robot.
* Kalman filter for sensor fusion and odometry correction using robot_localization package.
* ROS2 Navigation for testing robot navigation in real-world environments.

# **Installation**

## **Dependencies**

*   Ubuntu 22.04 LTS (Jammy Jellyfish) [link](https://releases.ubuntu.com/jammy/)
*   ROS 2 Humble Hawksbill [link](https://docs.ros.org/en/humble/Installation.html)
*   Gazebo (Version: 11.10.2) [link](https://gazebosim.org/docs)
*   PyTorch (Version: 2.3.0) [link](https://pytorch.org/)

## **Installing ROS2**
Install ROS 2 Humble according to the following guide: [link](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). Recommended installation is the desktop version (ROS, RViz, demos, tutorials). <br>
To prevent having to manually source the setup script every time, add the following line at the end of your `~/.bashrc` file:

```
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash
```

## **Installing Python3, Pytorch**

The last tested version for this project was Python 3.10.12

Install pip3 (python package manager for python 3) as follows:
```bash
sudo apt install python3-pip
```

To install the tested version of PyTorch (2.3.0) with CUDA support, run the following command:
```bash
pip3 install torch torchvision torchaudio
```

or without CUDA support:
```bash
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
```

To install the `pyqtgraph` and `PyQt` with is optional and only necessary if you want to visualize the neural network activity. 

```bash
pip3 install pyqtgraph PyQt5
```

**Note: The version of CUDA support to install will depend on the [compute capability](https://developer.nvidia.com/cuda-gpus) of your GPU**

## **Enabling GPU support (recommended)**

We can significantly speed up the training procedure by making use of a GPU. If no GPU is available or it is not initialized correctly the training will automatically be redirected to the CPU. Since most users have access to an NVIDIA GPU we will explain how to enable this to work with PyTorch on linux.
Three different components are required to train on GPU:
- NVIDIA drivers for linux
- The CUDA library for linux
- cuDNN (comes with pytorch and should be installed automatically)

Press the windows/command key and type "Additional drivers" to make the corresponding linux menu come up. Here, multiple radio button options should be listed for installing different nvidia drivers. Install the option with the latest version (highest number, e.g. currently nvidia-driver-510).

The next step is to download the correct CUDA version. This will depend on your NVIDIA drivers and GPU variant. Following [link](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_local) will take you to the CUDA download page. Follow the instructions to download the correct version for your system.

You can then verify that CUDA is installed using:

```bash
nvidia-smi
```

Which should display version numbers and GPU information. If you see this, you are enabled to train on the GPU.

Example output:
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.171.04    Driver Version: 535.171.04    CUDA Version: 12.2   |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|                               |                      |               MIG M. |
|===============================+======================+======================|
|   0  GeForce RTX 3060    Off  | 00000000:01:00.0 Off |                  N/A |
|  0%   41C    P8    15W / 350W |      0MiB / 24268MiB |      0%      Default |
|                               |                      |                  N/A |
+-------------------------------+----------------------+----------------------+
```

## **Downloading the Repository**

Open a terminal in the desired location for the new workspace. Clone the repository using:
```bash
git clone https://github.com/AthimetA/CARVER-Savvy.git
```

`cd` into the directory and make sure you are on the main branch
```bash
cd CARVER-Savvy/
git checkout main
```

Next, the ros2 dependencies by running (this will install all the necessary dependencies for the project):
```bash
source install.bash
```

Next, Build the workspace using colcon:
```bash
# If you are in the CARVER-Savvy directory
cd carverabwu/
# Build the workspace using colcon
# Always build with --symlink-install to avoid having to rebuild the workspace after every change
colcon build --symlink-install
```
After colcon has finished building source the repository using: 
```bash
source install/setup.bash
```

or add the following line to your `~/.bashrc` file (if the repository is in your home directory):
```bash
source ~/CARVER-Savvy/carverabwu/install/setup.bash
```

The last thing we need to do before running the code is add a few lines to our `~/.bashrc` file so that they are automatically executed whenever we open a new terminal. Add the following lines at the end of your `~/.bashrc` file and **replace the path to the workspace with the path to your workspace**:

```bash
# ROS2 domain id for network communication, machines with the same ID will receive each others' messages
export ROS_DOMAIN_ID=1

WORKSPACE_DIR=~/CARVER-Savvy/carverabwu
export ABWUDRL_BASE_PATH=$WORKSPACE_DIR

# Model path for Gazebo
export SIM_MODEL_PATH=$WORKSPACE_DIR/src/abwu_gazebo/models/

#Allow gazebo to find models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$WORKSPACE_DIR/src/abwu_gazebo/models

```

**Note: Always make sure to first run ```source install/setup.bash``` or open a fresh terminal after building with `colcon build`.**


# **Running the Project**

System architecture of the project is shown below:

<p float="left">
 <img src="media/SystemArchitecture.jpg" width="800">
</p>

The project is divided into three main components: the reinforcement learning agent, the reinforcement learning environment, and the robot. 
* The agent is responsible for learning the optimal policy for navigating the robot in the environment. 
* The environment is responsible for providing the agent with the necessary information to learn the policy. In training, the environment is simulated in Gazebo and the micro-ros node is used to communicate with the robot. In testing, the environment is the real world and the robot is controlled by the agent.
* The robot is responsible for executing the policy learned by the agent in the environment. In training, the robot is simulated in Gazebo and the micro-ros node is used to communicate with the environment. In testing, the robot is the physical robot.

## **Training the Agent**

To train the agent, we need to run the following commands in separate terminals:

1. Start the Gazebo simulation:
```bash
# The stage number can be changed to choose a different stage
ros2 launch awbu_drl abwu_drl_stage_5.launch
```

2. Start the reinforcement environment:
```bash
ros2 launch awbu_drl abwu_drl_env.launch
```

Note: the obstacle collision probability calculation node in simulation better be running using drl_obstacle_cp.py script for better detection of obstacles. for the real obstacle collision probability calculation, the detection of obstacles is done using the lidar data and the velocity of the obstacles is calculated using the kalman filter. 

Your can change the obstacle detection and velocity calculation method by changing the `drl_obstacle_cp` node in the `abwu_drl_env.launch.py` file.

```python

    drl_obstacle_cp = Node(
            package='awbu_drl',
            executable='drl_obstacle_cp_real.py', # change to drl_obstacle_cp.py for simulation
            name='drl_obstacle_cp',
            parameters=[{'use_sim_time': use_sim_time}],
         )

``` 

Note: By using drl_obstacle_cp_real.py script, the Evaluation matrix is calculation is <b>not implemented</b> yet.

3. Start the reinforcement learning agent:

When no arguments are provided, the agent will run in the default mode. The default mode is agent = 'td3' and mode = 'train'. The agent will start training and the training progress will be displayed in the terminal. 

```bash
ros2 run awbu_drl drl_agent
```

for Optinal mode, you can run the agent with the following command:

TD3:

```bash
ros2 run awbu_drl drl_agent td3
```
SAC:

```bash
ros2 run awbu_drl drl_agent sac
```

Training and Testing, can be run with following command:
```bash
ros2 run awbu_drl drl_agent td3 train
```
```bash
ros2 run awbu_drl drl_agent td3 test
```

## **Optional Configuration**

### Settings: change parameters

The `settings/constparams.py` file contains most of the interesting parameters that you might wish to change, including the DRL hyperparameters.

### Reward: tweak reward design

The `drlutils_reward.py` file contains the reward design. You can change the reward function to improve the agent's performance.

### Map and Obstacle: change the map and obstacle design

You can change the map and obstacle design by editing the `abwu_gazebo/maps` and `abwu_gazebo/models` directories.

the `abwu_gazebo/maps` directory contains the map files that define the environment. The map files are in the `.world` format and can be edited using Gazebo.

the `abwu_gazebo/models` directory contains the obstacle models that are placed in the environment. The obstacle models are in the `.sdf` format and can be edited using Gazebo.

for dynamic goal, the goal position can be changed by editing the model file in the `abwu_gazebo/models` directory.

When edit Physic in Gazebo, if you want to run the simulation in real-time (x1) or faster (x2) you can change in the world file as shown below:

```yaml
    <physics type="ode">

      <!-- Physics Rule 
      
      1. max_step_size: 
          The maximum time step size that can be taken by a variable time-step solver (such as simbody) during simulation. 
          For physics engines with fixed-step solvers (like ODE), this is simply the time step size. 
          The default value in Gazebo is 0.001 seconds.
      2. real_time_update_rate: 
          This is the frequency at which the simulation time steps are advanced. 
          The default value in Gazebo is 1000 Hz. Multiplying with the default max_step_size of 0.001 seconds gives a real_time_factor of 1.
          If real_time_update_rate is set to 0 the simulation will run as fast as it can. 
          If Gazebo is not able to update at the desired rate, it will update as fast as it can, based on the computing power.
      3. real_time_factor:
          max_step_size x real_time_update_rate sets an upper bound of real_time_factor. 
          If real_time_factor < 1 the simulation is slower than real time.
          real_time_factor = 1 means the simulation runs in real-time.
          real_time_factor = 2 means the simulation runs twice as fast as real-time.

      [real_time_factor = max_step_size * real_time_update_rate]
      
      -->
      <!-- Physics x1 realtime-->
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <real_time_factor>1</real_time_factor>

      <!-- Physics x2 realtime-->
      <!-- <max_step_size>0.002</max_step_size> 
      <real_time_update_rate>1000.0</real_time_update_rate>
      <real_time_factor>2</real_time_factor>  -->

      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <precon_iters>0</precon_iters>
          <sor>1.400000</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.000000</contact_max_correcting_vel>
          <contact_surface_layer>0.01000</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
```

Note: when edit real_time_update_rate, the controller parameters config file should be changed to the same value as the real_time_update_rate in the world file. You can edit at `carversavvy_description/config/carversavvy_controller.yaml` file.

```yaml
controller_manager:
  ros__parameters:

    use_sim_time: true
    
    # x1 Simulator
    update_rate: 1000 # Hz

    # x2 Simulator
    # update_rate: 500 # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    
    velocity_cont:
      type: velocity_controllers/JointGroupVelocityController

```


## **Utilities**

### Visualization: visualize the neural network activity

The `drlutils_visualization.py` file contains the code for visualizing the neural network activity. You can edit this file to visualize the neural network activity or any other information you wish to visualize.

<p float="left">
 <img src="media/visual.gif" width="800">
</p>

### Collision Probability: calculate the collision probability

The `drl_obstacle_cp.py` or `drl_obstacle_cp_real.py` file contains the code for calculating the collision probability. You can edit this file to change the collision probability calculation method.

There is a viusalization of the collision probability using rviz. Example of the visualization is shown below:

<p float="left">
 <img src="media/collision_probability.gif" width="800">
</p>

# **Physical Robot**

The physical robot is a Carver-Savvy, a differential drive mobile robot equipped with sensors: 2D lidar, IMU, and encoders.

## **Micro-Ros-node**

### **Set up Micro-Ros**

```bash
cd microros_ws
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```
### Microcontroller Programming
- **subscription**
  ```cpp
  void subscription_callback(const void *msgin) 
  {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    // if velocity in x direction is 0 turn off LED, if 1 turn on LED

    Sub_speedL = (msg->linear.x/ROBOT_WHEEL_RADIUS) - (msg->angular.z * ROBOT_BASE_WIDTH / (2*ROBOT_WHEEL_RADIUS));
      Sub_speedR = (msg->linear.x/ROBOT_WHEEL_RADIUS) + (msg->angular.z * ROBOT_BASE_WIDTH / (2*ROBOT_WHEEL_RADIUS));
    robotVelocityCmd.Vx = msg->linear.x;
    robotVelocityCmd.Vy = msg->linear.y;
    robotVelocityCmd.w = msg->angular.z;
  }
  ```
- **timer**
  ```cpp
  void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
  {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {

    inputcontrolL_msg.data = Sub_speedL;
    inputcontrolR_msg.data = Sub_speedR;
    wheelL_vel_msg.data = encoderLrad;
    wheelR_vel_msg.data = encoderRrad;
    // IMU_msg.angular_velocity.x = -1.0*IMU_data[0];
    // IMU_msg.angular_velocity.y = IMU_data[2];
    IMU_vz_msg.data = IMU_data[1];
    IMU_ax_msg.data = -1.0* IMU_data[3];
    // IMU_msg.linear_acceleration.y = IMU_data[5];
    // IMU_msg.linear_acceleration.z = IMU_data[4];
    // IMU_msg.orientation.w = IMU_data[6];
    // IMU_msg.orientation.x = IMU_data[7];
    // IMU_msg.orientation.y = IMU_data[8];
    IMU_yaw_msg.data = IMU_data[8];

    RCSOFTCHECK(rcl_publish(&wheelL_vel_publisher, &wheelL_vel_msg, NULL));
    RCSOFTCHECK(rcl_publish(&wheelR_vel_publisher, &wheelR_vel_msg, NULL));
    RCSOFTCHECK(rcl_publish(&IMU_yaw_publisher, &IMU_yaw_msg, NULL));
    RCSOFTCHECK(rcl_publish(&IMU_vz_publisher, &IMU_vz_msg, NULL));
    RCSOFTCHECK(rcl_publish(&IMU_ax_publisher, &IMU_ax_msg, NULL));
    RCSOFTCHECK(rcl_publish(&inputL_publisher, &inputcontrolL_msg, NULL));
    RCSOFTCHECK(rcl_publish(&inputR_publisher, &inputcontrolR_msg, NULL));
    }
  }
  ```
- **setup**
  ```cpp
  void uROSsetup()	
  {
    set_microros_serial_transports(Serial);
    allocator = rcl_get_default_allocator();
    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    //create node
    RCCHECK(rclc_node_init_default(&node, "mini_project_PMZB_node", "", &support));
      // sync time
      // rmw_uros_sync_session(1000);
    //create publisher

    RCCHECK(rclc_publisher_init_default(&wheelL_vel_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheelL_vel"));
    RCCHECK(rclc_publisher_init_default(&wheelR_vel_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "wheelR_vel"));

    // RCCHECK(rclc_publisher_init_default(&inputL_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "inputL"));
    // RCCHECK(rclc_publisher_init_default(&inputR_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "inputR"));
    RCCHECK(rclc_publisher_init_default(&IMU_yaw_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "IMU_yaw"));
    RCCHECK(rclc_publisher_init_default(&IMU_vz_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "IMU_vz"));
    RCCHECK(rclc_publisher_init_default(&IMU_ax_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "IMU_ax"));
    //create subscriber
    RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "carversavvy_cmd_vel"));
    //create timer
    const unsigned int timer_timeout = 1000/sampling_time; //25 HZ
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));
    //create executor
    RCCHECK(rclc_executor_init(&executor_pub, &support.context, 6, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor_pub, &subscriber, &Speed_msg, &subscription_callback, ON_NEW_DATA));
  }
  ```
- **loop**
  ```cpp
  void uROSloop()
  {
    RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(1)));
    delay(1);
  }	
  ```
- **IMU setup**

  ```cpp
  if (!bno08x.begin_I2C()) {
    	while (1) {
      	delay(10);
        }
      }
	for (int n = 0; n < bno08x.prodIds.numEntries; n++) {

    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    // Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    // Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    // Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    // Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  	}
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
  }
  if(!bno08x.enableReport(SH2_ROTATION_VECTOR)){
  }
  ```
- **IMU read loop**
  ```cpp
  if (bno08x.wasReset()) {
    		// Serial.print("sensor was reset ");
    		// setReports();
  			}		
  		if (!bno08x.getSensorEvent(&sensorValue)) {
    		return;
  			}
  		switch (sensorValue.sensorId) {
			case SH2_GYROSCOPE_CALIBRATED:
				IMU_data[0] = sensorValue.un.gyroscope.x;
				IMU_data[1] = sensorValue.un.gyroscope.y;
				IMU_data[2] = sensorValue.un.gyroscope.z;
				break;
			case SH2_LINEAR_ACCELERATION:
				IMU_data[3] = -1.0 * sensorValue.un.linearAcceleration.x;
				IMU_data[4] = -1.0 * sensorValue.un.linearAcceleration.y;
				IMU_data[5] = -1.0 * sensorValue.un.linearAcceleration.z;
				break;
			case SH2_ROTATION_VECTOR:
				IMU_data[6] = sensorValue.un.rotationVector.real;
				IMU_data[7] = sensorValue.un.rotationVector.i;
				IMU_data[8] = sensorValue.un.rotationVector.j;
				IMU_data[9] = sensorValue.un.rotationVector.k;
				break;
  		}
  ```
 
  | Index | Size |         Data          |
  | :---: | :--: | :-------------------: |
  |   0   |  32  |  Angular Velocity X   |
  |   1   |  32  |  Angular Velocity Y   |
  |   2   |  32  |  Angular Velocity Z   |
  |   3   |  32  | Linear Acceleration X |
  |   4   |  32  | Linear Acceleration Y |
  |   5   |  32  | Linear Acceleration Z |
  |   6   |  32  |  Rotational Vector    |  
  |   7   |  32  |         Roll          |
  |   8   |  32  |         Pitch         |
  |   9   |  32  |          Yaw          |


  The performance of the data transmission is approximately 25Hz.
 - **Feedforward function**
  ```cpp
  float InverseTFofMotorL(float Velo, float PredictVelo)
  {
    static float VeloLast = 0.0;
    static float Voltage = 0.0;
    static float VoltageLast = 0.0;
    static float Pwm = 0;
    if (PredictVelo > 0) 
    {
      Voltage = ((PredictVelo*2.2937 +3.5326)) - ((2.2937 *Velo));
    }
    else
    {
      Voltage = ((2.076*PredictVelo -3.1637) )- ((2.076*Velo) );
    }
    Pwm = (Voltage * 255)/24.0;
    return Pwm;
  }

  float InverseTFofMotorR(float Velo, float PredictVelo)
  {
    static float VeloLast = 0.0;
    static float Voltage = 0.0;
    static float VoltageLast = 0.0;
    static float Pwm = 0;
    if (PredictVelo > 0)
    {
      Voltage = ((PredictVelo*2.2501 +2.7908)) - ((2.2501 *Velo));
    }
    else
    {
      Voltage = ((2.1973*PredictVelo -2.9272) )- ((2.1973*Velo) );
    }
    Pwm = (Voltage * 255)/24.0;

    return Pwm;
  }
  ```
 - **Control setup**
 ```cpp
void controlSetup(){
	//move parameter to general_params.h
	//parameter setup
  pidParameter1.Kp = 0.034;
  pidParameter1.Ki = 0.001;
  pidParameter1.Kd = 0.1;
  pidParameter1.sampleTime = 1000/sampling_time;

  pidParameter2.Kp = 0.034;
  pidParameter2.Ki = 0.001;
  pidParameter2.Kd = 0.1;
  pidParameter2.sampleTime = 1000/sampling_time;

  pidParameter3.Kp = 0.048;
  pidParameter3.Ki = 0.002;
  pidParameter3.Kd = 0.0;
  pidParameter3.sampleTime = 1000/sampling_time;

  pidParameter4.Kp = 0.048;
  pidParameter4.Ki = 0.002;
  pidParameter4.Kd = 0.0;
  pidParameter4.sampleTime = 1000/sampling_time;

  motorR.pwmChannel = 1;
  motorR.pwmPin = 25;
  motorR.outAPin = 27;
  motorR.outBPin = 23;

  motorL.pwmChannel = 0;
  motorL.pwmPin = 26;
  motorL.outAPin = 33;
  motorL.outBPin = 32;
	//encoder 
	control.initialEnc(&encoderL, "enc1", pinEncA1, pinEncB1, 4096); 
  // encoder pin 17, 4
  control.initialEnc(&encoderR, "enc2", pinEncA2, pinEncB2, 4096); 
  // encoder pin 14, 19
  control.initialEnc(&encoder, "enc1", pinEncA1, pinEncB1, 4096);
	control.initialEnc(&encoder2, "enc2", pinEncA2, pinEncB2, 4096);
	// //motor setup
	control.initialMotor(&motorL, 100, 8);
  control.initialMotor(&motorR, 100, 8);
	// //PID setup
	control.initialPID(&pidController1, &pidParameter1, limitOffset1);
 	control.initialPID(&pidController2, &pidParameter2, limitOffset2);
	control.initialPID(&pidController3, &pidParameter3, limitOffset1);	
	control.initialPID(&pidController4, &pidParameter4, limitOffset2);
	control.zeroOutputSum(&pidController1);
	control.zeroOutputSum(&pidController2);
	control.zeroOutputSum(&pidController3);
	control.zeroOutputSum(&pidController4);
}
  ```
 - **Control loop**
  ```cpp
  void controlLoop()
  {
	// PID 100 HZ
	if (millis() - preIntervelMillis >= (1000/sampling_time)) 
	{
		preIntervelMillis = millis();
		// get velocity command
		robotVelocityCmd.v1 = Sub_speedL;
		robotVelocityCmd.v2 = Sub_speedR;
		pidParameter1.setPoint = rad_to_enc(robotVelocityCmd.v1);// 4096 pulse per revolution
		pidParameter2.setPoint = rad_to_enc(robotVelocityCmd.v2);
		pidParameter3.setPoint = rad_to_enc(robotVelocityCmd.v1);
		pidParameter4.setPoint = rad_to_enc(robotVelocityCmd.v2);
		encoderLrad = enc_to_rad(control.getIntervalEnc(&encoder)); 
    encoderRrad = enc_to_rad(control.getIntervalEnc(&encoder2));
    //feed forward control
		feedfowardL = InverseTFofMotorL(encoderLrad, robotVelocityCmd.v1);
		feedfowardR = InverseTFofMotorR(encoderRrad, robotVelocityCmd.v2);
		// setpoint
		if (robotVelocityCmd.w != 0)
		{
			control.setpoint(&pidController3, &pidParameter3, &encoderL);
			control.setpoint(&pidController4, &pidParameter4, &encoderR);
			feedfowardL = feedfowardL +pidParameter3.output;
			feedfowardR = feedfowardR +pidParameter4.output;
		}
		else
		{
			control.setpoint(&pidController1, &pidParameter1, &encoderL);
			control.setpoint(&pidController2, &pidParameter2, &encoderR);
			feedfowardL = feedfowardL +pidParameter1.output;
			feedfowardR = feedfowardR +pidParameter2.output;
		}
		if (feedfowardL > 250)
		{
			feedfowardL = 250;
		}
		if (feedfowardR > 250)
		{
			feedfowardR = 250;
		}
		if (robotVelocityCmd.Vx == 0 && robotVelocityCmd.w == 0) //all stop
		{
			control.drive(&motorL, 0);
			control.drive(&motorR, 0);
			control.zeroOutputSum(&pidController1);
			control.zeroOutputSum(&pidController2);
			control.zeroOutputSum(&pidController3);
			control.zeroOutputSum(&pidController4);
		}
		if (pidParameter1.setPoint == 0 && pidParameter2.setPoint == 0)
		{
			control.drive(&motorL, 0);
			control.drive(&motorR, 0);
			control.zeroOutputSum(&pidController1);
			control.zeroOutputSum(&pidController2);
			control.zeroOutputSum(&pidController3);
			control.zeroOutputSum(&pidController4);
		}
		else
		{           
		// drive
			control.drive(&motorL, feedfowardL);
			control.drive(&motorR, feedfowardR);
		}
		}
  }  
  ```

## **ROS Bridge**

This node serves to compute Wheel Odometry and calibrate IMU data. It takes inputs such as Wheel Velocity and Raw IMU data from topics sent by the ESP32.
<p float="center">
 <img src="media/velocity.png" width="600">
</p>
<p float="center">
 <img src="media/velocity2.png" width="600">
</p>

```python
def forward_kinematic_cal(self, wl, wr):
    now = self.get_clock().now()
    dt = now - self.time_last
    self.time_last = now
    dt = dt.nanoseconds / NS_TO_SEC

    if type(wl) == Float32:
        wl = wl.data
    if type(wr) == Float32:
        wr = wr.data

    # calculate the linear and angular velocity of the robot
    vx = (self._WHEEL_RADIUS/2) * (wl + wr)
    wz = (self._WHEEL_RADIUS/self._BASE_WIDTH) * (wr - wl)

    # Position
    ds = vx * dt
    dtheta = wz * dt

    if wz != 0:
        self.theta += dtheta

    if vx != 0:
        # calculate distance traveled in x and y
        x = np.cos(dtheta) * ds
        y = -np.sin(dtheta) * ds
        # calculate the final position of the robot
        self.x += ((np.cos(self.theta) * x) - (np.sin(self.theta) * y))
        self.y += ((np.sin(self.theta) * x) + (np.cos(self.theta) * y))

    # Create the odometry message
    odom_msg = Odometry()
    odom_msg.header.stamp = self.get_clock().now().to_msg()
    odom_msg.header.frame_id = 'odom'
    odom_msg.child_frame_id = 'base_link'

    # Set the position
    odom_msg.pose.pose.position.x = self.x
    odom_msg.pose.pose.position.y = self.y
    odom_msg.pose.pose.position.z = 0.0

    # Convert yaw angle to quaternion
    qx,qy,qz,qw = tf_transformations.quaternion_from_euler(0, 0, self.theta)
    odom_msg.pose.pose.orientation.x = qx
    odom_msg.pose.pose.orientation.y = qy
    odom_msg.pose.pose.orientation.z = qz
    odom_msg.pose.pose.orientation.w = qw

    # Set the velocity
    odom_msg.twist.twist.linear.x = vx
    odom_msg.twist.twist.linear.y = 0.0
    odom_msg.twist.twist.angular.z = wz

    # Publish the odometry message
    self.odom_pub.publish(odom_msg)
```

## **EKF Configuration**

This is the Odom and IMU Configuration for the EKF. The EKF is used to fuse the odometry and IMU data to get a more accurate odometry data. The EKF configuration file is located at `carversavvy_control/config/carversavvy_ekf.yaml`.

```yaml
odom0: carversavvy/wheel/odom
odom0_config: [false, false, false,   # x y z
              false, false, true,   # roll pitch yaw
              true,  true,  false,   # vx vy vz
              false, false, true,    # vroll vpitch vyaw
              false, false, false]   # ax ay az

imu0: /carversavvy/imu_ros
imu0_config: [false, false, false,   # x y z
              false, false, true,    # roll pitch yaw
              false, false, false,   # vx vy vz
              false,  false, true,    # vroll vpitch vyaw
              true,  false, false]    # ax ay az
```

## **Launch Robot with just Rviz2**

To launch the robot in the real world with just Rviz2, run the following command:

```bash
ros2 launch carversavvy_control carversavvy_rviz.launch.py
```

Note: The robot will not move in this case, it is just for visualization purposes.

but if you want to move the robot in the real world, you can run the following command along with the previous command:

```bash
ros2 run carversavvy_control test.py
```

Note: This file is just a test file to move the robot STRAIGHT LINE. (I recommend using  moode 1 only)

Moving the robot with a flexible distance and a 7-segment Trajectory Generation that you can choose by changing the distance variable in the file (self.Dist). Also yu can change the Jmax, Amax, Vmax for the trajectory generation.

```python
# Init Trahectory Profile
if self.mode == 1:
    self.Jmax = 0.2
    self.Amax = 0.1
    self.Vmax = 0.2
    self.Dist = 1.0
```

## **Launch Robot with SLAM**

To collect the map data of the environment, you can use the SLAM package. To launch the robot with SLAM, run the following command:

```bash
ros2 launch carversavvy_control carversavvy_slam.launch.py
```

Note: If you want to use the SLAM to localization. You can go to carversavvy_control/config/mapper_params_online_async.yaml and change the mode setting:

```yaml
# ROS Parameters
odom_frame: odom
map_frame: map
base_frame: base_footprint
scan_topic: /scan
mode: localization
```

Note2: After collect the map data of the environment, you can save the map to the folder `carversavvy_control/maps`. And you can use this map to navigate the robot in the real world.

## **Launch Robot with Navigation**

To navigate the robot in the real world, you can use the Navigation package. To launch the robot with Navigation, run the following command:

```bash
ros2 launch carversavvy_control carversavvy_nav2.launch.py
```

Note: If you want to use your own map for the Navigation to localization. You can go to carversavvy_control/launch/carversavvy_nav2.launch.py and change the mode setting:

```python
slam_map_path = os.path.join(
    get_package_share_directory(package_name),
    'maps',
    'FIBOFL5.yaml')

slam_map_file = LaunchConfiguration('map', default=slam_map_path)
```


## **DRL Agent** 

To run the DRL agent in the real world, you can use the DRL package. To launch the DRL agent, run the following command:

```bash
ros2 launch awbu_drl abwu_test.launch.py
```
Note: For the real world, additional behaviors are added to the DRL agent. which prevents the robot from colliding with obstacles and allows the robot to reach the goal position.

Which contains all the necessary nodes to run the DRL agent in the real world.

(Optional) if you want to add the Nav2 package to the DRL agent, you can use the following command:
```bash
ros2 launch awbu_drl nav2.launch.py
```

Then if you want to control the robot with the DRL agent, you can use the following command:

Example of setting the goal position:

```bash
ros2 service call /abwu_drl_set_goal awbu_interfaces/srv/UserSetGoal "{goal_pose_x: 5.0,goal_pose_y: 0.0}"
```


## **Real world implementation suggestion**

From the implementation of deep reinforcement learning in a real-world environment, it was found that the agent was able to control a robot to reach the desired destination. However, the mismatch between the maximum and minimum linear and angular velocities of the robot in simulation and the real world resulted in some failed episodes where the agent could not avoid certain obstacles. 

Additionally, LiDAR sometimes reflects off surfaces in a way that causes the sensor to misinterpret the presence or absence of obstacles. This issue can result in false positives, where the sensor detects an obstacle that isn't there, or false negatives, where it fails to detect an actual obstacle. These inaccuracies can significantly impact the performance of the agent.

To enhance performance, refine the simulation parameters to better reflect real-world conditions or implement a system that allows agents to train in real-world scenarios. Additionally, improve LiDAR data processing algorithms to more effectively handle reflective surfaces. Utilizing sensor fusion techniques by combining LiDAR with other sensors, such as cameras, can also help mitigate LiDAR limitations and enhance overall obstacle detection and agent performance.
