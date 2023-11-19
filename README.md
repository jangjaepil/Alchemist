## Requirement  

1. Ubuntu Bionic Beaver(18.04) or Focal(20.04)
2. ROS melodic or noetic

## Installation

### 1. clone franka_ros pkg in the /src folder
```bash
  git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros
```

### 2. install universal-robot
```bash
  sudo apt-get install ros-$ROS_DISTRO-universal-robots
```

### 3. install trac-ik
```bash
  sudo apt-get install ros-$ROS_DISTRO-trac-ik
```

### 4. build & source devel/setup.bash
```bash
  catkin_make
  source devel/setup.bash
```
## Usage

### 1. run franka panda gazebo tutorial

### example mode  

```bash
  roslaunch franka_gazebo panda.launch x:=-0.5 world:=$(rospack find franka_gazebo)/world/stone.sdf controller:=cartesian_impedance_example_controller rviz:=true
```

### demo mode  

```bash
  roslaunch franka_panda_description panda.launch 
```

```bash
  rosrun panda_controller impedance
```

```bash
  rostopic echo /panda/end_effector_global_pose 
```

### 2. run franka_unity launch file
- you can launch end_pose_pub to pub franka panda's end effort pose to unity.
  ```bash
    roslaunch franka_unity end_pose_pub.lanuch
  ```

- you can launch joint_state_pub to pub franka panda's joint_state pose to unity.
  ```bash
    roslaunch franka_unity joint_state_pub.lanuch
  ```

- you can launch ur5_ik_pub to pub Joint state value of ur5 obtained through inverse kinematics using franka panda's end pose to unity. 
  ```bash
    roslaunch franka_unity ur5_ik_pub.launch 
  ```
