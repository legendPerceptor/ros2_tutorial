# ROS 2 tutorial

This tutorial repo serves as quick review notes for ROS 2 basic concepts. I use ROS 2 Jazzy on Ubuntu 24.04, but the concepts should be identical/similar for other ROS 2 versions. If you have no experience with ROS2, it is recommended to get started with the [official tutorial](https://docs.ros.org/en/jazzy/Tutorials.html). This repo is good for reviewing concepts and quickly getting started for commands that you have a vague idea but not sure about the full parameters.

## 1. ROS 2 basic concepts

ROS 2 have some concepts that are crucial to understand: nodes, topics, services, parameters, actions, and launch files. This section will cover these concepts and practical code in both Python and C++ to control the famous turtle in ROS.

> If you have not installed ROS 2 on your computer, go to [Ubuntu (Debian packages)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html) for Ubuntu 24.04. If you are using other Linux distributions, I believe you know how to install ROS 2 already.

All the code for this section will be under `src/my_robot_controller` (for Python) and `src/my_robot_controller_cpp` (for C++).

> Because ROS 2 development requires many terminal windows. I recommend you to install Terminator by `sudo apt install terminator`, which is easier to use than tmux.

**Key knowledge points**

- We will create a basic node with timer callback.

- We will use the `turtle_sim` to learn how to write a publisher and a subscriber for some topics.

- We will write a complete controller to control the turtle.

- We will create a launch file to run everything in one command.

First of all, you need a ROS 2 workspace. It can be any empty folder, and you will create an `src` folder inside it. Let's assume the empty folder is called `ros2_course_ws`, and you should have only one folder `src` in it. Run `colcon build` under the `ros2_course_ws`, and it should succeed and will create two folders `install` and `build`. Add the following two lines to your `~/.bashrc` (modify the path accordingly).

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/your/ros2_course_ws/install/setup.bash
```

To create a C++ package, go into the `src` folder and run the following command.

```bash
ros2 pkg create my_robot_controller_cpp --build-type ament_cmake --dependencies rclcpp
```

If you are using VSCode, add the include path `/opt/ros/jazzy/include/**` to your workspace's `c_cpp_properties.json` under `"includePath":`, so that you have autocompletion and proper code highlight.

For C++ executables, the executable's name is determined by CMakeLists.txt in the `add_executable()` command. For Python, the executable's name is defined in the `setup.py` file under `entry_points`.

To create a Python package, go into the `src` folder and run the following command.

```bash
ros2 pkg create my_robot_controller --build-type ament_cmake --dependencies rclpy
```

If you want to use some other Python packages, installing the packages to the system python is NOT recommended. In addition, ROS 2 does NOT work well with conda virtual environment. The simpliest way to use a Python virtual environment with ROS 2 is using the following command.

```bash
python3 -m venv ros2_venv
source ros2_venv/bin/activate
```

To learn details about the Python code, please visit the [readme.md](src/my_robot_controller/readme.md) in the `src/my_robot_controller` folder.

To learn details about the C++ code, please visit the [readme.md](src/my_robot_controller_cpp/readme.md) in the `src/my_robot_controller_cpp` folder. 


## 2. Robot URDF and visualization

The code for this section will be in `src/my_robot_description`. Please navigate to the [readme.md](src/my_robot_description/readme.md) inside src/my_robot_description for more detailed information.

We will be using Gazebo Sim (not the classic Gazebo), so follow the [official installation guide](https://gazebosim.org/api/sim/8/install.html) to install it. For Ubuntu 24.04 users, you can just run the following commands.

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
```

Then just install gazebo. This software is separate from ROS 2 but it has great integration with ROS 2, and that's why we chose this software for robot simulation.

```bash
# I used version 8, but you should be able to use TAB to see what version is available.
sudo apt install libgz-sim8-dev
```

You would also need xacro for URDF files and ros-gz-sim pakcage. Xacro enhances URDFs, allowing you to define complex macro and reuse code with function calls. The ros-gz-sim package allows gazebo to interact with ROS 2.

```bash
sudo apt install ros-jazzy-xacro ros-jazzy-gz-sim
```

