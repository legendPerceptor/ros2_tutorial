# ROS 2 tutorial

This tutorial repo serves as quick review notes for ROS 2 basic concepts. I use ROS 2 Jazzy on Ubuntu 24.04, but the concepts should be identical/similar for other ROS 2 versions. If you have no experience with ROS2, it is recommended to get started with the [official tutorial](https://docs.ros.org/en/jazzy/Tutorials.html). This repo is good for reviewing concepts and quickly getting started for commands that you have a vague idea but not sure about the full parameters.

## 1. ROS 2 basic concepts

ROS 2 have some concepts that are crucial to understand: nodes, topics, services, parameters, actions, and launch files. This section will cover these concepts and practical code in both Python and C++ to control the famous turtle in ROS.

> If you have not installed ROS 2 on your computer, go to [Ubuntu (Debian packages)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html) for Ubuntu 24.04. If you are using other Linux distributions, I believe you know how to install ROS 2 already.

We will use the `turtle_sim` to learn how to write a publisher and a subscriber for some topics.

We will write a complete controller to control the turtle.


## 2. Robot URDF and visualization

