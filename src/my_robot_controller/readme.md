# ROS 2 with Python basics

To create a simple node, check out [first_node.py](my_robot_controller/first_node.py).

Make sure you add the correct dependencies in the [package.xml](package.xml) file. They usually look like this.

```xml
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>turtlesim</depend>
<depend>example_interfaces</depend>
```

Because Python language does not require compilation, once we add the entry points in the `setup.py`, we can use sim link build to avoid running colcon build everytime after changing the code. We can go to the ROS 2 workspace and run the following command.

```bash
colcon build --symlink-install
```

After colcon build, remember to source the setup.bash file in the install folder.

```bash
source install/setup.bash
```

The [draw_circle_publisher.py](my_robot_controller/draw_circle_publisher.py) will show you how to create a simple String publisher that publishes to the `robot_news` topic and how to publish to the topic `/turtle1/cmd_vel` to control the turtle.

To test the functionality, you need to run the following commands.

```bash
ros2 run turtlesim turtlesim_node
ros2 run my_robot_controller draw_circle_publisher
```

And you can check the String information is published to the `robot_news` topic. You should also see the turtle is running in a circle.

```bash
ros2 topic echo robot_news
```

The [turtle_controller.py](my_robot_controller/turtle_controller.py) will show you how to control the turtle by subscribing to the `/turtle1/pose` topic and publishing to the `/turtle1/cmd_vel` topic. We will also use a service `/turtle1/set_pen` to change the pen color so that the tutrle's path will be of difference color based on its location.


