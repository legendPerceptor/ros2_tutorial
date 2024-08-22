# URDF and gazebo simulation

To simulate the robot in Gazebo, we first need to use the robot_state_publisher to convert the xacro file into a URDF file.

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro my_robot.urdf.xacro)"
```

We can then obtain the URDF file from the parameter.

```bash
ros2 param get /robot_state_publisher robot_description
```

Alternatively, we can prepare the converted URDF file beforehand.

```bash
xacro my_robot.urdf.xacro > converted_my_robot.urdf
```

We use `ros2 launch ros_gz_sim gz_sim.launch.py` to start the gazebo.


> The old way to spawn an entity in gazebo classic is `ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_robot`. We cannot use this method in the current version of gz sim.


We use a service call to spawn an entity in the gazebo environment. It is good to grab a correct URDF/Xacro file from the Internet to ensure that the problem is not in the URDF file.

We convert the Xacro file into a URDF file with the following command.

```bash
xacro ./rrbot.xacro > rrbot_converted.urdf
```

We use the following command to load the URDF into gazebo.

```bash
gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/yuanjian/Development/ros2_course_ws/src/my_robot_description/urdf_online/rrbot_converted.urdf", name: "my_model"'
```

To insert our basic-shaped car model into the scene, run the following command. Make sure the `ros2 launch ros_gz_sim gz_sim.launch.py` command is run under the `src/my_robot_description` folder.

```bash
gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "urdf/converted.urdf", name: "my_model"'
```