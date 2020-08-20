# Integrate a sensor
This page contains all information required to add a new sensor to GRIP.

## Must have list
To follow this tutorial, make sure you have the following components:
- Launch file running the sensor
- Initial pose of the sensor

## Before starting the framework
1. Open a new terminal (ctrl+shift+t), and clone the ROS package containing the launch file in `/home/user/projects/shadow_robot/base/src`
2. Run `cd projects/shadow_robot/base/; catkin_make; source devel/setup.bash`

## Tutorial
1. Start the framework: `roslaunch modular_framework_api start_framework.launch` and **DO NOT PRESS** the `Launch robot` button.
2. Specify the number of sensors you want to integrate in the robot interface
3. Go to `Settings` and press `New` in the `Sensors config` editor.
4. Click on the + and enter the name you want to give to your sensor. A template with all the fields to fill should appear.
5. Fill the different fields. For the RealSense sensor the final configuration should look like this:
```yaml
RealSense:
    data_topics:
        pointcloud: /camera/depth/color/points
        depth_map: /camera/depth/image_rect_raw
        rgb: /camera/color/image_raw
    initial_pose:
        frame_id: camera
        reference_frame: ee_link
        position: {x: 0.08, y: -0.05, z: 0.05}
        orientation: {r: 3.141592654, p: 0, y: 0}
```
Note that the orientation can be changed to quaternion and the `initial_pose` can be set to a simple string that would denote the name of a pose that you've already set in the pose editor. The `data_topics` can be filled by only one element (doesn't have to be exactly these 3 keys).
6. Go in the terminal and open a new tab (ctrl+shift+t) and run the launch file starting the sensor
7. Finish the robot configuration and launch the robot
