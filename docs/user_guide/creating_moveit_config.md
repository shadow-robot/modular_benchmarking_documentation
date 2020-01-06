# Creating a Moveit! config package compatible with the framework
If you are reading this, it means that you already have a moveit_config package or you want to create one manually step by step. In the first case you can directly go to this [subsection](##making-the-package-compatible). Otherwise, please follow thoroughly the following steps.

## Copying an existing moveit_config package
Instead of creating a moveit config package from scratch, we are going to start from an existing one. First create an empty folder which name will be the name of the moveit config package (we advise you to make it end with *_moveit_config*) and copy the content of this [repository](https://github.com/ARQ-CRISP/template_moveit_config.git) inside it. You can also initialize a repository if you want to keep it in git. You should now have the following tree in your folder:
```
+-- config
|
+-- launch
|
CMakeLists.txt
package.xml
```

## Changing the package information
In `package.xml` replace all the information in the description, the maintainer, the author and the name. In `CMakeLists.txt`, modify what's inside project. The name inside `project` in the CMakeLists and the name inside `<name> </name>` of the package file **must correspond to the name of your folder!**. We encourage you to just replace `template_moveit_config` by the name of your folder in those two files.<br/>
To make sure that everything is working fine you can run `catkin_make`. It should finish successfully.

## Changing the content of the package to use your robot
Now, you can search all the occurrences of `template_moveit_config` and replace them with your folder's name. Once this is done (it is **very important** to do the previous step first), you can search for all the occurrences of `template` and replace it with the name you want to give to your robot. **Make sure to change the filenames as well**. You can then replace all the `<manipulator_name>` with the name you want to give to your manipulator. The last easy step is to find all the occurrences of `<manipulator_finger_i>` and to change them to the name you want to give for each finger.

## Changing the urdf and sdf file
The first thing to do is to include the proper urdf (or xacro) files of both the arm and the manipulator. If required you can also create arguments to make this urdf file a bit more generic. Change `<base_link_of_the_robot>` with the name of the base link of the robot arm. Define the two robot parts in the urdf file and you are done with it. <br/>
For the srdf file, change the description of each link between `<>` with their real names. For instance of ur arms, `<ee_link_of_robot_arm>` becomes `ee_link`. Make sure to do it everywhere in the file and to add the proper disabled collisions. For this part, we strongly advise to use the moveit setup assistant (and then copy-paste the result).

## Specifying the name of the joints of the robot
The last step consists in specifying the name of the joints for each group we want Moveit to be able to control. Replace the proper values in `controllers.yaml`, `fake_controllers.yaml` and `joint_limits.yaml`.

## Making the package compatible
If you already have a moveit config package or just have created it using the assistant, you just need to add `<rosparam command="load" file="$(find smart_manipulation_framework_api)/config/sensor_plugins.yaml" />` at the top of `sensor_manager.launch.xml`. If you don't, the modification that you bring inside the config files regarding the sensor's plugins won't be considered.

## Testing that everything works
To make sure that everything works, you can run the framework with your newly created moveit package. In order to avoid any error message **make sure** that the name of the ROS controllers declared in `controllers.yaml` match with the controller file that you provide to the framework. <br/>
You should be able to move the robot arm and the manipulator in rviz and see the change in Gazebo.

## It does not work for me...
If for some reason it does not work, you can try to have a look at [this](https://github.com/ARQ-CRISP/arq_ur5_ezgripper_moveit_config) moveit config package and see what are the differences.
