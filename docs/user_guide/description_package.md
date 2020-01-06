# Description package
This short tutorial explains what is a description package, why we need it and what it should contain. It also gives an overview about how to create Gazebo world files.

## What is it?
A description package is required to describe the environment in which the robot will operate. This ROS package (whatever its name) must contain all required information regarding the robot's setup. It gathers gazebo **.world** files, as well as all the models required to display it and scenes in order to restrain robot's movement and having proper collision checking (**.scene** files).
The different files should be organized in the three following folders:
```
- models
- worlds
- scenes
```
You can find a description package [here](https://github.com/ARQ-CRISP/ARQ_common_packages/tree/master/arq_description_common).

## Models
The folder *models* must contain at least all the objects used to compose a world, defined as sdf files. It is also **strongly** recommended adding inside the same folder all the objects you want to be able to spawn in gazebo.
The minimal directory structure of `models` is the following:
```
models
+-- <model_using_mesh_name>
|   +-- meshes
|       +-- <model_using_mesh_name>.stl
|   +-- model.config
|   +-- model.sdf
+-- <model_no_mesh_name>
|   +-- model.config
|   +-- model.sdf
```
Please note that the meshes can also be *.dae* files, allowing you to add texture.

## Creating Gazebo world file
The aforementioned models will allow creating Gazebo worlds. It exists several ways of creating gazebo *worlds*, but we encourage to follow the steps described [here](https://shadow-experimental.readthedocs.io/en/latest/user_guide/1_6_software_description.html#creating-a-new-world-scene) (all required dependencies are installed in the docker). If you are not a big fan of GUI and you already have a good hang of how Gazebo world files are working, you can still create your world.

### Creating a Gazebo world without a GUI
The **most** important part is having all the objects you want to add to your world to be in the folder that you have specified for **GAZEBO_MODEL_PATH** and **GAZEBO_WORKSPACE_PATH**. Once this is done, you can just edit the following template:
```xml
<?xml version="1.0" ?>
<!-- Must be a sdf file -->
<sdf version="1.5">
    <world name="default">
        <!-- Add a source of light, which is very important if embedding a camera -->
        <light name='sun' type='directional'>
            <cast_shadows>1</cast_shadows>
            <pose frame=''>0 0 10 0 -0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.5 -1</direction>
        </light>
        <!-- Include one after the other the different objects you want to add -->
        <include>
            <!-- the "model://" refers to the GAZEBO_MODEL_PATH and "my_first_model" refers to the name of the folder containing, the sdf and config file-->
            <uri>model://my_first_model</uri>
            <!-- Whether you want to make the object static (meaning that it will ignore all applied forces) -->
            <static>true</static>
            <!-- Name you want to give to the model -->
            <name>my_first_model</name>
            <!-- Create the reference frame that is going to be used by the pose provided in the model file -->
            <pose>0.0 0.0 0.0 0 0 0</pose>
        </include>
        <!-- model2 -->
        <include>
            <uri>model://my_second_model</uri>
            <static>true</static>
            <name>my_second_model</name>
            <pose>0.0 0.0 0.0 0 0 0</pose>
        </include>
        <!-- Physics parameters -->
        <physics type="ode">
            <gravity>0.000000 0.000000 -9.810000</gravity>
            <ode>
                <solver>
                    <type>quick</type>
                    <iters>100</iters>
                    <precon_iters>0</precon_iters>
                    <sor>1.000000</sor>
                </solver>
                <constraints>
                    <cfm>0.000000</cfm>
                    <erp>0.500000</erp>
                    <contact_max_correcting_vel>1000.000000</contact_max_correcting_vel>
                    <contact_surface_layer>0.00000</contact_surface_layer>
                </constraints>
            </ode>
            <real_time_update_rate>0.000000</real_time_update_rate>
            <max_step_size>0.001000</max_step_size>
        </physics>
        <scene>
            <ambient>0.4 0.4 0.4 1</ambient>
            <background>0.7 0.7 1 1</background>
            <shadows>1</shadows>
        </scene>
        <!-- Display configuration -->
        <gui fullscreen='0'>
            <camera name='user_camera'>
                <pose frame=''>2.8784 4.2586 1.43117 0 0.083643 -2.30699</pose>
                <view_controller>orbit</view_controller>
            </camera>
        </gui>
    </world>
</sdf>
```
Once modified, you can save the file with a *.world* extension, and that's it you have your Gazebo world created!

## Creating scenes files
Scene files gather all the information about the collision scene of the robot. You can either create a simplified version of your environment using primitive shapes (not recommended) or use the *.world* file that you have just created. You can either follow the instructions on this [link](https://shadow-experimental.readthedocs.io/en/latest/user_guide/1_6_software_description.html#creating-a-new-world-scene) or use the framework to create the scene file. In simulation mode, specify the world file corresponding to the scene you want to generate. Once the framework launched go in RViz, if you don't have `Motion Planning` add it. Then go in the tab called `Scene Objects` and click on `Export As Text`. Choose the path to the `scenes` folder of your description package, pick a name and save it. And here is your scene file.
