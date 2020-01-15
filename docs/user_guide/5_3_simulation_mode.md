# Interacting with the simulated environment
Most of the time, simulation is being used to either experiments some methods without involving the physical robot or to gather data. For both scenarios, you may want to interact with the environment by adding or deleting one or more objects. For this purpose we provide a simple interface to spawn and remove objects in Gazebo that will be automatically updated the collision scene.

## Prerequisite
The framework can only spawn objects that are described in a [sdf file](http://gazebosim.org/tutorials?tut=build_model). We strongly advise to store all the objects that you are supposed to spawn in the [description package](./2_1_description_package.md).

## Using the api package
The easiest way to spawn or delete objects is to use the provided [launch file](???) and set the following arguments:
* `delete_object`: Boolean stating whether the specified object must be deleted. Default is `false`.
* `object_name`: Optional string specifying the name that you want to give to the object to spawn. It is very useful when you want to spawn several times the same objects. **If you don't you will not be able to spawn twice the same object**. Default is `""`, meaning that you can spawn the given object only once. To respawn it you will need to delete it first.
* `object_type`: Optional string specifying the type of the object. It must correspond to the name of one of the folder in the description package corresponding to an object (in *models*). Default is `""`.
* `reference_frame`: Optional string specifying the name of the frame that should be used as reference for the spawned object. Default is `world`.
* `object_position`: Optional string specifying the position where the object should be spawned. Format should be `x,y,z` (in meters). For instance `0.2,0.5,0.9`. Default is `0,0,0`
* `object_rpy_orientation`: Optional string specifying the orientation of the spawned object. Format should be `r,p,y` (in radians). Default is `0,0,0`.
* `file_path`: Optional string containing the path to a sdf file containing the description of the object to spawn. Default is `""`.

Please note that you **must** provide **either** `file_path` or `object_type` in order to spawn an object. When deleting an object you just need to specify its name (and obviously set `delete_object` to `true`). If you spawned it without naming it, the default name is `object_type` or `file_path`. <br/>
When adding an object **the Allowed Collision Matrix is automatically updated** and by default, the robot **can't enter in collision with the object**. If you want to allow a collision, please use the different services provided by the [proper manager](./5_1_managers.md).

## Using the node from the core
If you prefer using the node that is wrapped in the api package, you can directly call the node `manage_object.py` that can be found in the `modular_framework_core` package. <br/>
You can specify the different options using the following arguments:
* `-d` or `--delete` followed by `true` or `false`.
* `-p` or `--position` followed by the position (same format as before)
* `-o` or `--orientation` followed by the orientation (same format as before)
* `-n` or `--name` followed by the name given to the object
* `-t` or `--type` followed by the type of object
* `-r` or `--reference` followed by the name of the reference frame
* `-f` or `--file` followed by the path to the sdf file describing the object to spawn

You can find the source [here](???) for more information about the arguments.
