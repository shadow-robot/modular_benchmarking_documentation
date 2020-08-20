# Real pick and place

In the last task, you have run a pick and place between two constant and pre-defined locations. Let's try to spice things up and integrate a RGB-D sensor that would provide the input data required to run a Grasping algorithm that generates the pose where the robot needs to pick the object. The object will be dropped at the location of your choice (still among the provided list)

**Add a Kinect2 and integrate an external grasping algorithm to run a realistic pick and place scenario.**

**Note:**
- In python you can get the depth image coming from the kinect using `depth_image_message = rospy.wait_for_message("simulated_kinect2/depth_image", Image)` if you use these import lines: `from sensor_msgs.msg import Image` and `import rospy`.

## Related documentation
* [Integrate a sensor](./integrate_sensor.md)
* [Wrap existing code inside a service](./wrap_to_service.md)
* [Integrate external component](./integrate_external_scripts.md)
