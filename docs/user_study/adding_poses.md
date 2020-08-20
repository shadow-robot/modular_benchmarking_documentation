# Adding pre-recorded poses

GRIP allows you to set an unlimited number of pre-recorded poses, that once loaded to the framework can be used in the task editor.
To do so, you need to go to the `Settings` tab. Assuming that you have not already stored them to the proper format, you can hit the `New` button. Once you have specified where you want the generated YAML file to be saved, you will be able to add as many poses as you wish. If you already have such a file then you can click on `Open` and its content will appear in the editor. The expected format is:
```yaml
<pose_name>:
  reference_frame: <frame_name>
  position: {x: <x_val>, y: <y_val>, z: <z_val>}
  orientation: {r: <r_value>, p: <p_value>, y: <y_value>}
```
If you have the orientation in quaternion and not in the RPY system, you can change `{r: <r_value>, p: <p_value>, y: <y_value>}` to `{x: <x_value>, y: <y_value>, z: <z_value>, w: <w_value>}`.
Note that clicking on the + symbol will directly add the template for you in the editor.
If you have already pressed the `Launch robot` button, you need to load this new information to the framework by clicking on `Robot > update settings`. If you haven't, then these poses will be automatically loaded when the robot starts. Now you will be able to specify poses to the robot as many times as you want using directly the name you set.
