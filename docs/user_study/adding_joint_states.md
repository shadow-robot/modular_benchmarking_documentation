# Adding pre-recorded joint states

GRIP allows you to set an unlimited number of pre-recorded joint states, that once loaded to the framework can be used in the task editor.
To do so, you need to go to the `Settings` tab. Assuming that you have not already stored them to the proper format, you can hit the `New` button. Once you have specified where you want the generated YAML file to be saved, you will be able to add as many joint states as you wish. If you already have such a file then you can click on `Open` and its content will appear in the editor. The expected format is:
```yaml
<joint_state_name_1>:
  <joint1_name>: <value>
  <joint2_name>: <value>
  ...

<joint_state_name_2>:
    <joint1_name>: <value>
    <joint2_name>: <value>
    ...

...
```
If you have already pressed the `Launch robot` button, you need to load this new information to the framework by clicking on `Robot > update settings`. If you haven't, then these joint states will be automatically loaded when the robot starts. Now you will be able to specify joint states to the robot as many times as you want using directly the name you set.
