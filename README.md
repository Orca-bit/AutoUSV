# AutoUSV
### Core Source Code for Our USV:ocean:.

Build top in [ROS Foxy](https://docs.ros.org/en/foxy/index.html):turtle:. Install it by following the official [wiki](https://docs.ros.org/en/foxy/Installation.html).

**Before** build this project, do this in the terminal (bash or zsh):

```bash
export COLCON_DEFAULTS_FILE=/path/to/here/colcon-defaults.yaml
```
and check whether the setting is successful:
```bash
echo $COLCON_DEFAULTS_FILE
```
This will make the compiler work in **release** mode but still yield **debug** info to monitor the compile process.

When using ```colcon build```, you can use this flag to make terminal yield the compile info: 
```bash
--event-handlers console_direct+
```

To just build a single package:
```bash
colcon build --packages-select <package_name>
```

Of course, you can combine these flags:
```bash
colcon build --packages-select <package_name> --event-handlers console_direct+
```

Recommend **Clion** IDE for developing this project.



:stuck_out_tongue_winking_eye: Thanks for :point_right:[AutoWare.Auto](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src), there are many reused codes extracted from AutoWare.Auto. 
