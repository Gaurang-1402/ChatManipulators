# ChatManipulators


## Panda robot

```
ros2 topic pub /joint_states sensor_msgs/msg/JointState "header:
  stamp:
    sec: $(date +%s)
    nanosec: $(date +%N)
  frame_id: ''
name:
- 'shoulder_pan_joint'
- 'shoulder_lift_joint'
- 'elbow_joint'
- 'wrist_1_joint'
- 'wrist_2_joint'
- 'wrist_3_joint'
position:
- 4.2
- 0.0
- 5.1
- 3.32
- 2.23
- 1.56
effort: []"

```

Useful ones

```
ros2 launch panda_description view.launch.py 

```

```
ros2 topic echo /joint_group_effort_controller/joint_trajectory 

```


Useless ones

```
ros2 launch panda_moveit_config move_group.launch.py 

```

```
ros2 launch panda_description view_gz.launch.py
```

```
ros2 launch panda_description view_ign.launch.py
```

## Credits
Simulation adapted from: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description

```
@article{koubaa2023rosgpt,
  title={ROSGPT: Next-Generation Human-Robot Interaction with ChatGPT and ROS},
  author={Koubaa, Anis},
  journal={Preprints.org},
  year={2023},
  volume={2023},
  pages={2023040827},
  doi={10.20944/preprints202304.0827.v2}
}

```
I am deeply appreciative of these individuals/teams for sharing their work to build on top of!
