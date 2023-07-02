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
