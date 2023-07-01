# ChatManipulators

```
ros2 topic pub /joint_states sensor_msgs/msg/JointState "header:
  stamp: $(date +%s)+%N)
    sec: $(date +%s)+%N)
    nanosec: $(date +%N)
  frame_id: ''n_joint'
name:oulder_pan_joint''
- 'shoulder_pan_joint''
- 'shoulder_lift_joint'
- 'elbow_joint't'
- 'wrist_1_joint'
- 'wrist_2_joint'
- 'wrist_3_joint'
position:
- 4.2
- 0.02
- 5.12
- 3.32
- 2.23
- 1.56
effort: []"]
```
