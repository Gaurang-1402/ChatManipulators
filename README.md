# ChatManipulators

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
effort: []"]
```
