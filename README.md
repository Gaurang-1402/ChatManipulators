# ChatManipulators: Control Manipulators with Natural Language

ChatManipulators is a project that merges the capabilities of Language Logic Models (LLMs) with the control of robotic manipulators. With it, users can **command manipulators using simple, everyday language.** The project uses the ROS2 (Robot Operating System) Humble and runs simulations in the Gazebo environment, providing a reliable platform for manipulator arm behavior.

ChatManipulators is complemented by a user-friendly web application, offering a streamlined interface for inputting commands. This combination of sophisticated technology with a user-centric interface enhances the overall usability, making the control of complex robotic manipulators as straightforward as possible.

Currently, ChatManipulators operates primarily within a simulated environment for robust testing and development. However, its design and architecture cater for real-world application, ready to translate everyday language into precise manipulator actions when required.

Key features include:

- A user-friendly web application providing an interactive interface for robot manipulator control

- Commands to randomize or bring the robot arm to equilibrium

[![output-onlinegiftools-4-1.gif](https://i.postimg.cc/NGkV1WYC/output-onlinegiftools-4-1.gif)](https://postimg.cc/MXXdw3n1)

- Complete control over the manipulator's joints, allowing 5 DoF (Degrees of Freedom) commands for "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", and "wrist_3_joint"

[![output-onlinegiftools-5-1.gif](https://i.postimg.cc/3wQCfxFX/output-onlinegiftools-5-1.gif)](https://postimg.cc/gx4Znd5j)

- Support for commands in multiple languages

[![output-onlinegiftools-6-1.gif](https://i.postimg.cc/Ssvn5ZbB/output-onlinegiftools-6-1.gif)](https://postimg.cc/DJ1f4PZ6)

## ROSGPT Architecture

![Screenshot from 2023-07-04 20-49-50](https://github.com/Gaurang-1402/ChatDrones/assets/71042887/f3534fd5-1ac8-4d55-8e67-fb5f6c0ddf8d)


## Getting started

Clone the repository

```
mkdir ros_ws
cd ros_ws
git clone <repo_url>
```

Install rosgpt libraries from the rosgpt folder

```
cd ~/src/rosgpt
pip3 install -r requirements.txt
```

Install ROS requirements

```
sudo apt-get install python-rosdep
sudo rosdep init
rosdep update
```

```
cd ~/ros_ws
rosdep install --from-paths src --ignore-src --rosdistro=<rosdistro> -y
```


Add your OpenAI API Key in your ```.bashrc``` as an environment variable 

```
echo 'export OPENAI_API_KEY=your_api_key' >> ~/.bashrc
```


## Running ROSGPT

First, navigate to the root directory of your workspace and build the project

```
cd ~/ros_ws
colcon build --symlink-install
```
Now run each of these commands on new terminals

```
source install/setup.sh
ros2 run rosgpt rosgpt
```

```
source install/setup.sh
ros2 run rosgpt rosgpt_client_node 
```

```
source install/setup.sh
ros2 run rosgpt rosgptparser_manipulator
```

## Running the simulation

```
source install/setup.sh
ros2 launch ur_description view_ur.launch.py ur_type:=ur5

```

Note: Please replace `<repository_url>` and `<your_api_key>` with the actual repository URL and your OpenAI API key, respectively.


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
