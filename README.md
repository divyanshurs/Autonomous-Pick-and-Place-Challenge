# MEAM520: Pick and Place Challenge 2021
<!-- ### Date Created: 08/26/2021 -->

<!-- Contributors: Divyanshu Sahu, Vanshil Shah, Yug Ajmera, Anokhee Choksi -->




<p align="center">
  <img src="/labs/pickandplacereallife.gif" />
</p>


[Results Video](https://www.youtube.com/watch?v=pp5F3g9YzWU&t)

[Final project report](/labs/final/MEAM520_Final_Project.pdf)

[Final project code](/labs/final/final.py)

# High level overview of the functionality developed (`lib`)
-  `calculateFK.py`: Calculates the end effector position w.r.t world frame for a given joint configuration of the manipulator [Code](/lib/calculateFK.py) | [Detailed Explanation](/labs/lab1/MEAM520_Lab1_Submission.pdf)
-  `calcJacobian.py`: Calculates the Jacobian Matrix for a given joint configuration of the manipulator [Code](/lib/calcJacobian.py) | [Detailed Explanation](/labs/lab2/meam520_lab2_Sub.pdf)
- `IK_velocity.py`: Calculates the joint velocities for achieving a particular end effector velocity in a given joint configuration [Code](/lib/IK_velocity.py) | [Detailed Explanation](/labs/lab2/meam520_lab2_Sub.pdf)
- `solveIK.py`: Calculates joint configuration for achieving particular end effector pose [Code](/lib/solveIK.py) | [Detailed Explanation](/labs/lab3/MEAM_520_LAB3.pdf)
- `rrt.py`: Rapidly-exploring random tree planner in joint space for traversing between two joint configurations [Code](/lib/rrt.py) | [Detailed Explanation](/labs/lab4/meam520_lab4.pdf)

# Subdirectory Structure:
- `core`: contains support code we provide to you, such as helper functions and interfaces for interacting with the robot (either simulated or real!)
- `lib`: contains functions implementing algorithms to solve computational problems such as forward and inverse kinematics and trajectory planning
- `labs`: contains test scripts that use the algorithms you implement in `lib` and the tools we provide in `core` to control the robot and achieve tasks
- `ros`: contains code necessary to launch the simulator. 

# Native Install Instructions 



## Operating System

The simulator must be run on Ubuntu 20.04 with ROS noetic installed. You can follow the standard installation instructions for [Ubuntu 20.04](https://phoenixnap.com/kb/install-ubuntu-20-04) and [ROS noetic](http://wiki.ros.org/noetic/Installation).

## panda_simulator installation

To get started using this development environment, you must first follow the instructions to install [panda_simulator](https://github.com/justagist/panda_simulator), a Gazebo-based simulator for the Franka Emika Panda robot. The only difference is that you will name the catkin workspace `meam520_ws` to avoid conflicts with other projects.

The instructions specify to use `catkin build`, but we recommend building with `catkin_make_isolated` instead.

Once you've completed that installation, add

```
source ~/meam520_ws/devel_isolated/setup.bash
```

to your `~/.bashrc` to source the workspace. If all has been done correctly, you should be able to run

```
roslaunch panda_gazebo panda_world.launch
```

to launch the Gazebo simulation.


### Speed up Gazebo shutdown

Since you may be launching and killing Gazebo many times , we recommend reducing the clean shutdown wait time for a better experience. Edit the file:
```
cd /opt/ros/noetic/lib/python3/dist-packages/roslaunch
sudo vim nodeprocess.py
```
and change the lines that say
```
...
_TIMEOUT_SIGINT = 15.0 #seconds

_TIMEOUT_SIGTERM = 2.0 #seconds
...
```
to `2.0` and `1.0` respectively.


## Update Python path to find core module

Add the following line to your ~/.bashrc to allow your python scripts to find the meam520 core modules.

```
export PYTHONPATH="${PYTHONPATH}:/home/${USER}/meam520_ws/src/meam520_labs"
```

## meam520_labs installation

Once you have a forked, cloned, and set the upstream for the git repository run `catkin_make_isolated` again.

Once the build finishes, open a new terminal and run:

```
roslaunch meam520_labs lab0.launch
```

To check that the installation is successful, run a demo script:

```
cd ~/meam520_ws/src/meam520_labs/labs/lab0
python demo.py
```

You should see the robot in Gazebo moving.



