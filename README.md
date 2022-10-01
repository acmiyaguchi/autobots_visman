# autobots visman f22

This repository contains some modules for exploring vision-based manipulation in the Fall 2022 section of the Autobots VIP.

## development

Install the pre-commit hooks:

```bash
pre-commit install
```

Make sure you have the following repositories checked out into your catkin workspace:

```bash
cd src

git clone git@github.com:ivalab/simData.git
git clone git@github.com:ivaROS/ivaHandy.git
git clone --recurse-dubmodules git@github.com:rickstaa/realsense-ros-gazebo.git
git clone git@github.com:machinekoder/ros_pytest.git
```

Then launch one of the demos.

```bash
# ensure you source the setup file for roslaunch
source devel/setup.zsh

roslaunch handy_warehouse_demo empty_world.launch
```

Here's a list of launch files included in this project, building up into tasks of larger complexity.

| project              | name        | description                                              |
| -------------------- | ----------- | -------------------------------------------------------- |
| handy_warehouse_demo | empty_world | setup an empty world in gazebo                           |
| handy_warehouse_demo | hammer      | import a model from the 3d warehouse                     |
| handy_warehouse_demo | arm         | launch a world with the handy arm                        |
| handy_warehouse_demo | camera_box  | launch a world with a realsense camera in front of a box |

## reference

### catkin usage

To create a new project:

```bash
catkin create pkg --rosdistro noetic ${package_name}
```

### ivaHandy (i.e. finalarm)

Not sure why handy is named `finalarm` in the [ivaHandy repo][handy-repo], but we use the naming scheme throughout.
The README has useful documentation regarding what resources need to be online.

We can launch the rviz interface to play with motion planning.

```bash
roslaunch finalarm_moveit_config demo.launch
```

[handy-repo]: https://github.com/ivaROS/ivaHandy

## simData and imgSaver

A non-trivial amount of starting code is pulled from the simData projects.
Contributions are made back where appropriate.

- [simData_imgSaver](https://github.com/ivalab/simData_imgSaver)
  - launch files, some models, and reference for utilizing rospy
- [simData](https://github.com/ivalab/simData)
  - model files for common objects
