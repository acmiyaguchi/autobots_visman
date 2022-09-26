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
git@github.com:ivaROS/ivaHandy.git
```

Then launch one of the demos.

```bash
# ensure you source the setup file for roslaunch
source devel/setup.zsh

roslaunch handy_warehouse_demo empty_world.launch
roslaunch handy_warehouse_demo hammer.launch
```

## reference

Here are a few handy commands.

To create a new project:

```bash
catkin create pkg --rosdistro noetic ${package_name}
```
