---
documentclass: article
title: |
  | Autobots VIP Spring 2023
  | Final Report
author: |
  | Anthony Miyaguchi
  | acmiyaguchi@gatech.edu
date: 2023-05-03
colorlinks: true
toc: true
dpi: 300
---

<!---
To generate the PDF for the document, run the following command:

```bash
make report-spr23
```

* Summary of work done
* Description of major code modules written. (links to GitHub repositories)
* What were the two significant points learned?
* Describe what the expectation at the beginning of the class was.
* Describe what you would do with an additional semester to meet or expand expectations.
* Describe what you would do differently to be more efficient about progress if doing the semester over again.
* Link to research journal or attach as an appendix.
-->

\newpage

## Summary

As part of the _Manipulation and Understanding_ sub-team, I worked toward building a pick and place simulation using Handy as the base manipulator.
I updated and refactored Handy ROS packages to work with ROS Noetic and Gazebo and created a Docker environment to facilitate usage.
I reproduced benchmarks from the Grasp Keypoint Network (GKNet) paper using the source code and created ROS modules for inference.
I also build the scaffolding for performing pick-and-place tasks, including calibration, workplace setup, perception, and manipulation.
Relevant code is available on GitHub in standalone repositories or contributed back into various IVALab repositories.

I had the most collaboration with the following students:

- Nicholas Gilpin - Scene Understanding ROS modules
- Ashylnn Zheng - Validating Handy/D435/GKNet behavior in the lab
- Calvin Truong - Validating Handy/D435/GKNet behavior in the lab
- Tawhid Ahmad - Context around Factory Automation efforts

I also had help from graduate students in the lab:

- Runian Xu - Collaboration on simData, Handy, and GKNet pull requests
- Yiye Chen - Camera extrinsic code

## Code Contributions

### ivapylibs/camera

Repository: [ivapylibs/camera](https://github.com/ivapylibs/camera)

- [[ivapylibs/camera] Refactor directory structure of the camera package #4 ](https://github.com/ivapylibs/camera/pull/4)
  - We move the scripts directory out into the root of the package.
    We also renamed some D435 modules and used `mayavi` instead of `pptk`.
- [[ivapylibs/camera] Use find_packages and remove old requirements.txt #5](https://github.com/ivapylibs/camera/pull/5)
  - We use `find_package` and update the `setup.py` to use references to packages via `git+https://`.
    We review all `ivapylibs` packages and update the `setup.py` files to match.
- [[ivapylibs/camera] Update CtoW_Calibrator_aruco with optional aruco_dict parameter #6](https://github.com/ivapylibs/camera/pull/6)
  - This allows estimating the camera extrinsic using user-defined values for the ArUco dictionary.
    The `aruco_dict` parameter is needed because the one used in simulation or physical experiments may not match the hardcoded value in the library.

### ivaROS/ivaHandy

Repository: [ivaROS/ivaHandy](https://github.com/ivaROS/ivaHandy)

- [[ivaROS/ivaDynamixel] Run 2to3 and add docker for testing #3 ](https://github.com/ivaROS/ivaDynamixel/pull/3)
  - This PR enables Handy to run on ROS Noetic and Python 3 by fixing broken serial communication.
    We rewrote the main routine for writing to and from the serial device for string behavior changes in Python 2 vs. 3.
    We resolve this by avoiding using strings and passing a valid byte array/memory view to `pyserial`.
    We use the struct module to decode messages from the serial device.
- [[ivaROS/ivaHandy] Add docker-compose configuration for running Handy #3](https://github.com/ivaROS/ivaHandy/pull/3)
  - This adds documentation and docker configuration for running Handy.
    The containers bind against the host machine as if running on the host itself.
    See the [documentation for more details](https://github.com/ivaROS/ivaHandy/blob/master/docs/docker.md).
- [[ivaROS/ivaHandy] Add noetic base image #5](https://github.com/ivaROS/ivaHandy/pull/5)
  - We add a new default docker image that uses ROS Noetic as the base.
    This image validates the changes made in the ivaDynamixel package in a reproducible environment.
- [[ivaROS/ivaHandy] Update control, description, and gazebo packages for RViz/MoveIt control #6](https://github.com/ivaROS/ivaHandy/pull/6)
  - We fixed mocked Gazebo controllers to work with MoveIt.
    The controllers now use the same namespaces as the physical robot and behave correctly with MoveIt.
    The controller scripts and launch files are also updated to work agnostic to simulation or physical robots.

### ivaROS/GraspKpNet

Repository: [ivaROS/GraspKpNet](https://github.com/ivalab/GraspKpNet)

- [[ivaROS/GraspKpNet] Reproduce training and benchmark results via Docker #3](https://github.com/ivalab/GraspKpNet/pull/3)
  - This PR prepares the repository for ROS integration.
    I rerun the benchmarks of the GKNet paper on available datasets and pre-trained models.
    I rehost datasets on a public Backblaze bucket (see [docs/INSTALL.md](https://github.com/ivalab/GraspKpNet/blob/7114b76f880d9c5bb698318a17c0e784e8289e86/docs/INSTALL.md#downloading-models)).
    I also refactor the repository structure into a proper Python package that is easier to navigate and use externally.
    I created a docker image with an updated PyTorch version and [forked DCNv2 code to build without a GPU](https://github.com/acmiyaguchi/DCNv2/commit/44743797c656fc940523b51432f2f1b1ea5caa59).
    I also create a docker-compose configuration for running the benchmarks.
- [[ivaROS/GraspKpNet] Add ROS module for GKNet perception and inference #4](https://github.com/ivalab/GraspKpNet/pull/4)
  - I added a ROS module to process images into their grasp key points.
    It will dump out a series of key points and an annotated image.
    The pick-and-place code transforms the grasps into the world coordinate frame.
- [[ivaROS/GraspKpNet] Add functionality to rank grasping poses on a per object basis #5](https://github.com/ivalab/GraspKpNet/pull/5)
  - This refactors the code to separate annotation into a node and adds filter-ranked poses per object.
    I've added a small OpenCV-based utility to stream image topics (which works better than image view via docker) and manually draw rectangles.
    ![Showing poses in a test scene.](images/report_spr23/annotated_image.png){ width=60% }

### ivalab/simData

Repository: [ivalab/simData](https://github.com/ivalab/simData)

- [[ivalab/simData] Generate preprocessed model files for imgSaver via catkin project #1](https://github.com/ivalab/simData/pull/1)
  - I refactor a Matlab script that generates model SDF files into the Catkin build process via CMake.
    The new build script exports the generated SDF files.
    We can reference the SDF files from a launch file using the `find_package` macro.
    We use this package extensively from the pick-and-place repository.
- [[ivalab/simData] Add collision/physics to rendered models #2 ](https://github.com/ivalab/simData/pull/2)
  - With @ruinianxu's help in enabling physics, I also add collision to the model SDF using model meshes.
    The patch allows better simulation of items and the manipulator in Gazebo.

### Autobots-Visman/segmentation

Repository: [Autobots-Visman/segmentation](https://github.com/Autobots-Visman/segmentation)

- [[Autobots-Visman/segmentation] Add skeleton of packages with initial unit tests and docker configuration #1](https://github.com/Autobots-Visman/segmentation/pull/1)
  - I helped add boilerplate code to the segmentation/perception code for the sub-team.
    The PR uses idiomatic ROS module patterns, including basic testing mechanisms.

### Autobots-Visman/pick-and-place

Repository: [Autobots-Visman/pick-and-place](https://github.com/Autobots-Visman/pick-and-place)

The Autbots-Visman/pick-and-place repository is the primary deliverable for the Spring 2023 semester.
It contains a (mostly) functional demo that integrates GKNet and Handy.
It falls short of the pick-and-place goal but demonstrates smaller system components.

![General architecture of modules/nodes involved in the pick-and-place simulation.](images/slides_spr23_02_pick_and_place/architecture.png){width=100%}

- [ros/autobots_realsense2_description@02e12b2146](https://github.com/Autobots-Visman/pick-and-place/tree/02e12b21468fdd74739e8fc88897a893caf8b636/ros/autobots_realsense2_description)
  - This package contains a modified version of urdf files for the realsense2 camera.
    The main difference in this model is that we can turn off gravity for the camera link.
    The modified urdf is useful for simulating the camera in a fixed position.
    The included launch file will configure the topics and align the depth image to the color image.
- [ros/autobots_calibration@02e12b2146](https://github.com/Autobots-Visman/pick-and-place/tree/02e12b21468fdd74739e8fc88897a893caf8b636/ros/autobots_calibration)
  - This package implements a calibration node that uses `ivapylibs/camera` to compute camera extrinsic with a single ArUco on a workspace.
    It publishes information to tf2 and a latched calibration topic.
    The package is agnostic to running in simulation or on a physical robot.
- [ros/autobots_handy_simulation@02e12b2146](https://github.com/Autobots-Visman/pick-and-place/tree/02e12b21468fdd74739e8fc88897a893caf8b636/ros/autobots_handy_simulation)
  - This package contains the Gazebo simulation for the Handy manipulator.
    It includes a launch file that will spawn the robot, tabletop, and camera in a fixed position.
    It also includes the control scripts for moving the manipulator based on GKNet inference.

## Major Points Learned

### Software engineering fluency is valuable when synthesizing a system

My time this semester in the VIP taught me the value of having professional software engineering experience under my belt.
I have a significant amount of systems experience, be it Linux systems administration, build-system configuration, or distributed systems.
It helped me identify various workflow issues in the lab that came between me and a physical pick-and-place simulation using Handy.
I was comfortable diving deep to resolve issues with serial communication, software differences between major revisions of ROS, and existing robotics software found across various repositories.

It was nice to be recognized for my help when resolving issues, especially in other folks' workflows.
For example, I helped a student in the lab work use a docker and a newer version of ROS to reduce overhead with the Mary surveillance robot.
Many of my pull requests were less about functional change and more about reducing the cognitive burden for people interested in using the software in the future.

It was also relatively straightforward to build non-trivial modules in ROS because I've had exposure to large codebases and have seen many of the patterns used in the framework.
I had fun while building many of these modules.
Still, I recognize that it would have been a more frustrating experience without the fluency needed to assemble the various subsystems.

### Remote collaboration without access to the hardware is challenging

Working on robotics without a robot is very hard.
I work best when I have an accurate mental model of how all of the software I touch interacts.
I only had this mental model once I saw Handy in person during my February Atlanta trip.
Up to this point, it was difficult to understand how code needed to be structured to solve particular manipulator tasks.
After seeing the robot in action and the software needed to run, it was much easier to see the path forward.

After leaving Atlanta, I found the small interactions with people in the lab made it easier to communicate with people online.
I learned that pairing with people over a voice call was an efficient use of time and helped the other person get over what I consider a high barrier to entry.
Meeting people made it easier to merge pull requests, e.g., simData and GraspKpNet PRs.
Conversely, it was difficult when the code I wrote on my computer did not work on the lab computer for one reason or another.
Various issues around Docker, CUDA driver versions, and `xauth` caused problems.
It is challenging to figure out the best way to write code for someone else to run, without having access to the actual hardware.

It's been a valuable learning experience working both in-person and online, taking away unexpected perspectives.

## Expectations

My experience from the Fall tempered my expectation for the VIP this semester.
I was ready to be the only online graduate student in the course and to push along at my own pace.
I had been planning to visit Atlanta since the end of the Fall semester, so I expected that I would have an opportunity to play around with some of the manipulators in the lab.

I set up three goals from the last semester:

- Finish the "VisMan Learning Adventures" tutorial series and build out skeleton code as an exercise for future students.
- Collaborate with the Multi-Robot Coordination team on the Factory Automation task, particularly around Gazebo simulations.
- To perform a literature review and pose and research problem.

These goals were a valid waypoint, but what happened in practice differed from the plan.
I was actually one of four OMSCS students, and I had a surprising number of interactions with students online and in-person.
I also dove deeper into the depths of software development and did not have much time to help out with Factory Automation or to complete the "VisMan Learning Adventures" as-is.
Overall, my VIP experience this semester differed from my initial expectations, but I gained valuable skills and unexpected opportunities for collaboration.

## What If: Additional Semester

I would focus on completing the pick-and-place demo with Handy and GKNet if I had another semester to expand or improve my contributions to the VIP.
There are slight differences in scope depending on whether I have one or three credit units to spend.

I plan on taking another semester with one credit unit this Fall, which involves finishing integrating the simulation components.
I had a fruitful experience collaborating with a few other students as they ran the simulation code in the lab and piece-meal tested various components.
Concretely, here are a few of the tasks needed:

- Validating the calibration module against a physical RGB-D camera and workspace.
- Validating GKNet against a physical RGB-D camera with a variety of objects.
- Validating TF transforms of grasp key points against calibration extrinsic with manipulator trajectories.
- Implement more realistic weights and center of gravity on simulated tools.
- Implement unit tests for calibration and simulation modules.
- Implement simulated grasping with a manipulator.
- Implement target location to move manipulator.

The overall goal is to complete a physical pick-and-place experiment.
Most of the written code for running the physical experiments (launch files, scripts, documentation) would come from students with access to the lab.
The secondary goal would be to integrate the manipulator simulation into the broader context of an assistive task, such as Factory Automation.
However, this is contingent on having a suitable code for simulating the broader environment.

If I had another three-credit unit semester, I would also dive deeper into improving model deployment for GKNet.
In particular, GKNet requires a GPU with CUDA, which limits experimentation inside a simulation.
Spending about a third of the time working on model inference optimization and distillation would be interesting.
We can deploy modern vision models to single-board computers.
It would also be helpful to deploy Handy/GKNet modules to a small computer.
In addition, I would also bootstrap other subgroups with ROS/Gazebo boilerplate that they can use for development.
For example, the Factory Automation team has a few scripts for controlling TurtleBots that are limited to execution on a physical robot.
The lack of simulation lengthens the development loop and makes it more challenging to realize more exciting demonstrations.

## What If: Redo Semester

If I had to redo the semester, I could have been more efficient about progress by spending more time on boilerplate code for other teams and dropping the in-progress work I had from last semester.
My critical juncture this semester was the time I spent in Atlanta in February.
This time let me see the state of the lab and interact with students in the VIP and the lab.
It was valuable for knowing the robot's environment and software stack and helped me figure out the most impactful work.

Knowing what I learned in the lab, I would drop experimental work following the "VisMan Learning Adventures".
The tutorial was a great starting place to understand ROS and Gazebo, but it was divorced from practical code bases.
I spent the first four weeks of the semester generally aimless in this regard.

The second thing I would do upfront to be more efficient is to set up various repositories that other students can work out of.
For example, the Factory Automation team started a small repository with some scripts for ArUco tag detection.
It would have been helpful to configure this with what I consider helpful boilerplate:

- A ROS module for the main functionality
- A testing harness for unit tests
- A Dockerfile and docker-compose.yml configuration for testing

It's hard to see how else I could have been more efficient in choosing impactful work.
This semester has been fruitful, if not ending on a somewhat unsatisfying conclusion.

## Research Journal

### Trello

See the [Trello card for Anthony's Spring 2023 Work](https://trello.com/c/WIigwsry/24-anthonys-spring-2023-work) for general progress this semester.

### Progress Presentations

Progress presentations effectively capture most of the work in the research journal in two-week increments from the semester's midpoint.
These presentations provide a more comprehensive overview of my work in the larger team context.

You can find the presentations in the appendix or linked below:

- [2023-03-14 - VisMan Progress Presentation 1](https://github.com/acmiyaguchi/autobots_visman/blob/3ee15467bcbe4355984047d13afc7912da45bb41/docs/rendered/slides_spr23_00_docker.pdf)
- [2023-04-04 - VisMan Progress Presentation 2](https://github.com/acmiyaguchi/autobots_visman/blob/3ee15467bcbe4355984047d13afc7912da45bb41/docs/rendered/slides_spr23_01_handy_sim.pdf)
- [2023-04-18 - VisMan Progress Presentation 3](https://github.com/acmiyaguchi/autobots_visman/blob/3ee15467bcbe4355984047d13afc7912da45bb41/docs/rendered/slides_spr23_02_pick_and_place.pdf)

\newpage

## Appendix

Slides for the progress presentations are attached to the end of this report.
