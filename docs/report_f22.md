---
documentclass: article
title: |
  | Autobots VIP Fall 2022
  | Final Report
author: |
  | Anthony Miyaguchi
  | acmiyaguchi@gatech.edu
date: 2022-12-11
colorlinks: true
toc: true
---

<!---
To generate the PDF for the document, run the following command:

```bash
pandoc -s -o rendered/report_f22.pdf report_f22.md camera.md
```

* What are the initial expectations of the project at the start?
* What was done as an overview?
* Provide details for one achievement.
* What remains to be done and would be done if given three more months
* Thoughts on the experience? (What was learned?)
* Where to find documented activities in Trello or Team OneDrive
-->

# Initial Expectations

I joined Autobots Vertically Integrated Project (VIP) this fall because I wanted to get an opportunity to try out research as part of my master's studies at Georgia Tech.
It is my second semester in the Online Masters in Computer Science (OMSCS) program, so I have been cautious not to over-commit the workload since I am working full-time in addition to my studies.
I signed up for one credit hour to gauge whether it would be worth spending three credit hours on the project next semester.
I knew I wouldn't be able to build anything substantial this semester, but it would lay down a foundation for potential future work.

My goals for this semester were to understand the problem space for assistive autonomous robots and to clearly articulate what challenges exist in a more constrained and concrete task.
As a remote graduate student, I knew I would be primarily semi-independent with my learning experience and work.
I expected hands-on experience with the toolchain for simulating a robotic task.
I was also interested in collaboration and interaction with other students in the section, but I knew this would take a lot of work to do remotely.

# Overview of Progress

I also chose to work on Vision-Based Manipulation (VisMan).
The semester primarily focused on working through the [VisMan Learning Adventures][learning-adventures] in the wiki.
I've been able to run a manipulator (both Edy and Handy), and camera (RealSense D435) in Gazebo.
I wrote functional unit tests to verify the camera's functionality.
I have not progressed beyond tabletop calibration.

The first few weeks of the semester involved installing Linux, ROS, and Gazebo on my computer.
I chose to install ArchLinux, and as a result spent time filing issues and making small pull requests to fix bugs in upstream dependencies.
I learned about ROS and models by making a few patches to the [simData] and [simData_imgSaver][imgsaver] repositories under the IVALab organization.
I could load models used in the experiments and learned how to export models for downstream catkin projects.

I configured a camera and an environment to demonstrate the pick-and-place task for the remainder of the semester.
To understand camera calibration, I had to read through the first three weeks of [ECE4850] notes.
I was also able to create a surface to be used for tabletop calibration.
I leave the completion of this task to next semester.

[learning-adventures]: https://pvela.gatech.edu/classes/doku.php?id=ece4560:visman:adventures
[imgsaver]: https://github.com/ivalab/simData_imgSaver
[simdata]: https://github.com/ivalab/simData
[ece4850]: https://pvela.gatech.edu/classes/doku.php?id=ece4580:start

# Notable Achievement

One of the most significant milestones this semester was building a catkin package that includes unit tests to verify the functionality of an RGB-D camera in a simulated environment.
The ability to unit-test that a camera is notable because it requires several prerequisites.
First, we can install and run arbitrary ROS packages from the source.
These are installed by cloning git repositories into the workspace or using the system package manager (i.e., `pacman` via the [AUR]) to install it on the path.
We added the following packages to our workspace:

```bash
git clone --recurse-submodules git@github.com:rickstaa/realsense-ros-gazebo.git
git clone git@github.com:machinekoder/ros_pytest.git
```

After installing the packages, we can verify the camera in Gazebo and RViz.

```bash
roslaunch realsense2_description view_d435_model_rviz_gazebo.launch
```

![d435 gazebo rvis](images/2022-09-28_d435-gazebo-rviz.png)

We add a red box to the environment so that we have something to observe.
We add a new `sdf` model to the project directory.
We write a script to obtain color and depth images, which runs on delay as a node in a launch file for testing the camera.

![bridge block](images/2022-09-29_opencv-bridge-block.png)

To wrap everything together, we create a test directory that takes advantage of catkin tests and the `pytest` framework.

```XML
<!-- tests/camera_box.test -->
<launch>
  <include file="$(find handy_warehouse_demo)/launch/camera_box.launch">
    <arg name="GUI" value="false" />
  </include>

  <param name="test_module" value="$(find handy_warehouse_demo)/tests" />
  <test
    test-name="camera_box"
    pkg="ros_pytest"
    type="ros_pytest_runner"
    args="-k camera_box -vvv -s"
    time-limit="60.0"
  />
</launch>
```

Because it's just a launch file, we can include other launch files to set up the environment before running the test node.
The test does two things: first, it checks that the camera can detect red pixels from the OpenCV bridge, and second, the number of red pixels monotonically increases as the box moves toward the camera.

```bash
$ catkin test --this

[handy_warehouse_demo.rosunit-camera_box/test_camera_can_see_red_box][passed]
[handy_warehouse_demo.rosunit-camera_box/test_camera_red_increases_when_box_moves_closer][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0
```

These are simple tests, but verifies a valid environment and correct behavior.
This milestone was significant because it helped me build a good mental model around how to build software in ROS and Gazebo from the ground up.

Refer to the report appendix or the [camera documentation on GitHub][camera-github] for more details.

[aur]: https://aur.archlinux.org/
[camera-github]: https://github.com/acmiyaguchi/autobots_visman/blob/main/docs/camera.md

# Future Work

Here are a few things I'd like to do if I had three more months to work on this project.
The ultimate goal is to build foundational knowledge that I might use in the future as an academic or professional and to have concrete deliverables that I can present at the Spring 2023 OMSCS showcase.

## VisMan Learning Adventure Skeleton Code

I want to finish the VisMan learning adventure and consolidate my work into a skeleton that can serve as an exercise for a future student.
The exercises in the wiki are challenging (and achievable).
However, it would aid learning and ramp-up to have a starting point for the tasks that require an understanding in robotics or computer vision theory.
A skeleton project would formalize the exercises with enough code to get started quickly. For example, consider the following task in the camera section of the learning series:

> Connect to the depth camera and display the streaming RGB and depth images.

We might provide a future student with an empty launch file with a python node that runs a script that displays an empty plot in a loop.
The student would then have to complete the following:

1. Add the appropriate ROS nodes to instantiate a camera and ensure it writes to the ROS topic.
2. Add the appropriate code to read from the OpenCV bridge and show the image through the screen via `matplotlib`.

One challenge with this approach is that modules build on each other, which makes it difficult to write later modules without spoiling solutions for earlier modules.
I need to finish the series for myself before I'm sure how best to simplify skeleton code while leaving the conceptual challenges intact.

## Collaboration with the Multi-Robot Coordination Team

One of the main goals established this semester was to work toward a pick-and-place task.
Pick-and-place is one of the fundamental tasks of a vision-based manipulator, and it provides a relatively good context for understanding research topics in the area.

It would be nice to collaborate with the multi-robot coordination team, such as building a simulation environment that they could use for the manipulation portion of their payload delivery task.
There is existing work from previous semesters to build [a model of the fourth floor of the TSRB building][tsrb-world], which is a helpful starting point.

[tsrb-world]: https://trello.com/c/VWUTJQsR/87-fletcher-2022sp

## Literature Review and Posing a Research Problem

The last thing I want to do is a literature review.
I've gotten a good sense of ROS and Gazebo, so I want to expand my knowledge and understanding of the state-of-the-art in the vision-based manipulation space.
After reading a few papers, I would like to propose a research problem.
I would deliver this as a presentation or a report with a summary of techniques for a particular research problem.
If time permits, I would like to study or reproduce an implementation of an algorithm using skills that I've developed in earlier portions of the project.
I will likely move on to focus on the rest of my master's program after the end of the school year, but it would be an interesting exercise to think through what it would take to solve a problem in the area.

# Thoughts on Experience

The discussion sessions were an excellent opportunity to learn a variety of problem spaces and solutions from the literature.
Out of the presentations, I enjoyed topics about planning and navigation the most, such as the graph-based planner for cave exploration and sequential scene understanding and manipulation (SUM).
I enjoyed learning about different approaches and gaining a breadth around autonomous robotics.
I was one of the first and last presenters, so my presentation was heavily focused on what I had built, but it would be interesting to do a research oriented presentation too.
As a side-note, it would be nice to consolidate all the paper references into a single document reference.

I also enjoyed building out the catkin project and learning ROS and Gazebo.
It was challenging, but I now feel comfortable reading and modifying other packages for my own use cases.
I also had the opportunity to contribute to open-source projects like [eigenpy].
I also appreciated design choices made with ROS, coming in with the perspective of a professional software engineer.
Building out my package from scratch was an excellent exercise for learning about robotics software development.

I spent most of the budgeted time working on the learning adventures and got stuck a few times.
After spending a few hours reading through the documentation, forums, issue trackers, and source code, I resolved the issues myself.
However, it would have been nice to have the opportunity to ask questions in office hours.
It was challenging to reach out with questions about office hours in TSRB because I am remote.
Regardless, the project's structure was a good fit for me since I could work and learn at my own pace.
I was initially concerned about being the only remote member of the group and the only graduate student.
The Zoom meetings and Trello board worked well for me; I could attend the discussion sessions synchronously and update my learning progress asynchronously.

[eigenpy]: https://github.com/stack-of-tasks/eigenpy/pull/321

# Documentation

I have put all relevant documentation and code on GitHub at [acmiyaguchi/autobots_visman][autobots_visman].
It is the canonical source for work that I have done in Autobots.

My progress can be tracked in the [Fall 2022 VisMan planning card in Trello][trello-main].
The two primary tickets that I worked on this semester were the ["Add demo of the virtual camera with readings (color/depth)"][trello-camera] and ["Model the task workspace plane"][trello-workspace] cards.
These cards have links to the relevant GitHub issues and pull requests.
Other tickets in the central planning card capture other miscellaneous work that I've done, such as [pull requests I've made to ivaHandy, ivaEdy, and the simData repositories][iva-simdata-card].

The [2022-09-20 presentation][ppt-visman-update-01] summarizes work toward the beginning of the semester.
The [2022-11-29 presentation][ppt-visman-update-02] summarizes work up to the end of the semester.
Both of these presentations are found on the shared Autobots drive.

[autobots_visman]: https://github.com/acmiyaguchi/autobots_visman
[trello-main]: https://trello.com/c/tvmoPzOO/5-plan-visman-2022-fall
[trello-camera]: https://trello.com/c/yJuIosj3/18-add-demo-of-virtual-camera-with-readings-color-depth
[trello-workspace]: https://trello.com/c/z2yTkCzZ/27-model-the-task-workspace-plane
[iva-simdata-card]: https://trello.com/c/fsrCGKDL/9-play-with-models-code-in-simdataimgsaver-and-ivahandy-with-respect-to-a-functional-opencv-bridge
[github-main]: https://github.com/acmiyaguchi/autobots_visman
[ppt-visman-update-01]: https://gtvault.sharepoint.com/:p:/s/autobots/EaTXlF0kwWNBo1Zf35Ygq2QBB0arncl0zyV2ch4FLRPE_g?e=gDlwTw
[ppt-visman-update-02]: https://gtvault.sharepoint.com/:p:/s/autobots/ERxN89TxRkhGvtmhWGJIsMUBok4YUV84KV4ikzVIXGKzAg?e=8LUt9T

\newpage
\section{Appendix}
