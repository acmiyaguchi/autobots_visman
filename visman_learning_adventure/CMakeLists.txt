cmake_minimum_required(VERSION 3.0.2)
project(visman_learning_adventure)

find_package(catkin REQUIRED COMPONENTS)

# warehouse: https://github.com/ivalab/simData/pull/1
catkin_package(
    CATKIN_DEPENDS
        roslaunch
        gazebo_ros
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/camera_box.test)
endif()
