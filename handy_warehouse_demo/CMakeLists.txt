cmake_minimum_required(VERSION 3.0.2)
project(handy_warehouse_demo)

find_package(catkin REQUIRED COMPONENTS)

# warehouse: https://github.com/ivalab/simData/pull/1
catkin_package(CATKIN_DEPENDS roslaunch warehouse)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
