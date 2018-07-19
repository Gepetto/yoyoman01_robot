cmake_minimum_required(VERSION 2.6)

project(sot_yoyoman01_bringup)

find_package(catkin REQUIRED)
catkin_package()

#set(PROJECT_NAME sot_talos_bringup)
#set(PROJECT_DESCRIPTION ROS package for Stack of Tasks on Talos)
#set(PROJECT_URL "")

#setup_project()

INSTALL(
  FILES 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

INSTALL(
  FILES
  launch/geometric_simu.launch
  launch/geometric_simu_context.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

INSTALL(
  FILES
  sot/yoyoman01.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/sot
)

INSTALL(
  FILES
  package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

#setup_project_finalize()

