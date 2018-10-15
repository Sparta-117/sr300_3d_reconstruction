cmake_minimum_required(VERSION 2.8.3)
project(build_model)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++0x")

############### dependencies ######################
# 添加cmake模块以使用g2o
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
#g2o
find_package( G2O REQUIRED )
include_directories(${G2O_INCLUDE_DIRS})
LIST(APPEND G2O_LIBS
  cxsparse
  cholmod
  g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
  )
#CSparse
find_package( CSparse REQUIRED )
include_directories(${CSPARSE_INCLUDE_DIR})
#Opencv
find_package(OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIR})
#PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
# Eigen
include_directories( "/usr/include/eigen3" )
# Sophus 
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
#ros
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
)
include_directories(${catkin_INCLUDE_DIRS})

#self file
include_directories(../../devel/include)
include_directories(${PROJECT_SOURCE_DIR}/include/build_model)

############### execute ######################
set(SRC src/camera.cpp src/config.cpp src/qrplane.cpp src/optimize.cpp)

#add_executable(build_model_ftf src/BuildModel_ftf.cpp ${SRC})
# add_executable(build_model src/main.cpp ${SRC})
add_executable(build_model2 src/main_all_images.cpp ${SRC})
#add_executable(get_image1 src/get_image.cpp)

# target_link_libraries(build_model
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#    ${Sophus_LIBRARIES}
#    ${CSPARSE_LIBRARY}
#    ${G2O_LIBS}
#  )
target_link_libraries(build_model2
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
   ${Sophus_LIBRARIES}
   ${CSPARSE_LIBRARY}
   ${G2O_LIBS}
 )
# target_link_libraries(get_image1
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${PCL_LIBRARIES}
# )






