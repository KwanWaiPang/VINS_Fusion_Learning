cmake_minimum_required(VERSION 2.8.3) #最小版本号
project(vins)  #项目名字
# project(evio)  #项目名字

set(CMAKE_BUILD_TYPE "Release")  #编译的类型
set(CMAKE_CXX_FLAGS "-std=c++11")  #采用的c++标准
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")  #编译的参数

# 在系统中安装了一些库，而希望在工程中引用一些库，就采用这。
# 找对应的库的config文件，找到之后即可使用库对应的一些头文件、连接库的变量等等
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_models #ins-fusion里面定义的，实现了多种相机模型。
    image_transport)

find_package(OpenCV REQUIRED)

# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)  #采用ceres库来作优化处理

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})  #把上面库的头文件路径给加进来

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

# 要连接的库文件
# 把自己写的vio连接成一个库，然后后面生成的可执行文件vins_node跟这个库相对应
add_library(vins_lib
    src/estimator/parameters.cpp
    src/estimator/estimator.cpp
    src/estimator/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projectionTwoFrameOneCamFactor.cpp
    src/factor/projectionTwoFrameTwoCamFactor.cpp
    src/factor/projectionOneFrameTwoCamFactor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    src/featureTracker/feature_tracker.cpp)
target_link_libraries(vins_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}) #把依赖的第三方库也加进来


add_executable(vins_node src/rosNodeTest.cpp) #找到程序入口点
target_link_libraries(vins_node vins_lib) 

add_executable(kitti_odom_test src/KITTIOdomTest.cpp)
target_link_libraries(kitti_odom_test vins_lib) 

add_executable(kitti_gps_test src/KITTIGPSTest.cpp)
target_link_libraries(kitti_gps_test vins_lib) 

