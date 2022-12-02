# CMakeLists.txt

`CMakeLists.txt`原本是Cmake编译系统的规则文件，而Catkin编译系统基本沿用了CMake的编译风格，只是针对ROS工程添加了一些宏定义。所以在写法上，catkin的`CMakeLists.txt`与CMake的基本一致。

CMake参考资料

- [cmake-practice.pdf](https://github.com/Akagi201/learning-cmake/blob/master/docs/cmake-practice.pdf)
- [解析CMakeList.txt文件](https://dlonng.com/posts/ros-cmakelist)

这个文件直接规定了这个package要依赖哪些package，要编译生成哪些目标，如何编译等等流程。所以`CMakeLists.txt`非常重要，它指定了由源码到目标文件的规则，catkin编译系统在工作时首先会找到每个package下的`CMakeLists.txt`，然后按照规则来编译构建。

## turtlesim的CMakeLists

以turtlesim小海龟这个pacakge为例，`roscd tuetlesim`查看，在`turtlesim/CMakeLists.txt`的写法如下:

```cmake
cmake_minimum_required(VERSION 2.8.3)
#CMake至少为2.8.3版

project(turtlesim)
#项目(package)名称为turtlesim，在后续文件中可使用变量${PROJECT_NAME}来引用项目名称turltesim

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    rosconsole
    roscpp
    roscpp_serialization
    roslib
    rostime
    std_msgs
    std_srvs
)
#cmake宏，指定依赖的其他pacakge，实际是生成了一些环境变量，如<NAME>_FOUND, <NAME>_INCLUDE_DIRS, <NAME>_LIBRARYIS
#此处catkin是必备依赖 其余的geometry_msgs...为组件

find_package(Qt5Widgets REQUIRED)
find_package(Boost REQUIRED COMPONENTS 
    thread
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
#指定C++的头文件路径
link_directories(${catkin_LIBRARY_DIRS})
#指定链接库的路径

add_message_files(DIRECTORY msg FILES
    Color.msg
    Pose.msg
)
#自定义msg文件

add_service_files(DIRECTORY srv FILES
    Kill.srv
    SetPen.srv
    Spawn.srv
    TeleportAbsolute.srv
    TeleportRelative.srv
)
#自定义srv文件

generate_messages(DEPENDENCIES geometry_msgs std_msgs std_srvs)
#在add_message_files、add_service_files宏之后必须加上这句话，用于生成srv msg头文件/module，生成的文件位于devel/include中

catkin_package(CATKIN_DEPENDS 
    geometry_msgs
    message_runtime
    std_msgs
    std_srvs
)
# catkin宏命令，用于配置ROS的package配置文件和CMake文件
# 这个命令必须在add_library()或者add_executable()之前调用，该函数有5个可选参数：
# (1) INCLUDE_DIRS - 导出包的include路径
# (2) LIBRARIES - 导出项目中的库
# (3) CATKIN_DEPENDS - 该项目依赖的其他catkin项目
# (4) DEPENDS - 该项目所依赖的非catkin CMake项目。
# (5) CFG_EXTRAS - 其他配置选项

set(turtlesim_node_SRCS
    src/turtlesim.cpp
    src/turtle.cpp
    src/turtle_frame.cpp
)
set(turtlesim_node_HDRS
    include/turtlesim/turtle_frame.h
)
#指定turtlesim_node_SRCS、turtlesim_node_HDRS变量

qt5_wrap_cpp(turtlesim_node_MOCS ${turtlesim_node_HDRS})

add_executable(turtlesim_node ${turtlesim_node_SRCS} ${turtlesim_node_MOCS})
# 指定可执行文件目标turtlesim_node
target_link_libraries(turtlesim_node Qt5::Widgets ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# 指定链接可执行文件
add_dependencies(turtlesim_node turtlesim_gencpp)

add_executable(turtle_teleop_key tutorials/teleop_turtle_key.cpp)
target_link_libraries(turtle_teleop_key ${catkin_LIBRARIES})
add_dependencies(turtle_teleop_key turtlesim_gencpp)

add_executable(draw_square tutorials/draw_square.cpp)
target_link_libraries(draw_square ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(draw_square turtlesim_gencpp)

add_executable(mimic tutorials/mimic.cpp)
target_link_libraries(mimic ${catkin_LIBRARIES})
add_dependencies(mimic turtlesim_gencpp)
# 同样指定可执行目标、链接、依赖

install(TARGETS turtlesim_node turtle_teleop_key draw_square mimic
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# 安装目标文件到本地系统

install(DIRECTORY images
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")
```
