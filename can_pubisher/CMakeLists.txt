
cmake_minimum_required(VERSION 3.0.2)
project(can_publisher)

## 设置依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  can_msgs
)

## 声明此包的依赖
catkin_package()

## 设置包含的头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 编译可执行文件或节点
add_executable(can_publisher_node src/can_publisher.cpp)

## 链接所需的库
target_link_libraries(can_publisher_node ${catkin_LIBRARIES})




    return 0;
}


