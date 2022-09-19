cmake_minimum_required(VERSION 3.0) # 指明CMAKE最小版本要求
project(EIEGN_TUTORIAL) # 定义工程名称

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall") # g++编译选项

set(CMAKE_BUILD_TYPE Debug) # 编译类型 Debug/Release

include_directories(/usr/include/eigen3) # Eigen库路径

add_executable(main main.cpp) # 生成可执行文件
