cmake_minimum_required(VERSION 3.12)
project(OSQP_EIGEN_TUTORIAL)
set(CMAKE_BUILD_TYPE "Debug")
# g++编译选项 
# Note: -march=native 导致OsqpEigen运行出现段错误
set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3 -std=c++14 -Wall") 

find_package(OsqpEigen) # 找OsqpEigen库
find_package(Eigen3) # 找Eigen3库

# 添加头文件路径 -I
include_directories(${EIGEN3_INCLUDE_DIR} # Eigen库路径
)

# 输出可执行文件 
add_executable(main main.cpp)

# 链接库 -l
target_link_libraries(main PRIVATE 
                    OsqpEigen::OsqpEigen # OsqpEigen库
)
