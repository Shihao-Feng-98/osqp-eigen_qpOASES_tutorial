cmake_minimum_required(VERSION 3.12)
project(QPOASES_TUTORIAL)
set(CMAKE_BUILD_TYPE "Debug")
# g++编译选项 
set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3 -std=c++11 -Wall") 

# 添加头文件路径 -I
include_directories(/usr/local/include) # qpOASES库路径

# -L 
link_directories(/usr/local/lib) # .so file path

# 输出可执行文件 
add_executable(main main.cpp)

# 链接库 -l
target_link_libraries(main PRIVATE 
                    libqpOASES.a    # qpOASES静态库
)
