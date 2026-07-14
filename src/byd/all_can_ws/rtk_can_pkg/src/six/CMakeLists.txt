# 指定CMake编译器的最低版本
cmake_minimum_required(VERSION 3.10)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)

# 设置工程名（某些cmake版本不支持project() - 千方科技）
set(PROJECT_NAME "six")
project(${PROJECT_NAME})

# 设置项目路径
set(proPath ${CMAKE_SOURCE_DIR})

# 设置项目版本号
set(sdk_VERSION "1.1.3.61")

# 添加编译选项 -- 安全编译相关
# 设置C语言标准为gnu99
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON) # 启用GNU扩展，相当于 -std=gnu99

# 设置C++标准（如需要）
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-fstack-protector-all)
add_compile_options(-D_FORTIFY_SOURCE=2)
add_compile_options(-z now)

# 安全版本需要打开tlslib
include_directories(${proPath}/inc)
include_directories(${proPath}/include)
link_directories(${CMAKE_SOURCE_DIR}/lib)
link_directories(${CMAKE_SOURCE_DIR})

add_compile_options(-fPIE -pie)

