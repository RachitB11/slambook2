# Minimum cmake version required
cmake_minimum_required(VERSION 2.8)

# Name of the project
project(HelloSLAM)

# Type of the cmake type (Set to "release" if you want to build without debugging for release)
set(CMAKE_BUILD_TYPE "Debug")

# Make a simple executable with no dependencies
# Format：add_executable( name cpp file with a main ）
add_executable(helloSLAM helloSLAM.cpp)

# Libraries are just a collection of algos. This allows you to import them.
# Making a static library (.a file)
add_library(hello libHelloSLAM.cpp)
# Making a shared library (.so file)
add_library(hello_shared SHARED libHelloSLAM.cpp)

# Make an executable for this script that has dependencies
add_executable(useHello useHello.cpp)
# Link the directories
target_link_libraries(useHello hello_shared)
