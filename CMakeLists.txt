# Preferable to avoid warnings during execution
cmake_minimum_required(VERSION 3.9)

set(CMAKE_BUILD_TYPE Debug)

# Set to search the root source directory for the FindAriadne.cmake module
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

# Set the C++ standard to C++17, as used by Ariadne
set(CMAKE_CXX_STANDARD 17)

# Find Ariadne as a required dependency
find_package(Ariadne REQUIRED)

# Add the Ariadne headers to the includes
include_directories(SYSTEM ${ARIADNE_INCLUDE_DIRS})

# Set the executable along with the required source files
include_directories(src)
add_executable(boat src/boat.cpp)

# Instruct to link against the ariadne library
target_link_libraries(boat ${ARIADNE_LIBRARIES})
