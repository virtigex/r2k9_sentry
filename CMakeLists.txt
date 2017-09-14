# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package.
project(r2k9_sentry)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs std_srvs rosbag)
find_package(Boost COMPONENTS filesystem system REQUIRED)
find_package(message_generation)


set(CMAKE_CXX_FLAGS "-std=c++11" ${CMAKE_CXX_FLAGS})
#set(CMAKE_CXX_FLAGS "-fpermissive" ${CMAKE_CXX_FLAGS})

#catkin_python_setup()
add_message_files(FILES TrainingReady.msg)
add_message_files(FILES TrainingUpdate.msg)
generate_messages(DEPENDENCIES std_msgs)

# Declare our catkin package.
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs rosbag message_runtime
)

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files.
add_executable(r2k9_sentry r2k9_sentry.cpp utils.cpp)
add_dependencies(r2k9_sentry ${catkin_EXPORTED_TARGETS})

# Specify libraries against which to link.
target_link_libraries(r2k9_sentry ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${rosbag_LIBRARIES} )

