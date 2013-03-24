cmake_minimum_required(VERSION 2.8.3)
project(jpeg_streamer)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


find_package(catkin REQUIRED COMPONENTS sensor_msgs roscpp image_transport)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#add_message_files(DIRECTORY msg FILES ...)
#uncomment if you have defined services
#add_service_files(DIRECTORY srv FILES ...)

#generate_messages()

include_directories("include")

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES jpeg_streamer
   CATKIN_DEPENDS sensor_msgs roscpp image_transport
#  DEPENDS system_lib
)

#common commands for building c++ executables and libraries
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(jpeg_streamer src/jpeg_streamer.cpp src/mongoose.c)

target_link_libraries(jpeg_streamer ${catkin_LIBRARIES})
target_link_libraries(jpeg_streamer ${Boost_LIBRARIES}) 