cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(ROS_BUILD_STATIC_LIBS true)

set(IPC ${PROJECT_SOURCE_DIR}/ipc-3.8.6)

# TODO: Parse through ipc's makefile to find when these should be defined
# using capabilities, instead of statically
ADD_DEFINITIONS(-DREDHAT_52 -DREDHAT_6 -DREDHAT_71)

# ipc library
rosbuild_add_library(ipc ${IPC}/src/ipc.c ${IPC}/src/queryResponse.c ${IPC}/src/marshall.c ${IPC}/src/timer.c ${IPC}/src/list.c ${IPC}/src/behaviors.c ${IPC}/src/tcModError.c ${IPC}/src/datamsg.c ${IPC}/src/formatters.c ${IPC}/src/hash.c ${IPC}/src/idtable.c ${IPC}/src/key.c ${IPC}/src/primFmttrs.c ${IPC}/src/reg.c ${IPC}/src/sendMsg.c ${IPC}/src/tcaMem.c ${IPC}/src/tcaRef.c ${IPC}/src/comModule.c ${IPC}/src/com.c ${IPC}/src/globalM.c ${IPC}/src/globalMUtil.c ${IPC}/src/strList.c ${IPC}/src/modLogging.c ${IPC}/src/modVar.c ${IPC}/src/resMod.c ${IPC}/src/parseFmttrs.c ${IPC}/src/lex.c ${IPC}/src/printData.c)

# central
rosbuild_add_executable(central ${IPC}/src/comServer.c ${IPC}/src/dispatch.c ${IPC}/src/msgTap.c ${IPC}/src/recvMsg.c ${IPC}/src/res.c ${IPC}/src/tcerror.c ${IPC}/src/logging.c ${IPC}/src/globalS.c ${IPC}/src/centralIO.c ${IPC}/src/globalVar.c ${IPC}/src/centralMain.c)
target_link_libraries(central ipc)

