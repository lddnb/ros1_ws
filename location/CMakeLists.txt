cmake_minimum_required(VERSION 3.16)
project(location)

#debug还是release对pcl_ndt速度影响巨大
set(CMAKE_BUILD_TYPE release)
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

#这句在ros中不需要，ros有制定位置
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

add_compile_options(-std=c++14)
add_definitions(-std=c++11)
set(CMAKE_CXX_STANDARD 11)

include_directories(${PROJECT_SOURCE_DIR}/include)

include(cmake/global_definition.cmake)

configure_file(
  ${PROJECT_SOURCE_DIR}/cmake/config.h.cmake
  ${PROJECT_BINARY_DIR}/include/common/config.hpp)

#Glog
find_package(Glog REQUIRED)
#include_directories(${GLOG_INCLUDE_DIR})

#protobuf
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})

#lua
find_package(Lua 5.2 REQUIRED)
include_directories(${LUA_INCLUDE_DIR})

#Eigen3
#include_directories(${PROJECT_SOURCE_DIR}/third_party/eigen3)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#PCL
find_package(PCL 1.10 REQUIRED)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${PCL_INCLUDE_DIRS})
message("PCL path:", ${PCL_INCLUDE_DIRS})

#GeographicLib
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/GeographicLib)
include_directories(${PROJECT_SOURCE_DIR}/third_party/GeographicLib/include)

#catkin
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    pcl_ros
    tf2_ros
)
include_directories(${catkin_INCLUDE_DIRS})

#这句用于指定输出位置，必不可少，即使空参数
catkin_package()

set(ALL_TARGET_LIBRARIES 
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBS}
    ${PCL_LIBRARIES}
    ${LUA_LIBRARIES}
    ${PROTOBUF_LIBRARY}
    #${GLOG_LIBS}
    glog::glog
    libGeographiccc
)

#TODO(lua->pb.h)
file(GLOB_RECURSE ALL_PROTOS "${PROJECT_SOURCE_DIR}/*/*.proto")
message(STATUS "Proto Source Files: ${ALL_PROTOS}")
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)
foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")
  #PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${ABS_FIL})

  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROJECT_BINARY_DIR} -I
        ${PROJECT_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
  message("pb.h = " ${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h)
  message("PROTO_HDRS" ${PROTO_HDRS})
endforeach()

include_directories(${PROJECT_BINARY_DIR})
include_directories(${PROJECT_BINARY_DIR}/include)

message("PROJECT_BINARY_DIR = " ${PROJECT_BINARY_DIR})
message("PROJECT_SOURCE_DIR = " ${PROJECT_SOURCE_DIR})

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
#file(GLOB_RECURSE MAKE_FILE "CMakeCXXCompilerId.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})
#list(REMOVE_ITEM ALL_SRCS ${MAKE_FILE})


message("ALL_SRCS = " ${ALL_SRCS})

# 必须要在生成执行文件或库文件的时候添加proto的cc文件，编译时才能生成cc和pb.h文件
add_executable(test_frame_node src/test_frame_node.cpp ${ALL_SRCS} ${ALL_PROTO_SRCS})
target_link_libraries(test_frame_node ${ALL_TARGET_LIBRARIES})

add_executable(front_end_node src/front_end_node.cpp ${ALL_SRCS} ${ALL_PROTO_SRCS})
target_link_libraries(front_end_node ${ALL_TARGET_LIBRARIES})
