find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()


PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS joint_angles.proto)
add_library(joint_angle_msgs SHARED ${PROTO_SRCS})
target_link_libraries(joint_angle_msgs ${PROTOBUF_LIBRARY})

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS joint_efforts.proto)
add_library(joint_effort_msgs SHARED ${PROTO_SRCS})
target_link_libraries(joint_effort_msgs ${PROTOBUF_LIBRARY})

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS joint_velocities.proto)
add_library(joint_velocity_msgs SHARED ${PROTO_SRCS})
target_link_libraries(joint_velocity_msgs ${PROTOBUF_LIBRARY})

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS sim_variables.proto)
add_library(sim_variable_msgs SHARED ${PROTO_SRCS})
target_link_libraries(sim_variable_msgs ${PROTOBUF_LIBRARY})

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS image_data.proto)
add_library(image_data_msgs SHARED ${PROTO_SRCS})
target_link_libraries(image_data_msgs ${PROTOBUF_LIBRARY})
