cmake_minimum_required(VERSION 2.8.3)
project(ur5_vs)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    roslib
    geometry_msgs
    image_transport
    cv_bridge
    visualization_msgs
    sensor_msgs
    tf
    actionlib
    control_msgs
    message_generation
    pcl_ros
    pcl_conversions
    joint_state_publisher
    robot_state_publisher
    object_detection
  )

find_package(OpenCV)
#find_package(PCL REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_msgs)


find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include(FindBoost)

include_directories(
${GAZEBO_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIR}
${PCL_INCLUDE_DIR}
include)

find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
find_package(Protobuf REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_LIBRARIES} /home/raina_pc/ur_vs_gazebo/src/ur5_vs/msgs)
link_directories(${GAZEBO_LIBRARY_DIRS} ${catkin_LIBRARIES} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
#add_subdirectory(msgs)
link_directories(${GAZEBO_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_message_files(
   DIRECTORY msgs FILES
   joint_vel.msg
   sim_variables.msg
joint_angles.msg
joint_states.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ur5_ik
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
    geometry_msgs
    visualization_msgs
    message_runtime
)

#IBVS with moveit
#add_executable(eyeinhand_6dof_tracking src/eyeinhand_6dof_tracking.cpp include/my_funcs.h)
#target_link_libraries(eyeinhand_6dof_tracking ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

#IBVS without moveit
#add_executable(eyeinhand_ibvs src/eyeinhand_ibvs.cpp include/my_funcs.h)
#target_link_libraries(eyeinhand_ibvs ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

#IBVS with PID control
#add_executable(ibvs_PID src/ibvs_PID.cpp include/my_funcs.h)
#target_link_libraries(ibvs_PID ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#add_executable(ibvs_new_PID src/ibvs_new_PID.cpp include/my_funcs.h)
#target_link_libraries(ibvs_new_PID ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

#IBVS with 2 features (By Deepak Raina)
#add_executable(ibvs_two_features src/ibvs_two_features.cpp include/my_funcs.h)
#target_link_libraries(ibvs_two_features ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

#IBVS with 2 features (By Deepak Raina)
#add_executable(new_ibvs_two_features src/new_ibvs_two_features.cpp include/my_funcs.h)
#target_link_libraries(new_ibvs_two_features ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

#add_executable(new_ibvs_two_features src/new_ibvs_two_features.cpp include/my_funcs.h)
#target_link_libraries(new_ibvs_two_features ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

#add_executable(new_ibvs_two_features src/new_ibvs_two_features.cpp include/my_funcs.h)
#target_link_libraries(new_ibvs_two_features ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})


#add_executable(gazebo_ibvs_two_features src/gazebo_ibvs_two_features.cpp include/my_funcs.h)
#target_link_libraries(gazebo_ibvs_two_features ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${PCL_LIBRARIES} ${GAZEBO_libraries} ${roscpp_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES}
#pthread joint_angle_msgs joint_velocity_msgs sim_variable_msgs image_data_msgs)

#IBVS with 3 features (By Deepak Raina)
#add_executable(new_ibvs_three_features src/new_ibvs_three_features.cpp include/my_funcs.h)
#target_link_libraries(new_ibvs_three_features ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

#IBVS with 3 features (By Deepak Raina)
#add_executable(ibvs_three_features src/ibvs_three_features.cpp include/my_funcs.h)
#target_link_libraries(ibvs_three_features ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

#executable for direct velocity test on robot
#add_executable(velocity_test_direct src/velocity_test_direct.cpp include/my_funcs.h)
#target_link_libraries(velocity_test_direct ${catkin_LIBRARIES} ${OpenCV_LIBS})

#executable for direct velocity test on gazebo-robot
add_executable(gazebo_velocity_test_direct src/gazebo_velocity_test_direct.cpp include/my_funcs.h)
target_link_libraries(gazebo_velocity_test_direct ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_executable(gazebo_reconfigure_robot src/gazebo_reconfigure_robot.cpp include/my_funcs.h)
target_link_libraries(gazebo_reconfigure_robot ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})

#add_executable(gazebo_ibvs_two_features src/gazebo_ibvs_two_features.cpp include/my_funcs.h)
#target_link_libraries(gazebo_ibvs_two_features ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})

#add_executable(gazebo_ibvs_three_features src/gazebo_ibvs_three_features.cpp include/my_funcs.h)
#target_link_libraries(gazebo_ibvs_three_features ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})

#add_executable(gazebo_ibvs_centroid_area src/gazebo_ibvs_centroid_area.cpp include/my_funcs.h)
#target_link_libraries(gazebo_ibvs_centroid_area ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})

#add_executable(gazebo_ibvs_area_intmatrix src/gazebo_ibvs_area_intmatrix.cpp include/my_funcs.h)
#target_link_libraries(gazebo_ibvs_area_intmatrix ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})

#executable for direct velocity test on gazebo-robot
#add_executable(velocity_controller_plugin src/velocity_controller.cpp)
#target_link_libraries(velocity_controller_plugin  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES}
#pthread joint_angle_msgs joint_velocity_msgs sim_variable_msgs image_data_msgs)

#executable for direct velocity test on gazebo-robot through ROS-transport mechanism
add_library(velocity_controller_ros SHARED src/velocity_controller_ros.cpp)
target_link_libraries(velocity_controller_ros ${catkin_LIBRARIES} ${GAZEBO_libraries} ${roscpp_LIBRARIES})


add_library(velocity_controller_ros_1 SHARED src/velocity_controller_ros_1.cpp)
target_link_libraries(velocity_controller_ros_1 ${catkin_LIBRARIES} ${GAZEBO_libraries} ${roscpp_LIBRARIES})

#executable for direct velocity test on gazebo-robot through ROS-transport mechanism
#add_library(velocity_controller_ros SHARED src/velocity_controller_ros(copy).cpp)
#target_link_libraries(velocity_controller_ros ${catkin_LIBRARIES} ${GAZEBO_libraries} ${roscpp_LIBRARIES})

#add_library(ball_animation_plugin SHARED src/ball_animation_plugin.cpp)
#target_link_libraries(ball_animation_plugin ${catkin_LIBRARIES} ${GAZEBO_libraries} ${roscpp_LIBRARIES})

##executable for direct velocity test on robot
#add_executable(tf_vs_numerical_transform src/tf_vs_numerical_transform.cpp)
#target_link_libraries(tf_vs_numerical_transform ${catkin_LIBRARIES} ${OpenCV_LIBS})

#executable for plotting features
#add_executable(plots src/plots.cpp src/gnuplot_ci.cpp include/gnuplot_ci.h)
#target_link_libraries(plots ${catkin_LIBRARIES})

#add_executable(traj_controller src/traj_controller.cpp src/compute_image_jacobian src/matrix_for_ee_to_camera.cpp include/my_funcs.h src/compute_new_geo_jacobian.cpp src/area_interaction_matrix.cpp src/set_feature_points.cpp)
#add_executable(traj_controller_area src/traj_controller_area.cpp src/new_area_L_matrix.cpp src/compute_image_jacobian src/matrix_for_ee_to_camera.cpp include/my_funcs.h src/compute_new_geo_jacobian.cpp src/area_interaction_matrix.cpp src/set_feature_points.cpp src/read_text_file.cpp)
#add_executable(traj_controller_area_inc src/traj_controller_area_inc.cpp src/new_area_L_matrix.cpp src/compute_image_jacobian src/matrix_for_ee_to_camera.cpp include/my_funcs.h src/compute_new_geo_jacobian.cpp src/area_interaction_matrix.cpp src/set_feature_points.cpp src/read_text_file.cpp)
#add_executable(traj_controller_area_sectask src/traj_controller_area_sectask.cpp src/new_area_L_matrix.cpp src/compute_image_jacobian src/matrix_for_ee_to_camera.cpp include/my_funcs.h src/compute_new_geo_jacobian.cpp src/area_interaction_matrix.cpp src/set_feature_points.cpp src/read_text_file.cpp)
#add_executable(traj_controller_sectask_self src/traj_controller_sectask_self.cpp src/new_area_L_matrix.cpp src/compute_image_jacobian src/matrix_for_ee_to_camera.cpp include/my_funcs.h src/compute_new_geo_jacobian.cpp src/area_interaction_matrix.cpp src/set_feature_points.cpp src/read_text_file.cpp)
#add_executable(joint_traj_controller_execute src/joint_traj_controller_execute.cpp src/compute_image_jacobian src/matrix_for_ee_to_camera.cpp include/my_funcs.h src/compute_new_geo_jacobian.cpp src/area_interaction_matrix.cpp src/set_feature_points.cpp src/read_text_file.cpp src/gnuplot_ci.cpp include/gnuplot_ci.h)


#add_executable(eyeinhand_6dof_area src/eyeinhand_6dof_area.cpp src/compute_image_jacobian src/new_area_L_matrix.cpp src/matrix_for_ee_to_camera.cpp include/my_funcs.h src/compute_new_geo_jacobian.cpp src/area_interaction_matrix.cpp)
#add_executable(eyeinhand_6dof_area_sectask src/eyeinhand_6dof_area_sectask.cpp src/compute_image_jacobian src/new_area_L_matrix.cpp src/matrix_for_ee_to_camera.cpp include/my_funcs.h src/compute_new_geo_jacobian.cpp src/area_interaction_matrix.cpp)

#add_executable(rrt_planning_543 src/rrt_planning_543.cpp src/compute_image_jacobian src/new_area_L_matrix.cpp src/matrix_for_ee_to_camera.cpp include/my_funcs.h src/compute_new_geo_jacobian.cpp src/area_interaction_matrix.cpp src/gen_new_node.cpp src/gen_new_node_543.cpp src/base_to_ee_transform.cpp src/ee_to_camera_transfrom.cpp src/Jacob_ee_to_camera.cpp src/plots.cpp src/read_text_file.cpp include/gnuplot_ci.h src/gnuplot_ci.cpp)

#add_executable(rrt_planning_area src/rrt_planning_area.cpp src/compute_image_jacobian src/new_area_L_matrix.cpp src/matrix_for_ee_to_camera.cpp include/my_funcs.h src/compute_new_geo_jacobian.cpp src/area_interaction_matrix.cpp src/gen_new_node.cpp src/gen_new_error.cpp src/gen_new_node_543.cpp src/base_to_ee_transform.cpp src/ee_to_camera_transfrom.cpp src/Jacob_ee_to_camera.cpp src/plots.cpp src/read_text_file.cpp include/gnuplot_ci.h src/gnuplot_ci.cpp)

#add_executable(rrt_planning_area_new src/rrt_planning_area_new.cpp src/compute_image_jacobian src/new_area_L_matrix.cpp src/matrix_for_ee_to_camera.cpp include/my_funcs.h src/compute_new_geo_jacobian.cpp src/area_interaction_matrix.cpp src/gen_new_node.cpp src/gen_new_error_new.cpp src/gen_new_error_DC.cpp src/gen_new_node_543.cpp src/base_to_ee_transform.cpp src/ee_to_camera_transfrom.cpp src/Jacob_ee_to_camera.cpp src/plots.cpp src/read_text_file.cpp include/gnuplot_ci.h src/gnuplot_ci.cpp)
#add_executable(rrt_planning_area_obstacle src/rrt_planning_area_obstacle.cpp src/compute_image_jacobian src/new_area_L_matrix.cpp src/matrix_for_ee_to_camera.cpp include/my_funcs.h src/compute_new_geo_jacobian.cpp src/area_interaction_matrix.cpp src/gen_new_node.cpp src/gen_new_error_new.cpp src/gen_new_node_543.cpp src/base_to_ee_transform.cpp src/ee_to_camera_transfrom.cpp src/Jacob_ee_to_camera.cpp src/plots.cpp src/read_text_file.cpp include/gnuplot_ci.h src/gnuplot_ci.cpp)
#add_executable(rrt_planning_area_sectask src/rrt_planning_area_sectask.cpp src/compute_image_jacobian src/new_area_L_matrix.cpp src/matrix_for_ee_to_camera.cpp include/my_funcs.h src/compute_new_geo_jacobian.cpp src/area_interaction_matrix.cpp src/gen_new_node.cpp src/gen_new_error_new.cpp src/gen_new_node_543.cpp src/base_to_ee_transform.cpp src/ee_to_camera_transfrom.cpp src/Jacob_ee_to_camera.cpp src/plots.cpp src/read_text_file.cpp include/gnuplot_ci.h src/gnuplot_ci.cpp)

#add_executable(rrt_planning src/rrt_planning.cpp src/compute_image_jacobian src/new_area_L_matrix.cpp src/matrix_for_ee_to_camera.cpp include/my_funcs.h src/compute_new_geo_jacobian.cpp src/area_interaction_matrix.cpp src/gen_new_node.cpp src/base_to_ee_transform.cpp src/ee_to_camera_transfrom.cpp src/Jacob_ee_to_camera.cpp src/plots.cpp src/read_text_file.cpp include/gnuplot_ci.h src/gnuplot_ci.cpp)
#add_executable(rrt_planning_centroid src/rrt_planning_centroid.cpp src/compute_image_jacobian src/new_area_L_matrix.cpp src/matrix_for_ee_to_camera.cpp include/my_funcs.h src/compute_new_geo_jacobian.cpp src/area_interaction_matrix.cpp src/gen_new_node_centroid.cpp src/base_to_ee_transform.cpp src/ee_to_camera_transfrom.cpp src/Jacob_ee_to_camera.cpp src/plots.cpp src/read_text_file.cpp include/gnuplot_ci.h src/gnuplot_ci.cpp)

##add_executable(eyeinhand_6dof_area_new src/eyeinhand_6dof_area_new.cpp src/compute_image_jacobian src/matrix_for_ee_to_camera.cpp include/my_funcs.h include/ur5_geo_jacobian.h src/compute_new_geo_jacobian.cpp src/area_interaction_matrix.cpp)
##add_executable(eyeinhand_planar_tracking src/eyeinhand_planar_tracking.cpp src/compute_image_jacobian src/matrix_for_ee_to_camera.cpp include/my_funcs.h include/ur5_geo_jacobian.h src/compute_new_geo_jacobian.cpp)


#add_executable(calibration_transform_setup src/calibration_transform_setup.cpp)

#add_executable(eyeinhand_plane_tracking src/eyeinhand_plane_tracking.cpp src/compute_image_jacobian src/matrix_for_ee_to_camera.cpp include/my_funcs.h src/compute_new_geo_jacobian.cpp)

#add_executable(plots_new src/plots_new.cpp src/gnuplot_ci.cpp include/gnuplot_ci.h)
#add_executable(plots_execution src/plots_execution.cpp src/gnuplot_ci.cpp include/gnuplot_ci.h)
#add_executable(plots_planning src/plots_planning.cpp src/gnuplot_ci.cpp include/gnuplot_ci.h)
#add_executable(plots_planning_exe src/plots_planning_execution.cpp src/gnuplot_ci.cpp include/gnuplot_ci.h)
#add_executable(plots_execution_tex src/plots_execution_tex.cpp src/gnuplot_ci.cpp include/gnuplot_ci.h)

add_definitions(-std=c++11)

# Linking GSL libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(gsl REQUIRED gsl)

# Linking GNUplot Library
#target_link_libraries(test_opencv ${OpenCV_LIBS})

#target_link_libraries(plots_new ${catkin_LIBRARIES})
#target_link_libraries(plots_execution ${catkin_LIBRARIES})
#target_link_libraries(plots_planning_exe ${catkin_LIBRARIES})
#target_link_libraries(plots_execution_tex ${catkin_LIBRARIES})



#target_link_libraries(eyeinhand_6dof_area
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#   )

#target_link_libraries(eyeinhand_6dof_area_sectask
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#   )

##target_link_libraries(eyeinhand_6dof_area_new
##    ${catkin_LIBRARIES}
##    ${OpenCV_LIBS}
##    ${PCL_LIBRARIES}
##   )

#target_link_libraries(eyeinhand_plane_tracking
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#   )

#target_link_libraries(traj_controller
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#   )

#target_link_libraries(rrt_planning
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#   )

#target_link_libraries(rrt_planning_543
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#   )

#target_link_libraries(rrt_planning_centroid
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#   )

#target_link_libraries(rrt_planning_area
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#   )

#target_link_libraries(rrt_planning_area_new
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#   )

#target_link_libraries(rrt_planning_area_obstacle
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#   )

#target_link_libraries(rrt_planning_area_sectask
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#   )

#target_link_libraries(traj_controller_area
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#   )

#target_link_libraries(traj_controller_area_inc
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#   )

#target_link_libraries(traj_controller_area_sectask
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#   )

#target_link_libraries(traj_controller_sectask_self
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#   )

#target_link_libraries(joint_traj_controller_execute
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#   )

#target_link_libraries(calibration_transform_setup
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#   )


#add_executable(eyeinhand_tracking_sift src/eyeinhand_tracking_sift.cpp src/compute_image_jacobian_sift src/matrix_for_ee_to_camera.cpp include/my_funcs.h include/ur5_geo_jacobian.h src/compute_new_geo_jacobian.cpp src/area_interaction_matrix.cpp)
#target_link_libraries(eyeinhand_tracking_sift
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#   )

#target_link_libraries(gp_test ${PROJECT_NAME})
#install(TARGETS ${PROJECT_NAME} DESTINATION ${PROJECT_SOURCE_DIR}/lib)
