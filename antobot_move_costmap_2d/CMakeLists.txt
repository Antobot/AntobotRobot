cmake_minimum_required(VERSION 3.0.2)
project(antobot_move_costmap_2d)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            dynamic_reconfigure
            geometry_msgs
            laser_geometry
            map_msgs
            message_filters
            message_generation
            nav_msgs
            pluginlib
            roscpp
            sensor_msgs
            std_msgs
            tf2
            tf2_geometry_msgs
            tf2_ros
            tf2_sensor_msgs
            visualization_msgs
            voxel_grid
            antobot_platform_msgs
            std_srvs
            antobot_devices_msgs
        )

find_package(OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

# messages
# add_message_files(
#     DIRECTORY msg
#     FILES
#     VoxelGrid.msg
# )

# generate_messages(
#     DEPENDENCIES
#         std_msgs
#         geometry_msgs
#         map_msgs
# )

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/Costmap2D.cfg
  cfg/ObstaclePlugin.cfg
  cfg/GenericPlugin.cfg
  cfg/InflationPlugin.cfg
  cfg/VoxelPlugin.cfg
)

catkin_package(
    INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
    LIBRARIES antobot_move_costmap_2d layers
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        laser_geometry
        map_msgs
        message_filters
        message_runtime
        nav_msgs
        pluginlib
        roscpp
        sensor_msgs
        std_msgs
        tf2_ros
        visualization_msgs
        voxel_grid
        antobot_platform_msgs
        std_srvs
        antobot_devices_msgs
    DEPENDS
        EIGEN3
        Boost
)

add_library(antobot_move_costmap_2d
  src/array_parser.cpp
  src/costmap_2d.cpp
  src/observation_buffer.cpp
  src/layer.cpp
  src/layered_costmap.cpp
  src/costmap_2d_ros.cpp
  src/costmap_2d_publisher.cpp
  src/costmap_math.cpp
  src/footprint.cpp
  src/costmap_layer.cpp
)
add_dependencies(antobot_move_costmap_2d ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(antobot_move_costmap_2d
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OPENGL_INCLUDE_DIRS}
)

add_library(am_layers
  plugins/inflation_layer.cpp
  plugins/obstacle_layer.cpp
  plugins/static_layer.cpp
  plugins/voxel_layer.cpp
  src/observation_buffer.cpp
)
add_dependencies(am_layers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(am_layers
antobot_move_costmap_2d
)

add_executable(am_costmap_2d_markers src/costmap_2d_markers.cpp)
add_dependencies(am_costmap_2d_markers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(am_costmap_2d_markers
    antobot_move_costmap_2d
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OPENGL_INCLUDE_DIRS}
)

add_executable(am_costmap_2d_cloud src/costmap_2d_cloud.cpp)
add_dependencies(am_costmap_2d_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(am_costmap_2d_cloud
    antobot_move_costmap_2d
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OPENGL_INCLUDE_DIRS}
)

add_executable(am_costmap_2d_node src/costmap_2d_node.cpp)
add_dependencies(am_costmap_2d_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(am_costmap_2d_node
    antobot_move_costmap_2d
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OPENGL_INCLUDE_DIRS}
)

# install( TARGETS
#     am_costmap_2d_markers
#     am_costmap_2d_cloud
#     am_costmap_2d_node
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS
# antobot_move_costmap_2d
#     am_layers
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(FILES am_costmap_plugins.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   PATTERN ".svn" EXCLUDE
# )
