cmake_minimum_required(VERSION 3.10)
project(rover_vda5050_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
    ament_cmake
    geometry_msgs
    nav_msgs
    pluginlib
    rclcpp
    rover_msgs
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    vda5050_connector
    vda5050_msgs)

foreach(PACKAGE IN ITEMS ${DEPENDENCIES})
    find_package(${PACKAGE} REQUIRED)
endforeach()

# Domain + application: plain C++, no ROS dependencies. Kept in its own target so the
# boundary is enforced by the linker as well as by scripts/check_domain_purity.sh.
add_library(${PROJECT_NAME}_core STATIC
    src/domain/route_tracker.cpp
    src/domain/rover_status.cpp
    src/application/navigation_use_case.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
set_property(TARGET ${PROJECT_NAME}_core PROPERTY POSITION_INDEPENDENT_CODE ON)

# Infrastructure + the handler plugins the connector's adapter node loads (plugins.xml).
add_library(${PROJECT_NAME}_plugins SHARED
    src/infrastructure/mission_manager_client.cpp
    src/infrastructure/rover_link.cpp
    src/infrastructure/vda_conversions.cpp
    src/plugins/mission_nav_through_nodes.cpp
    src/plugins/pause_actions.cpp
    src/plugins/rover_state_handler.cpp)
target_link_libraries(${PROJECT_NAME}_plugins
    ${PROJECT_NAME}_core
    vda5050_connector::adapter
    ${vda5050_connector_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    pluginlib::pluginlib
    rclcpp::rclcpp
    ${rover_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    tf2::tf2
    tf2_ros::tf2_ros
    ${vda5050_msgs_TARGETS})
pluginlib_export_plugin_description_file(vda5050_connector plugins.xml)

add_executable(adapter_node src/adapter_node.cpp)
target_link_libraries(adapter_node vda5050_connector::adapter rclcpp::rclcpp)

install(TARGETS ${PROJECT_NAME}_plugins
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(TARGETS adapter_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_route_tracker test/unit/test_route_tracker.cpp)
    target_link_libraries(test_route_tracker ${PROJECT_NAME}_core)

    ament_add_gtest(test_rover_status test/unit/test_rover_status.cpp)
    target_link_libraries(test_rover_status ${PROJECT_NAME}_core)

    ament_add_gtest(test_navigation_use_case test/unit/test_navigation_use_case.cpp)
    target_link_libraries(test_navigation_use_case ${PROJECT_NAME}_core)

    # Mechanically enforce that domain/ never grows a ROS dependency.
    add_test(
        NAME check_domain_purity
        COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_domain_purity.sh ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_export_include_directories(include)

ament_package()
