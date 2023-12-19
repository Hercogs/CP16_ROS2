cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tortoisebot_waypoints_interfaces REQUIRED)

# Custom action msg
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
	"action/WaypointAction.action"

	DEPENDENCIES geometry_msgs
)


set (dependencies rclcpp
				  rclcpp_action
				  std_msgs
				  nav_msgs
				  geometry_msgs
				  tf2
				  std_srvs
				  tortoisebot_waypoints_interfaces
)

# ADD LIBRARY
include_directories(include)
add_library(action_server_lib src/action_server_component.cpp include/tortoisebot_waypoints/action_server_component.hpp)
ament_target_dependencies(action_server_lib ${dependencies})

ament_export_targets(action_server_lib HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/tortoisebot_waypoints
  DESTINATION include
)

install(TARGETS action_server_lib
  DESTINATION lib/action_server_lib)

install(
  TARGETS action_server_lib
  EXPORT action_server_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# ADD EXECUTABLE
add_executable(main_node src/main_node.cpp)
target_link_libraries(main_node PUBLIC action_server_lib)

add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client ${dependencies})

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server ${dependencies})

install(TARGETS
	action_server
	action_client
	main_node
	DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
      test/main.cpp
      test/action_server_test.cpp
      )
  # Add a gtest executables
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  ament_target_dependencies(${PROJECT_NAME}_test ${dependencies})
  
  target_link_libraries(${PROJECT_NAME}_test action_server_lib)

  install(TARGETS
        ${PROJECT_NAME}_test
        # Install the generated test executable file to the path after DESTINATION
        DESTINATION lib/${PROJECT_NAME})
  # uncomment to enforce lintners and static code analyzers defined in ament_lint_common package        
  # ament_lint_auto_find_test_dependencies()
endif()

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(rclcpp REQUIRED)
#   find_package(rclcpp_action REQUIRED)
#   find_package(std_msgs REQUIRED)
#   find_package(nav_msgs REQUIRED)
#   find_package(geometry_msgs REQUIRED)
#   find_package(tf2 REQUIRED)
#   find_package(std_srvs REQUIRED)
#   find_package(tortoisebot_waypoints_interfaces REQUIRED)

#   set(TESTFILES
#   	test/main.cpp
# 	test/action_server_test.cpp
#   )

#     # Add a gtest executables
#   ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
#   ament_target_dependencies(${PROJECT_NAME}_test
#   							rclcpp 
# 							std_msgs 
# 							nav_msgs 
# 							geometry_msgs 
# 							rclcpp_action 
# 							tortoisebot_waypoints_interfaces
# 							tf2
# 							std_srvs
# 	)
# 	# target_link_libraries(${PROJECT_NAME}_test action_server_lib)

#   install(TARGETS
#         ${PROJECT_NAME}_test
#         # Install the generated test executable file to the path after DESTINATION
#         DESTINATION lib/${PROJECT_NAME})
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
