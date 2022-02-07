cmake_minimum_required(VERSION 3.5)
project(rostui)

set(Project-Name_VERSION_MAJOR 0)
set(Project-Name_VERSION_MINOR 1)
set(Project-Name_VERSION_PATCH 0)
set(Project-Name_VERSION "${Project-Name_VERSION_MAJOR}.${Project-Name_VERSION_MINOR}.${Project-Name_VERSION_PATCH}")

set(PROJECT_NAME "rostui") # PROJECT_NAME is a predefined variable
set(PROJECT_DESCRIPTION "\"ROS terminal user interface\"")

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++19
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

set(NCPP_DIR "external/notcurses")
# Use Notcurses
add_definitions(-w)
add_subdirectory(${NCPP_DIR})
remove_definitions(-w)
# remove_definitions(-Werror)


# Set subdirectory variables
# option(USE_POC OFF)
option(USE_POC "Build small, uninstalled proof-of-concept binaries" OFF)

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Threads REQUIRED)
find_package(notcurses 3.0.0 CONFIG)

set_package_properties(notcurses PROPERTIES TYPE REQUIRED)
# Silence colcon from throwing a TON of warnings at us
set_target_properties(notcurses PROPERTIES COMPILE_FLAGS "-w")
set_target_properties(notcurses++ PROPERTIES COMPILE_FLAGS "-w")

set(LIBRARY_SOURCES
    src/node_monitor.cpp
    src/topic_monitor.cpp
    src/service_monitor.cpp
    src/topic_streamer.cpp
    include/rostui/stream_interface.hpp
)

add_library(${PROJECT_NAME}-lib ${LIBRARY_SOURCES} "${Notcurses_LIBRARY_DIRS}") # A library
set_target_properties(${PROJECT_NAME}-lib PROPERTIES PREFIX "") # Remove the lib prefix
target_include_directories(${PROJECT_NAME}-lib PUBLIC ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_BINARY_DIR}/include ${NCPP_DIR}/include)

 # ${rclcpp_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/main.cpp src/ui.cpp src/object_controller.cpp src/ros_interface_node.cpp) # The main executables
target_link_libraries(rostui rostui-lib
  Threads::Threads
  notcurses++
  ${rclcpp_LIBRARIES}
  ) # Link our sources to the executable

ament_target_dependencies(${PROJECT_NAME} rclcpp)
# ament_export_dependencies(rclcpp)
# ament_target_dependencies(rostui
# ament_target_dependencies(${PROJECT_NAME}
#    rclcpp)


# Install the built library and executable into the appropriate directory
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
install(TARGETS ${PROJECT_NAME}-lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
