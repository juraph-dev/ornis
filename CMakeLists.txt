cmake_minimum_required(VERSION 3.5)
project(ornis)

set(Project-Name_VERSION_MAJOR 0)
set(Project-Name_VERSION_MINOR 1)
set(Project-Name_VERSION_PATCH 0)
set(Project-Name_VERSION "${Project-Name_VERSION_MAJOR}.${Project-Name_VERSION_MINOR}.${Project-Name_VERSION_PATCH}")

set(PROJECT_NAME "ornis")
set(PROJECT_DESCRIPTION "\"Open Ros2 Notcurses Interface System\"")

# Options
# option(BUILD_TESTS "Build test executable" OFF)
# option(GEN_DOCS "Generate documentation" OFF)
# option(ENABLE_COVERAGE "Enable code coverage" OFF)

# if (CHECK_INCLUDES)
#   find_program(iwyu_path NAMES include-what-you-use iwyu REQUIRED)
# endif(CHECK_INCLUDES)
option(CHECK_INCLUDES "use iwyu to check for bad includes" OFF)
option(FIX_INCLUDES "use iwyu to fix bad includes" OFF)


# if (ENABLE_COVERAGE)
#     include(CodeCoverage)
#     append_coverage_compiler_flags()
#     set(COVERAGE_EXCLUDES "/usr/include/\\*;${CMAKE_SOURCE_DIR}/src/main.cpp;${CMAKE_SOURCE_DIR}/test/*;${CMAKE_SOURCE_DIR}/third_party/doctest/*")

#     setup_target_for_coverage(NAME coverage EXECUTABLE ctest DEPENDENCIES coverage)
# endif(ENABLE_COVERAGE)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra )
endif()

# Set subdirectory variables
# option(USE_POC "Build small, uninstalled proof-of-concept binaries" OFF) # Doesn't work
set(NCPP_DIR "external/notcurses")
# Use Notcurses
add_subdirectory(${NCPP_DIR})

# find dependencies
find_package(ament_index_cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(notcurses 3.0.0 CONFIG)

# Perform this after find_package to prevent iwyu from running on our external dir
IF (FIX_INCLUDES)
  find_program(IWYU_PATH NAMES include-what-you-use iwyu)
  IF(NOT IWYU_PATH)
    message(FATAL_ERROR "Could not find the program include-what-you-use")
  ENDIF()
  set(IWYU_PATH_OPTS
    ${IWYU_PATH})
  set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH_OPTS})
  set(CMAKE_C_INCLUDE_WHAT_YOU_USE ${IWYU_PATH_OPTS})
ELSEIF (CHECK_INCLUDES)
  find_program(IWYU_PATH NAMES include-what-you-use iwyu)
  IF(NOT IWYU_PATH)
    message(FATAL_ERROR "Could not find the program include-what-you-use")
  ENDIF()
  set(IWYU_PATH_OPTS
    ${IWYU_PATH}
    -Xiwyu)
  set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH_OPTS})
  set(CMAKE_C_INCLUDE_WHAT_YOU_USE ${IWYU_PATH_OPTS})
ENDIF(FIX_INCLUDES)

file(GLOB LIBRARY_SOURCES
  src/*.cpp
)

list(REMOVE_ITEM LIBRARY_SOURCES "main.cpp")

add_library(${PROJECT_NAME}-lib ${LIBRARY_SOURCES})
target_include_directories(${PROJECT_NAME}-lib PUBLIC ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_BINARY_DIR}/include ${NCPP_DIR}/include
  ${rclcpp_INLUDE_DIR} rosidl_generator_cpp
  rosidl_typesupport_introspection_cpp
  rosidl_typesupport_cpp)

add_executable(${PROJECT_NAME} src/main.cpp) # The main executables

target_link_libraries(ornis-lib PUBLIC
  notcurses++
  Threads::Threads
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rcl::rcl
  rcl_action::rcl_action
  ) # Link our sources to the executable

target_link_libraries(ornis PUBLIC
  ornis-lib
  notcurses++
  Threads::Threads
  rclcpp::rclcpp
  rcl::rcl
  rcl_action::rcl_action
  ament_index_cpp::ament_index_cpp
  ) # Link our sources to the executable

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
