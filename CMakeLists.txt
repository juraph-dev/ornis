cmake_minimum_required(VERSION 3.5)
project(rostui)

set(Project-Name_VERSION_MAJOR 0)
set(Project-Name_VERSION_MINOR 1)
set(Project-Name_VERSION_PATCH 0)
set(Project-Name_VERSION "${Project-Name_VERSION_MAJOR}.${Project-Name_VERSION_MINOR}.${Project-Name_VERSION_PATCH}")

set(PROJ_NAME "Rostui") # PROJECT_NAME is a predefined variable
set(PROJECT_DESCRIPTION "\"ROS terminal user interface\"")

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
# uncomment the following section in order to fill in
# further dependencies manually.
# Use threads
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

add_library(rostui-lib ${LIBRARY_SOURCES} "${Notcurses_LIBRARY_DIRS}") # A library
set_target_properties(rostui-lib PROPERTIES PREFIX "") # Remove the lib prefix
target_include_directories(rostui-lib PUBLIC ${CMAKE_SOURCE_DIR}/include ${CMAKE_BINARY_DIR}/include ${NCPP_DIR}/include)


add_executable(Rostui src/main.cpp src/ui.cpp src/object_controller.cpp) # The main executables
target_link_libraries(Rostui PUBLIC rostui-lib
    PRIVATE Threads::Threads
    PRIVATE notcurses++
    ) # Link our sources to the executable

# Install the built library and executable into the appropriate directory
install(TARGETS Rostui DESTINATION bin)
install(TARGETS rostui-lib
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
