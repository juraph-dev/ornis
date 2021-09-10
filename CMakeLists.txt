cmake_minimum_required(VERSION 3.12)

project(Rostui)

set(Project-Name_VERSION_MAJOR 0)
set(Project-Name_VERSION_MINOR 1)
set(Project-Name_VERSION_PATCH 0)
set(Project-Name_VERSION "${Project-Name_VERSION_MAJOR}.${Project-Name_VERSION_MINOR}.${Project-Name_VERSION_PATCH}")

set(PROJ_NAME "Rostui") # PROJECT_NAME is a predefined variable
set(PROJECT_DESCRIPTION "\"A cross-platform C++17 project template\"")

configure_file(include/Rostui/config.hpp.in
    ${CMAKE_BINARY_DIR}/include/Rostui/config.hpp
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_COMPILER_ID "GNU")
set(CMAKE_CXX_COMPILER_VERSION "10.1.0")

# Options
option(BUILD_TESTS "Build test executable" OFF)
option(GEN_DOCS "Generate documentation" ON)
option(ENABLE_COVERAGE "Enable code coverage" OFF)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as no build type was specified")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the build type (Debug/Release)" FORCE)
endif (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)

if (ENABLE_COVERAGE AND NOT CMAKE_CONFIGURATION_TYPES)
    if (NOT BUILD_TESTS)
        message(FATAL_ERROR "Tests must be enabled for code coverage!")
    endif (NOT BUILD_TESTS)

    include(CodeCoverage)

    append_coverage_compiler_flags()
    set(COVERAGE_EXCLUDES "/usr/include/\\*;${CMAKE_SOURCE_DIR}/src/main.cpp;${CMAKE_SOURCE_DIR}/test/*;${CMAKE_SOURCE_DIR}/third_party/doctest/*")

    setup_target_for_coverage(NAME coverage EXECUTABLE ctest DEPENDENCIES coverage)
else (ENABLE_COVERAGE AND NOT CMAKE_CONFIGURATION_TYPES)
    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra ")
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4 /WX")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
    endif(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
endif (ENABLE_COVERAGE AND NOT CMAKE_CONFIGURATION_TYPES)

set(LIBRARY_SOURCES
    src/factorial.cpp
    src/hello_world.cpp
    src/node_monitor.cpp
)

add_subdirectory(external/FTXUI)

add_library(Project-Name-lib ${LIBRARY_SOURCES}) # A library
set_target_properties(Project-Name-lib PROPERTIES PREFIX "") # Remove the lib prefix
target_include_directories(Project-Name-lib PUBLIC ${CMAKE_SOURCE_DIR}/include ${CMAKE_BINARY_DIR}/include)

add_executable(Rostui src/main.cpp src/ui.cpp src/object_controller.cpp) # The main executable
target_link_libraries(Rostui PUBLIC Project-Name-lib
  PRIVATE ftxui::screen
  PRIVATE ftxui::dom
  PRIVATE ftxui::component # Not needed for this example.
    ) # Link our sources to the executable

if (BUILD_TESTS)
    include(CTest)
    enable_testing()
    add_subdirectory(test)
endif (BUILD_TESTS)

# if (GEN_DOCS)
#     add_subdirectory(doc)
# endif (GEN_DOCS)

# Install the built library and executable into the appropriate directory
install(TARGETS Rostui DESTINATION bin)
install(TARGETS Project-Name-lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install the project header files into the appropriate directory
install(DIRECTORY include/ DESTINATION include PATTERN "config.hpp.in" EXCLUDE) # Install our main header files
install(FILES ${CMAKE_BINARY_DIR}/include/Rostui/config.hpp DESTINATION include/project-abbr) # Install our configuration file

# Install the project's extra files
install(FILES README.md LICENSE DESTINATION share/Rostui)
