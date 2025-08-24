cmake_minimum_required(VERSION 3.12)
project(MyEngine LANGUAGES CXX)

# Use Clang (if available)
set(CMAKE_CXX_COMPILER "g++")

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build configurations
set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type (Debug/Release)")
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")

# Compiler flags
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
    set(CMAKE_CXX_FLAGS_RELEASE "-O2")
endif()

# SDL3 Options
option(USE_VENDORED_SDL "Use vendored SDL instead of system SDL" OFF)

if(USE_VENDORED_SDL)
    # Using vendored SDL from your local path
    message(STATUS "Using vendored SDL3")
    add_subdirectory(${CMAKE_HOME_DIRECTORY}/Installer/vendored/SDL EXCLUDE_FROM_ALL)
else()
    # Using system-installed SDL3
    message(STATUS "Using system SDL3")
    
    # Help CMake find SDL3 in /usr/local if not in default paths
    list(APPEND CMAKE_PREFIX_PATH "/usr/local")
    
    find_package(SDL3 REQUIRED CONFIG)
    if(NOT SDL3_FOUND)
        message(FATAL_ERROR "SDL3 not found! Install it or set USE_VENDORED_SDL=ON")
    endif()
endif()

# Add sources
file(GLOB SOURCES "src/*.cc")

# Create executable
add_executable(MyEngine ${SOURCES})

# Include directories
target_include_directories(MyEngine PRIVATE "src")

# Link SDL3
target_link_libraries(MyEngine PRIVATE SDL3::SDL3)

# Print SDL3 location for verification
get_target_property(SDL3_LIB SDL3::SDL3 LOCATION)
message(STATUS "Linking with SDL3 library: ${SDL3_LIB}")
