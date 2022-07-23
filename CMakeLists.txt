cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 11)

project(
    realsense-network-hardware-video-encoder
)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# Make project require C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

# Simple non robust way to find the librealsense library
if(WIN32)
    set(VCPKG_INSTALLED_DIR "D:\\Tools\\vcpkg\\installed")
    if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8")
        set(LIBRARY_DIR "${VCPKG_INSTALLED_DIR}\\x64-windows\\lib")          # TODO: Update this variable to correct path - folder where realsense2.lib is found
        set(DLL_DIR "${VCPKG_INSTALLED_DIR}\\x64-windows\\bin")              # TODO: Update this variable to correct path - folder where realsense2.dll is found
    else()
        set(LIBRARY_DIR "${VCPKG_INSTALLED_DIR}\\x86-windows\\lib")          # TODO: Update this variable to correct path - folder where realsense2.lib is found
        set(DLL_DIR "${VCPKG_INSTALLED_DIR}\\x86-windows\\lib")              # TODO: Update this variable to correct path - folder where realsense2.dll is found
    endif()
    set(PROJECT_BINARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/\$\(Configuration\)")                # TODO: Update this variable to correct path - folder to which your project will compile
    set(ADDITIONAL_INCLUDE_DIRS "${VCPKG_INSTALLED_DIR}\\x64-windows\\include")   # TODO: Update this variable to correct path - folder where librealsense2 folder is found
endif()


find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()

# build the libraries tree
add_subdirectory(network-hardware-video-encoder)

# those are our main targets
add_executable(realsense-nhve-h264 rnhve_h264.cpp)
target_include_directories(realsense-nhve-h264 PRIVATE network-hardware-video-encoder)
target_link_libraries(realsense-nhve-h264 nhve ${REALSENSE2_FOUND})

add_executable(realsense-nhve-hevc rnhve_hevc.cpp)
target_include_directories(realsense-nhve-hevc PRIVATE network-hardware-video-encoder)
target_link_libraries(realsense-nhve-hevc nhve ${REALSENSE2_FOUND})

add_executable(realsense-nhve-depth-ir rnhve_depth_ir.cpp)
target_include_directories(realsense-nhve-depth-ir PRIVATE network-hardware-video-encoder)
target_link_libraries(realsense-nhve-depth-ir nhve ${REALSENSE2_FOUND})

add_executable(realsense-nhve-depth-color rnhve_depth_color.cpp)
target_include_directories(realsense-nhve-depth-color PRIVATE network-hardware-video-encoder)
target_link_libraries(realsense-nhve-depth-color nhve ${REALSENSE2_FOUND})

add_executable(realsense-nhve-depth-color-audio rnhve_depth_color_audio.cpp audio_winmm.cpp)
target_include_directories(realsense-nhve-depth-color-audio PRIVATE network-hardware-video-encoder)
target_link_libraries(realsense-nhve-depth-color-audio nhve ${REALSENSE2_FOUND})

