cmake_minimum_required(VERSION 3.14)
project(hk-camera-compressed)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Ament dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)

find_package(CUDAToolkit REQUIRED)
cmake_policy(SET CMP0146 OLD)
find_package(OpenCV REQUIRED)

add_subdirectory(GPUJPEG)

add_library(hikrobot INTERFACE)
target_include_directories(hikrobot
  INTERFACE
    ${OpenCV_INCLUDE_DIRS}
    /opt/MVS/include/
)
target_link_directories(hikrobot
  INTERFACE
    /opt/MVS/lib/64/
)
target_link_libraries(hikrobot
  INTERFACE
    ${OpenCV_LIBRARIES}
    MvCameraControl
    MVGigEVisionSDK
    MVRender
    MvUsb3vTL
    MediaProcess
)

add_executable(hk-camera src/main.cpp src/hk-utils.cpp src/CameraMaster.cpp)
target_include_directories(hk-camera PUBLIC include/hk-camera-compressed GPUJPEG)
target_link_libraries(hk-camera hikrobot CUDA::cudart gpujpeg)
ament_target_dependencies(hk-camera rclcpp std_msgs image_transport)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(hk-camera PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(FetchContent)
FetchContent_Declare(
    nanobench
    GIT_REPOSITORY https://github.com/martinus/nanobench.git
    GIT_TAG v4.3.11
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(nanobench)

add_executable(bench src/bench.cpp src/nanobench.cpp)
target_include_directories(bench PUBLIC include/hk-camera-compressed GPUJPEG)
target_link_libraries(bench CUDA::cudart nanobench gpujpeg hikrobot)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(bench PRIVATE -Wall -Wextra -Wpedantic)
endif()


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

install(TARGETS
  hk-camera
  DESTINATION lib/${PROJECT_NAME})

ament_package()
