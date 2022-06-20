cmake_minimum_required(VERSION 3.5)
project(ros_qwiic_tof)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(MSVC)
  add_compile_definitions(
    _USE_MATH_DEFINES
    _WIN32_WINNT=0x0601
  )
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Arduino code can have unused parameters
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

add_executable(${PROJECT_NAME} 
  src/main.cpp
  src/wiring_for_ros/SPI.cpp
  src/wiring_for_ros/Wire.cpp
  src/wiring_for_ros/Arduino.cpp
  src/SparkFun_VL53L5CX_Arduino_Library/src/SparkFun_VL53L5CX_IO.cpp
  src/SparkFun_VL53L5CX_Arduino_Library/src/SparkFun_VL53L5CX_Library.cpp
  src/SparkFun_VL53L5CX_Arduino_Library/src/vl53l5cx_api.cpp
  src/SparkFun_VL53L5CX_Arduino_Library/src/platform.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  MAP_IMPORTED_CONFIG_MINSIZEREL Release
  MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release
  )


find_program(MAKE_EXECUTABLE
    NAMES gmake mingw32-make make
    NAMES_PER_DIR
    DOC "GNU Make")
if(NOT MSVC)
include(ExternalProject)
ExternalProject_Add(libi2c
    GIT_REPOSITORY https://github.com/amaork/libi2c.git
    UPDATE_DISCONNECTED true
    BUILD_IN_SOURCE true
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ${MAKE_EXECUTABLE}
    INSTALL_COMMAND ""
)
ExternalProject_Get_property(libi2c SOURCE_DIR)

message (STATUS "*** libi2c installed to ${SOURCE_DIR}")
target_link_libraries(${PROJECT_NAME} ${SOURCE_DIR}/libi2c.so)

else()
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED system)

include_directories(
  include/${PROJECT_NAME}
  ${SOURCE_DIR}/include
  src/wiring_for_ros/include
  src/SparkFun_VL53L5CX_Arduino_Library/src
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  Boost)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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