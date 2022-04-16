cmake_minimum_required(VERSION 3.8)
project(picar-cpp-sensors)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE CACHE BOOL
    "Create compile_commands.json" FORCE )
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_compile_options(-Wall -Wextra -Wpedantic)
add_compile_options(-O0 -g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(Distance_SF_SR02 src/Distance_SF_SR02.cpp)
ament_target_dependencies(Distance_SF_SR02 rclcpp std_msgs)
target_link_libraries(Distance_SF_SR02 gpiodcxx)
target_include_directories(Distance_SF_SR02 PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>)
install(TARGETS Distance_SF_SR02
        DESTINATION lib/${PROJECT_NAME})

ament_package()
