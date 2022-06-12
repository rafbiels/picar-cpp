cmake_minimum_required(VERSION 3.8)
project(picar-cpp-displays)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE CACHE BOOL
    "Create compile_commands.json" FORCE )
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
add_compile_options(-O0 -g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

link_directories(${CMAKE_CURRENT_BINARY_DIR}/../lcdgfx)

add_executable(Display_SH1106 src/Display_SH1106.cpp)
ament_target_dependencies(Display_SH1106 rclcpp std_msgs)
target_link_libraries(Display_SH1106 lcdgfx)
target_include_directories(Display_SH1106 PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>
                           ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/lcdgfx/src)
install(TARGETS Display_SH1106
        DESTINATION lib/${PROJECT_NAME})

ament_package()
