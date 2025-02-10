cmake_minimum_required(VERSION 3.8)
project(rtc_streamer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(LibDataChannel SHARED IMPORTED)
set_target_properties(LibDataChannel PROPERTIES
    IMPORTED_LOCATION /home/damon/robotics/CameraSystem/deps/libdatachannel/build/libdatachannel.so
    INTERFACE_INCLUDE_DIRECTORIES /home/damon/robotics/CameraSystem/deps/libdatachannel/include)
# add_library(nlohmann_json SHARED IMPORTED)
# set_target_properties(nlohmann_json PROPERTIES
#     IMPORTED_LOCATION /home/damon/robotics/CameraSystem/deps/libdatachannel/deps/json/single_include/nlohmann/json.hpp)


add_executable(rtc_streamer src/main.cpp)

target_include_directories(rtc_streamer PRIVATE /home/damon/robotics/CameraSystem/deps/libdatachannel/deps/json/single_include)

set_target_properties(rtc_streamer PROPERTIES
    CXX_STANDARD 17
    OUTPUT_NAME rtc_streamer)

find_package(Threads REQUIRED)
target_link_libraries(rtc_streamer LibDataChannel Threads::Threads nlohmann_json)