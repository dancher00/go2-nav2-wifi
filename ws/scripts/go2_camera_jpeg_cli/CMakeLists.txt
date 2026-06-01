cmake_minimum_required(VERSION 3.10)
project(go2_camera_jpeg_cli CXX)

find_package(Threads REQUIRED)

set(UNITREE_SDK2 "$ENV{HOME}/unitree_sdk2" CACHE PATH "unitree_sdk2 root")
set(THIRD "${UNITREE_SDK2}/thirdparty")
set(ARCH "${CMAKE_SYSTEM_PROCESSOR}")

add_executable(go2_camera_jpeg_cli go2_camera_jpeg_cli.cpp)

target_include_directories(go2_camera_jpeg_cli PRIVATE
  "${UNITREE_SDK2}/include"
  "${THIRD}/include"
  "${THIRD}/include/ddscxx"
)

target_link_directories(go2_camera_jpeg_cli PRIVATE
  "${UNITREE_SDK2}/lib"
  "${UNITREE_SDK2}/build/lib"
  "${UNITREE_SDK2}/lib/${ARCH}"
  "${THIRD}/lib/${ARCH}"
)

target_link_libraries(go2_camera_jpeg_cli PRIVATE
  unitree_sdk2 ddsc ddscxx Threads::Threads
)

set_target_properties(go2_camera_jpeg_cli PROPERTIES CXX_STANDARD 14)

install(TARGETS go2_camera_jpeg_cli RUNTIME DESTINATION bin)
