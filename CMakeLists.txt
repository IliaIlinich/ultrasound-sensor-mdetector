cmake_minimum_required(VERSION 3.10)
project(ultrasound-sensor-mdetector CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

add_executable(mdetector mdetector.cpp)
target_include_directories(mdetector PRIVATE ${LIBMODBUS_INCLUDE_DIRS})
target_link_libraries(mdetector ${LIBMODBUS_LIBRARIES})
