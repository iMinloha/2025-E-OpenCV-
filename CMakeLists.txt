cmake_minimum_required(VERSION 3.10)
project(EC_Gimbal)

set(CMAKE_CXX_STANDARD 17)

# 给出自己的OpenCV路径
set(OpenCV_DIR E:/opencv/build/install/x64/mingw/lib)

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

#find_library(WIRINGPI_LIB wiringPi)
#if (NOT WIRINGPI_LIB)
#    message(FATAL_ERROR "wiringPi library not found!")
#endif()
#
## 查找头文件路径
#find_path(WIRINGPI_INCLUDE_DIRS wiringPi.h)
#if (NOT WIRINGPI_INCLUDE_DIRS)
#    message(FATAL_ERROR "wiringPi headers not found!")
#endif()


# 寻找必要文件
find_package(OpenCV REQUIRED)
# 判断Opencv是否存在
if (OpenCV_FOUND)
    message(STATUS "OpenCV Found")
    message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
else ()
    message(FATAL_ERROR "OpenCV Not Found")
endif ()

SET(SOURCE_DIR Armor/*.* Driver/Camera/*.* Driver/UART/*.*
        Driver/UTD/*.* Driver/Motor/*.* TASK/*.*)

SET(INCLUDE_DIR Armor/ Driver/Camera/ Driver/UART/ Driver/UTD/ Driver/Motor/)

file(GLOB_RECURSE SOURCES ${SOURCE_DIR})

# 导入OpenCV的头
include_directories(${OpenCV_INCLUDE_DIRS} ${INCLUDE_DIR})

add_executable(${PROJECT_NAME} main.cpp ${SOURCES})

#target_include_directories(${PROJECT_NAME} PRIVATE ${WIRINGPI_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
#target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${WIRINGPI_LIB})
