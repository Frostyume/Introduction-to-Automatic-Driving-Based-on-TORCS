cmake_minimum_required(VERSION 3.20)
project(driver_cruise)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(driver_cruise
        class_Visualization.cpp
        class_Visualization.h
        Common_OpenCV.h
        driver_cruise.cpp
        driver_cruise.h
        osspec.h
        tgf.h)
