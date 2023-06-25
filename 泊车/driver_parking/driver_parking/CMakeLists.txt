cmake_minimum_required(VERSION 3.20)
project(driver_parking)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(driver_parking
        class_Visualization.cpp
        class_Visualization.h
        Common_OpenCV.h
        driver_parking.cpp
        driver_parking.h
        osspec.h
        tgf.h)
