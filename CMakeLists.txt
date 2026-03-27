cmake_minimum_required(VERSION 3.0.2)
project(rbkairos_edge_vision)

## C++14 standard
add_compile_options(-std=c++14)

## ROS komponente (Obavezno za oba uređaja)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  vision_msgs
  rbkairos_etf_services
)

## OpenCV je uvek potreban
find_package(OpenCV REQUIRED)

## CUDA tražimo "tiho" (QUIET) - bez REQUIRED!
find_package(CUDA QUIET)

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs vision_msgs rbkairos_etf_services
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Uslovno kompajliranje: Samo ako postoji CUDA (Jetson Nano scenario)
if(CUDA_FOUND)
    message(STATUS "CUDA pronađena - kompajliram yolo_node za Jetson Nano")
    
    include_directories(
        ${CUDA_INCLUDE_DIRS} 
        /usr/local/cuda/include
        /usr/include/aarch64-linux-gnu
    )
    
    link_directories(
        /usr/local/cuda/lib64
        ${catkin_LIBRARY_DIRS}
    )

    add_executable(yolo_node src/yolo_node.cpp)
    target_link_libraries(yolo_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${CUDA_LIBRARIES}
        nvinfer
        cudart
    )
else()
    message(WARNING "CUDA nije pronađena - preskačem C++ yolo_node (Ovo je normalno na laptopu)")
endif()
