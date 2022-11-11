cmake_minimum_required(VERSION 3.5)

project(DepthVista LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Widgets Concurrent OpenGL Core Gui REQUIRED)

find_package(PCL 1.8.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

INCLUDE_DIRECTORIES(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)
add_executable(DepthVista
  main.cpp
  mainwindow.cpp
  mainwindow.h
  mainwindow.ui
  cameraproperties.cpp
  cameraproperties.h
  cameraproperties.ui
  opengldisplay.cpp
  opengldisplay.h
  openglpcl.cpp
  openglpcl.h
  images.qrc
)


if(UNIX)
    target_link_libraries(DepthVista PRIVATE ${PCL_LIBRARIES} /usr/lib/libopencv_world.so.4.2.0 Qt5::Widgets Qt5::OpenGL Qt5::Gui Qt5::Core DepthVistaSDK)
    target_include_directories(DepthVista PRIVATE /usr/local/include/DepthVistaSDK ${Qt5_INCLUDE_DIRS})
else()
    target_include_directories(DepthVista PRIVATE ${Qt5_INCLUDE_DIRS})
    target_link_libraries(DepthVista PRIVATE ${PCL_LIBRARIES} Qt5::Widgets Qt5::OpenGL Qt5::Gui Qt5::Core)
endif()
