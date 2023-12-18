cmake_minimum_required(VERSION 3.5)

project(DepthVista LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Widgets Concurrent OpenGL Core Gui REQUIRED)

find_package(OpenCV REQUIRED)

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

target_include_directories(DepthVista PRIVATE ${TOF_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Qt5_INCLUDE_DIRS})
if(UNIX)
    target_link_libraries(DepthVista PRIVATE ${OpenCV_LIBRARIES} Qt5::Widgets Qt5::OpenGL Qt5::Gui Qt5::Core DepthVistaSDK)
else()
    target_link_libraries(DepthVista PRIVATE ${OpenCV_LIBRARIES} Qt5::Widgets Qt5::OpenGL Qt5::Gui Qt5::Core)
endif()
