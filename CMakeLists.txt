cmake_minimum_required(VERSION 2.4.6)
project(GLRenderer)

set(CMAKE_CXX_FLAGS "-O3 -g -std=c++11")

find_package(catkin REQUIRED COMPONENTS 
  cmake_modules 
)

find_package(OpenCV 3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS eigen opencv glut
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${OpenCV_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

add_library(${PROJECT_NAME} src/cvCamera.cpp src/glInfo.cpp src/glm.cpp src/glRenderer.cpp src/marker.cpp src/markerDetector.cpp src/timer.cpp src/renderLib.cpp) 

add_executable(render src/renderTest.cpp)
target_link_libraries(render ${PROJECT_NAME} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY}  ${OpenCV_LIBS})

