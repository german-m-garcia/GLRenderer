cmake_minimum_required(VERSION 2.4.6)
project(GLRenderer)

set(CMAKE_CXX_FLAGS "-O3 -g -std=c++11")

find_package(OpenCV 3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)



include_directories(
  include  
)


include_directories(${OpenCV_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

add_library(GLRender src/cvCamera.cpp src/glInfo.cpp src/glm.cpp src/glRenderer.cpp src/marker.cpp src/markerDetector.cpp src/timer.cpp) 

add_executable(render src/renderTest.cpp)
target_link_libraries(render GLRender ${OPENGL_LIBRARIES} ${GLUT_LIBRARY}  ${OpenCV_LIBS})

