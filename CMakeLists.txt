cmake_minimum_required(VERSION 3.10)
project(Vis VERSION 1.1.1)

# Make the lib all -fPIC
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(WIN32)
  find_package(OpenSceneGraph REQUIRED COMPONENTS osgViewer osgGA osgText osgDB osgUtil osgFX)
  set(OSG_INCLUDE_DIR ${OPENSCENEGRAPH_INCLUDE_DIRS})
  set(OSG_LIBS ${OPENSCENEGRAPH_LIBRARIES})
else()
  # install OpenSceneGraph with sudo apt install libopenscenegraph-3.4-dev
  find_package(osg REQUIRED COMPONENTS osgViewer osgGA osgText osgDB osgUtil osgFX)
  set(OSG_LIBS osg osgViewer osgGA osgText osgDB osgUtil osgFX)
endif()

message(STATUS OPENSCENEGRAPH_INCLUDE_DIRS:${OPENSCENEGRAPH_INCLUDE_DIRS})
message(STATUS OPENSCENEGRAPH_LIBRARIES:${OPENSCENEGRAPH_LIBRARIES})

include(GNUInstallDirs)

find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Boost REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

# add_subdirectory(Externals)
# add_subdirectory(Src)
add_subdirectory(ZenVis)
add_subdirectory(Examples)

include(CPack)