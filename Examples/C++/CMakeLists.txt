add_executable(Basics Basics.cpp)
target_link_libraries(Basics PRIVATE Vis)

find_package(Eigen3 CONFIG REQUIRED)
add_executable(OsgPointWithNormals OsgPointWithNormals.cpp)
target_link_libraries(OsgPointWithNormals PRIVATE ${OPENSCENEGRAPH_LIBRARIES} Eigen3::Eigen spdlog::spdlog)
target_include_directories(OsgPointWithNormals PRIVATE ${OPENSCENEGRAPH_INCLUDE_DIRS})