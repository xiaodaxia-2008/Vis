file(GLOB_RECURSE srcs CONFIGURE_DEPENDS *.cpp *.h)
add_library(ZenVis ${srcs})
target_link_libraries(ZenVis PUBLIC ${OPENSCENEGRAPH_LIBRARIES} spdlog::spdlog)
target_include_directories(ZenVis PUBLIC ${OPENSCENEGRAPH_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
