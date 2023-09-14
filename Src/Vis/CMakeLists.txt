file(GLOB_RECURSE srcs CONFIGURE_DEPENDS *.cpp *.h)
add_library(Vis ${srcs})
target_link_libraries(Vis PUBLIC spdlog::spdlog libgizmo::libgizmo OpenGL::GL ${OSG_LIBS})
target_include_directories(Vis PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/Include>)
