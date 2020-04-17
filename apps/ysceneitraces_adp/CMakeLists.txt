add_executable(ysceneitraces_adp ysceneitraces_adp.cpp)

set_target_properties(ysceneitraces_adp  PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
target_include_directories(ysceneitraces_adp PUBLIC ${CMAKE_SOURCE_DIR}/libs)
target_link_libraries(ysceneitraces_adp yocto yocto_adp yocto_gui)
