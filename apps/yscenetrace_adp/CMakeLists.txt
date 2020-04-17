add_executable(yscenetrace_adp yscenetrace_adp.cpp)

set_target_properties(yscenetrace_adp PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
target_include_directories(yscenetrace_adp PRIVATE ${CMAKE_SOURCE_DIR}/libs)
target_link_libraries(yscenetrace_adp yocto yocto_adp)
