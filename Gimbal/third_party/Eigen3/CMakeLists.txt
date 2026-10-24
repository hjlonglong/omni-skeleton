get_hwc_lib_name("${CMAKE_CURRENT_SOURCE_DIR}" lib_name)
project(${lib_name} C CXX ASM)

get_hwc_lib_dir("${CMAKE_CURRENT_SOURCE_DIR}" lib_dir)
set(LIBRARY_OUTPUT_PATH ${lib_dir})

set_property(GLOBAL PROPERTY ${lib_name}_libs)
set_property(GLOBAL PROPERTY ${lib_name}_incs)
set_property(GLOBAL PROPERTY ${lib_name}_srcs)

set(${lib_name}_public_includes ${CMAKE_CURRENT_SOURCE_DIR})

set(${lib_name}_private_includes ${CMAKE_CURRENT_SOURCE_DIR}/PrivateInclude)

file(GLOB_RECURSE ${lib_name}_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.*)

set_property(GLOBAL APPEND PROPERTY ${lib_name}_incs
                                    ${${lib_name}_public_includes})
set_property(GLOBAL APPEND PROPERTY ${lib_name}_srcs ${${lib_name}_sources})
