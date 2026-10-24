# ##############################################################################
# #################        CMake Template (CUSTOM)       ######################
# #################    Copyright (c) 2023 Hello World    ######################
# ##############################################################################

# Set the system name and version
set(CMAKE_SYSTEM_VERSION 1)

# Specify the minimum required version of CMake
cmake_minimum_required(VERSION 3.22)

include("cmake/gcc-arm-none-eabi.cmake")

# Set the C++ and C standards
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)
get_filename_component(CMAKE_CURRENT_SOURCE_PARENT_DIR
                       ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)

# ########################## USER CONFIG SECTION ##############################
# Set the project name and the languages used
project(omni_gimbal C CXX ASM) # TODO: change project name here

# 启用 HAL 版 tick 实现：src/tick.cpp 的内容由 USE_HAL_TICK/USE_UNIX_TICK 包裹，
# 骨架未定义宏 → GetTickMs/GetTickUs/DelayUs 无实现（WP-02 修复，契约 §1.3）
add_compile_definitions(USE_HAL_TICK)

# Specify user folders TODO Add your own user folders here
# 新人的代码目录写在这里（需先手动创建对应文件夹），例如：
#   set(user_folders "Instance" "Task" "RobotModules")
set(user_folders "Instance" "RobotModules" "Task" "RobotComponents")
set(user_folders_in_parent_folder "RobotComponents")

# 收集 root_dir 及其所有子目录，作为头文件搜索路径
function(collect_includes root_dir res_list)
  set(dirs ${root_dir})
  file(GLOB_RECURSE children CONFIGURE_DEPENDS LIST_DIRECTORIES true RELATIVE
       ${root_dir}
       ${root_dir}/*)
  foreach(child ${children})
    if(IS_DIRECTORY ${root_dir}/${child})
      list(APPEND dirs ${root_dir}/${child})
    endif()
  endforeach()
  set(${res_list}
      ${dirs}
      PARENT_SCOPE)
endfunction()

# Enable preprocessing for assembler files
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

# Disable some warnings
set(COM_FLAGS
    "-Wno-unused-parameter -Wno-missing-field-initializers -Wno-pedantic -Wno-unknown-pragmas"
)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COM_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COM_FLAGS} -Wno-reorder")

add_subdirectory(cmake/stm32cubemx)
get_target_property(STM32_COMPILE_DEFINES stm32cubemx
                    INTERFACE_COMPILE_DEFINITIONS)
list(APPEND STM32_COMPILE_DEFINES DEBUG) # Add DEBUG definition
foreach(STM32_COMPILE_DEFINE ${STM32_COMPILE_DEFINES})
  # Exclude the element if it starts with "$"
  if(STM32_COMPILE_DEFINE MATCHES "^\\$")
    continue()
  endif()
  add_definitions(-D${STM32_COMPILE_DEFINE})

  # Get HAL filename
  if(STM32_COMPILE_DEFINE MATCHES "STM32[A-Z][0-9]")
    string(SUBSTRING ${STM32_COMPILE_DEFINE} 0 7 STM32_DEVICE)
    string(TOLOWER ${STM32_DEVICE} stm32_hal_filename)
    string(CONCAT stm32_hal_filename ${stm32_hal_filename} "xx_hal.h")
    message(STATUS "HAL file name: ${stm32_hal_filename}")
    # Set macro for the HAL filename
    add_definitions(-DSTM32_HAL_FILENAME="${stm32_hal_filename}")
  endif()
endforeach()

# #################### ADD LIBRARIES AND EXECUTABLE SECTION ####################
# Initialize source and include lists
set(project_srcs)
set(project_incs)

# Core 目录
collect_includes("${CMAKE_CURRENT_SOURCE_DIR}/Core" core_incs)
file(GLOB_RECURSE core_srcs CONFIGURE_DEPENDS "Core/*.c")
list(APPEND project_srcs ${core_srcs})
list(APPEND project_incs ${core_incs})

# Drivers 目录（STM32 HAL / CMSIS）
collect_includes("${CMAKE_CURRENT_SOURCE_DIR}/Drivers" drivers_incs)
file(GLOB_RECURSE drivers_srcs CONFIGURE_DEPENDS "Drivers/*.c")
list(APPEND project_incs ${drivers_incs})

# 组件目录：inc/ 放头文件，src/ 放源文件
collect_includes("${CMAKE_CURRENT_SOURCE_DIR}/inc" component_incs)
file(GLOB_RECURSE component_srcs CONFIGURE_DEPENDS "src/*.c*")
list(APPEND project_srcs ${component_srcs})
list(APPEND project_incs ${component_incs})

# 新人自定义目录
foreach(user_folder ${user_folders})
  list(FIND user_folders_in_parent_folder ${user_folder} INDEX)

  if(${INDEX} GREATER -1)
    message(STATUS "${user_folder} is in the parent folder list")
    set(search_path "${CMAKE_CURRENT_SOURCE_PARENT_DIR}/${user_folder}")
  else()
    set(search_path "${CMAKE_CURRENT_SOURCE_DIR}/${user_folder}")
  endif()

  if(NOT EXISTS ${search_path})
    message(FATAL_ERROR "${user_folder}(${search_path}) does not exist")
  endif()

  collect_includes("${search_path}" ${user_folder}_incs)
  file(GLOB_RECURSE ${user_folder}_srcs CONFIGURE_DEPENDS "${search_path}/*.c*")
  list(APPEND project_incs ${${user_folder}_incs})
  list(APPEND project_srcs ${${user_folder}_srcs})
endforeach()

# Add a static library for the drivers
add_library(drivers STATIC ${drivers_srcs})
target_include_directories(drivers PUBLIC ${core_incs} ${drivers_incs})

# Add the third party libraries (CMSIS-DSP / Eigen3 / SEGGER RTT)
add_subdirectory(third_party)

# Add an executable for the project
file(GLOB startup_file CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.s")
add_executable(${PROJECT_NAME} ${project_srcs} ${startup_file})
message(STATUS "Project name: ${PROJECT_NAME}")

# Add the project includes to the executable
target_include_directories(${PROJECT_NAME} PUBLIC ${project_incs})

# Link the drivers library and the third party libraries to the executable
target_link_libraries(${PROJECT_NAME} PUBLIC drivers)
target_link_libraries(${PROJECT_NAME} PUBLIC cmsis_dsp)
# WP-02：链接 SEGGER RTT（third_party 已编译，骨架漏链接）；WP-06 起用于调试输出
target_link_libraries(${PROJECT_NAME} PUBLIC segger_rtt)

# Define the output hex and bin files
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

# Add a post-build command to generate the hex and bin files
add_custom_command(
  TARGET ${PROJECT_NAME}
  POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}> ${HEX_FILE}
  COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}> ${BIN_FILE}
  COMMENT "Building ${HEX_FILE} Building ${BIN_FILE}")
