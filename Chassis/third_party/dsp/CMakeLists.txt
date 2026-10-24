# 获取当前目录的库名
get_hwc_lib_name("${CMAKE_CURRENT_SOURCE_DIR}" lib_name)
project(${lib_name} C CXX ASM)

# 获取当前目录的库目录，设置库的输出路径
get_hwc_lib_dir("${CMAKE_CURRENT_SOURCE_DIR}" lib_dir)
set(LIBRARY_OUTPUT_PATH ${lib_dir})

# 设置C++标准为C++17，并要求必须支持
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置全局属性，用于传输文件信息
set_property(GLOBAL PROPERTY ${lib_name}_libs)
set_property(GLOBAL PROPERTY ${lib_name}_incs)

# 如果使用当前库，将通过 ${lib_name}_incs 传递 public 头文件，通过 ${lib_name}_libs 传递库名
if(use_${lib_name})
  # ~~~
  # DSP 库强制依赖 enable_hard_fp 选项，且必须为 ON enable_hard_fp 还有一些其他的编译选项，此部分在最外层的
  # CMakeLists.txt 中设置，例如： add_compile_options(-mfloat-abi=hard
  # -mfpu=fpv4-sp-d16) add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
  # ~~~
  # if(NOT enable_hard_fp)
  #   message(
  #     FATAL_ERROR "enable_hard_fp must be set to ON when using ${lib_name}")
  # endif()

  set(CMSISDSP "${CMAKE_CURRENT_SOURCE_DIR}/CMSIS-DSP")

  # 设置公共和私有的头文件目录，并移除没有头文件的目录
  set(${lib_name}_public_includes "${CMSISDSP}/Include")
  set(${lib_name}_private_includes "${CMSISDSP}/PrivateInclude")
  remove_redundant_include_directories(${lib_name}_public_includes)

  # TODO(ZhouShichan): 谨慎使用 *.* ，因为会包含一些不需要的文件 获取源文件
  file(GLOB_RECURSE ${lib_name}_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.c)

  # 局部变量，用于判断是否使用预编译库
  set(use_prebuilt OFF)
  set(${lib_name}_file)

  # 全局变量，用于判断是否使用预编译库
  if(use_prebuilt_${lib_name})
    # 如果使用预编译库，先判断预编译库是否存在，如果存在则使用，否则提示用户
    message(STATUS "use prebuilt lib: ${lib_name}")

    find_library(${lib_name}_file ${lib_name} PATHS ${LIBRARY_OUTPUT_PATH})

    if(${lib_name}_file)
      set(use_prebuilt ON)
      message(STATUS "found ${${lib_name}_file}")
    else()
      set(use_prebuilt OFF)
      message(STATUS "prebuilt lib not found, build from source soon")
    endif()
  endif()

  # ~~~
  # 裁剪的 DSP 库需要 STM32 工程的一些文件，在最外层的 CMakeLists 中将这些核心文件编译成了 divers 库
  # 设置依赖的库
  # ~~~
  set(depend_libs drivers)

  if(use_prebuilt)
    # ~~~
    # 添加库并设置属性(库文件所在位置以及暴露出去的接口头文件)
    # IMPORTED 表示这是一个预构建库
    # ! GLOBAL 表示该库可被全局使用，在其他目录使用时可正常获取到该库的属性
    # 接口头文件实际上和不使用预构建库时的公有(PUBLIC)头文件一致
    # 其他依赖当前库的库仍然可以使用 $<TARGET_PROPERTY:${depend_lib},PUBLIC_INCLUDE_DIRECTORIES> 获取头文件
    # 图方便可以直接使用 include_directories(${${lib_name}_public_includes}) 将头文件添加到全局
    # ~~~
    add_library(${lib_name} STATIC IMPORTED GLOBAL)
    set_target_properties(${lib_name} PROPERTIES IMPORTED_LOCATION
                                                 ${${lib_name}_file})
    target_include_directories(${lib_name}
                               INTERFACE ${${lib_name}_public_includes})
    foreach(depend_lib ${depend_libs})
      target_link_libraries(${lib_name} INTERFACE ${depend_lib})
    endforeach()
  else()
    # 不使用预编译库文件，编译源文件

    # DSP 库需要定义 ARM_MATH_MATRIX_CHECK 和 ARM_MATH_ROUNDING
    add_compile_definitions(ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)

    # ~~~
    # 编译源文件
    # TODO: 后续 install 需要考虑 PUBLIC_HEADER 属性
    # ~~~
    add_library(${lib_name} STATIC ${${lib_name}_sources})
    target_include_directories(
      ${lib_name}
      PUBLIC ${${lib_name}_public_includes}
      PRIVATE ${${lib_name}_private_includes})

    # 对每个依赖的库，设置头文件目录，添加依赖和链接库
    foreach(depend_lib ${depend_libs})
      # 似乎不需要 target_include_directories 也行
      target_include_directories(
        ${lib_name}
        PUBLIC $<TARGET_PROPERTY:${depend_lib},PUBLIC_INCLUDE_DIRECTORIES>)
      target_include_directories(
        ${lib_name}
        INTERFACE $<TARGET_PROPERTY:${depend_lib},INTERFACE_INCLUDE_DIRECTORIES>
      )
      add_dependencies(${lib_name} ${depend_lib})
      target_link_libraries(${lib_name} PUBLIC ${depend_lib})
    endforeach()
  endif()

  set_property(GLOBAL APPEND PROPERTY ${lib_name}_libs ${lib_name})
  set_property(GLOBAL APPEND PROPERTY ${lib_name}_incs
                                      ${${lib_name}_public_includes})
endif()
