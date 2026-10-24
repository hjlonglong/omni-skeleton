# ##############################################################################
# #################       THIRD PARTY LIBRARIES          ######################
# #################    Copyright (c) 2024 Hello World    ######################
# ##############################################################################

# ---- CMSIS-DSP（ahrs / BMI088 依赖的数学库） -------------------------------
set(CMSIS_DSP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dsp/CMSIS-DSP")

file(GLOB_RECURSE cmsis_dsp_srcs "${CMSIS_DSP_DIR}/Source/*.c")

add_library(cmsis_dsp STATIC ${cmsis_dsp_srcs})
target_include_directories(
  cmsis_dsp
  PUBLIC ${CMSIS_DSP_DIR}/Include
  PRIVATE ${CMSIS_DSP_DIR}/PrivateInclude)
target_compile_definitions(cmsis_dsp
  PUBLIC ARM_MATH_MATRIX_CHECK ARM_MATH_ROUNDING)

# CMSIS-DSP 依赖 Cortex-M 内核头（cmsis_compiler.h / core_cm4.h），
# 它们随 Drivers 一起提供
target_link_libraries(cmsis_dsp PUBLIC drivers)

# ---- Eigen3（纯头文件库，仅供新人写算法时使用，不参与编译） ----------------
add_library(eigen3 INTERFACE)
target_include_directories(eigen3 INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/Eigen3")

# ---- SEGGER RTT（调试打印，新人可用于在线看变量） --------------------------
add_library(
  segger_rtt STATIC
  "${CMAKE_CURRENT_SOURCE_DIR}/SEGGER/RTT/SEGGER_RTT.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/SEGGER/RTT/SEGGER_RTT_printf.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.S")
target_include_directories(
  segger_rtt
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/SEGGER/RTT"
         "${CMAKE_CURRENT_SOURCE_DIR}/SEGGER/Config")
