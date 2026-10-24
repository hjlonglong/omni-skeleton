/**
 *******************************************************************************
 * @file      : stm32_hal.hpp
 * @brief     : STM32 HAL 驱动头文件
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-07-11      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  必须与 CMakeLists.txt 模板搭配使用，因为该文件中包含了 STM32 HAL 库的具体名称
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_BSP_STM32_HAL_STM32_HAL_HPP_
#define HW_COMPONENTS_BSP_STM32_HAL_STM32_HAL_HPP_

/* Includes ------------------------------------------------------------------*/
#ifdef STM32_HAL_FILENAME
#include STM32_HAL_FILENAME
#else
#error "Please define STM32_HAL_FILENAME in CMakeLists.txt"
#endif /* STM32_HAL_FILENAME */
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif /* HW_COMPONENTS_BSP_STM32_HAL_STM32_HAL_HPP_ */
