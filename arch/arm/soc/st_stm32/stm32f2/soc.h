/*
 * Copyright (c) 2016 RnDity Sp. z o.o.
 * Copyright (c) 2017 Jérôme Guéry
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file SoC configuration macros for the STM32F2 family processors.
 *
 * Based on reference manual:
 *   STM32F205xx, STM32F207xx, STM32F215xx and STM32F217xx
 *   advanced ARM ® -based 32-bits MCUs
 *
 * Chapter 2.3: Memory organization
 */


#ifndef _STM32F2_SOC_H_
#define _STM32F2_SOC_H_

#define GPIO_REG_SIZE         0x400
/* base address for where GPIO registers start */
#define GPIO_PORTS_BASE       (GPIOA_BASE)

#ifndef _ASMLANGUAGE

#include <device.h>
#include <misc/util.h>
#include <drivers/rand32.h>

#include <stm32f2xx.h>

#include "soc_irq.h"

#ifdef CONFIG_CLOCK_CONTROL_STM32_CUBE
#include <stm32f2xx_ll_utils.h>
#include <stm32f2xx_ll_bus.h>
#include <stm32f2xx_ll_rcc.h>
#include <stm32f2xx_ll_system.h>
#endif /* CONFIG_CLOCK_CONTROL_STM32_CUBE */

#ifdef CONFIG_SERIAL_HAS_DRIVER
#include <stm32f2xx_ll_usart.h>
#endif

#ifdef CONFIG_I2C
#include <stm32f2xx_ll_i2c.h>
#endif

#ifdef CONFIG_RANDOM_STM32_RNG
#include <stm32f2xx_ll_rng.h>
#endif

#endif /* !_ASMLANGUAGE */

#endif /* _STM32F2_SOC_H_ */
