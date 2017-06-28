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

/* IO pin functions */
enum stm32f2x_pin_config_mode {
	STM32F2X_PIN_CONFIG_BIAS_HIGH_IMPEDANCE = 0,
	STM32F2X_PIN_CONFIG_BIAS_PULL_UP,
	STM32F2X_PIN_CONFIG_BIAS_PULL_DOWN,
	STM32F2X_PIN_CONFIG_ANALOG,
	STM32F2X_PIN_CONFIG_DRIVE_OPEN_DRAIN,
	STM32F2X_PIN_CONFIG_DRIVE_PUSH_PULL,
	STM32F2X_PIN_CONFIG_DRIVE_OPEN_DRAIN_PU,
	STM32F2X_PIN_CONFIG_DRIVE_OPEN_DRAIN_PD,
	STM32F2X_PIN_CONFIG_DRIVE_PUSH_PULL_PU,
	STM32F2X_PIN_CONFIG_DRIVE_PUSH_PULL_PD,
	STM32F2X_PIN_CONFIG_AF,
};

#include "soc_irq.h"

#ifdef CONFIG_SERIAL_HAS_DRIVER
#include <stm32f2xx_ll_usart.h>
#endif

#ifdef CONFIG_CLOCK_CONTROL_STM32_CUBE
#include <stm32f2xx_ll_utils.h>
#include <stm32f2xx_ll_bus.h>
#include <stm32f2xx_ll_rcc.h>
#include <stm32f2xx_ll_system.h>
#endif /* CONFIG_CLOCK_CONTROL_STM32_CUBE */

#endif /* !_ASMLANGUAGE */

#endif /* _STM32F2_SOC_H_ */
