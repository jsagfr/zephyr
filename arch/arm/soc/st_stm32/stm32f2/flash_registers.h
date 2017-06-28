/*
 * Copyright (c) 2016 RnDity Sp. z o.o.
 * Copyright (c) 2017 Jérôme Guéry
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _STM32F2X_FLASH_REGISTERS_H_
#define _STM32F2X_FLASH_REGISTERS_H_

#include <zephyr/types.h>

/**
 * @brief
 *
 * Based on reference manual:
 *   STM32F205xx, STM32F207xx, STM32F215xx and STM32F217xx
 *   advanced ARM ® -based 32-bits MCUs
 *
 * Chapter 2.3.3: Embedded Flash Memory
 */

enum {
	STM32_FLASH_LATENCY_0 = 0x0,
	STM32_FLASH_LATENCY_1 = 0x1,
	STM32_FLASH_LATENCY_2 = 0x2,
	STM32_FLASH_LATENCY_3 = 0x3,
	STM32_FLASH_LATENCY_4 = 0x4,
	STM32_FLASH_LATENCY_5 = 0x5,
	STM32_FLASH_LATENCY_6 = 0x6,
	STM32_FLASH_LATENCY_7 = 0x7,
};

/* 3.3.3 FLASH_ACR */
union ef_acr {
	u32_t val;
	struct {
		u32_t latency :3 __packed;
		u32_t rsvd__3_7 :5 __packed;
		u32_t prften :1 __packed;
		u32_t icen :1 __packed;
		u32_t dcen :1 __packed;
		u32_t icrst :1 __packed;
		u32_t dcrst :1 __packed;
		u32_t rsvd__13_31 :19 __packed;
	} bit;
};

/* 3.3.3 Embedded flash registers */
struct stm32_flash {
	union ef_acr acr;
};

/* list of device commands */
enum stm32_embedded_flash_cmd {
	STM32_FLASH_CMD_LATENCY_FOR_CLOCK_SET,
};

#endif /* _STM32F2X_FLASH_REGISTERS_H_ */
