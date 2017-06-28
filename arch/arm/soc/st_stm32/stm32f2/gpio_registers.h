/*
 * Copyright (c) 2016 RnDity Sp. z o.o.
 * Copyright (c) 2017 Jérôme Guéry
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _STM32F2X_GPIO_REGISTERS_H_
#define _STM32F2X_GPIO_REGISTERS_H_

/**
 * @brief
 *
 * Based on reference manual:
 *   STM32F205xx, STM32F207xx, STM32F215xx and STM32F217xx
 *   advanced ARM ® -based 32-bits MCUs
 *
 * Chapter 6: General-purpose I/Os (GPIO)
 */

struct stm32f2x_gpio {
	u32_t moder;
	u32_t otyper;
	u32_t ospeedr;
	u32_t pupdr;
	u32_t idr;
	u32_t odr;
	u32_t bsrr;
	u32_t lckr;
	u32_t afrl;
	u32_t afrh;
};

union syscfg__memrmp {
	u32_t val;
	struct {
		u32_t mem_mode :2 __packed;
		u32_t rsvd__2_31 :30 __packed;
	} bit;
};


union syscfg__pmc {
	u32_t val;
	struct {
		u32_t rsvd__0_22 :23 __packed;
		u32_t mii_rmii_sel :1 __packed;
		u32_t rsvd__24_31 :8 __packed;
	} bit;
};

union syscfg__exticr {
	u32_t val;
	struct {
		u16_t exti;
		u16_t rsvd__16_31;
	} bit;
};

struct stm32f2x_syscfg {
	union syscfg__memrmp memrmp;
	union syscfg__pmc rcr;
	union syscfg__exticr exticr1;
	union syscfg__exticr exticr2;
	union syscfg__exticr exticr3;
	union syscfg__exticr exticr4;
	u32_t cmpcr;
};

#endif /* _STM32F2X_GPIO_REGISTERS_H_ */
