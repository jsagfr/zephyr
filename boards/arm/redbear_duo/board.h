/*
 * Copyright (c) 2016 Linaro Limited.
 * Copyright (c) 2017 Jérôme Guéry
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __INC_BOARD_H
#define __INC_BOARD_H

#include <soc.h>

/* USER push button */
#define USER_PB_GPIO_PORT	"GPIOB"
#define USER_PB_GPIO_PIN	2

/* LED0 red LED */
#define LED0R_GPIO_PORT	"GPIOB"
#define LED0R_GPIO_PIN	11

/* LED0 green LED */
#define LED0G_GPIO_PORT	"GPIOB"
#define LED0G_GPIO_PIN	1

/* LED0 blue LED */
#define LED0B_GPIO_PORT	"GPIOB"
#define LED0B_GPIO_PIN	0

/* Create aliases to make the basic samples work */
#define SW0_GPIO_NAME	USER_PB_GPIO_PORT
#define SW0_GPIO_PIN	USER_PB_GPIO_PIN
#define LED0_GPIO_PORT	LED0R_GPIO_PORT
#define LED0_GPIO_PIN	LED0R_GPIO_PIN

#endif /* __INC_BOARD_H */
