/*
 * Copyright (c) 2016 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include "soc.h"
#include <device.h>
#include <misc/util.h>
#include <pinmux/stm32/pinmux_stm32.h>
#include <drivers/clock_control/stm32_clock_control.h>

#define PAD(AF, func)				\
				[AF - 1] = func

/* #define _PINMUX_PWM(pin, port, chan)		\ */
/* 	PAD(STM32F4_PINMUX_FUNC_##pin##_PWM##port##_CH##chan,\ */
/* 	    STM32F4X_PIN_CONFIG_AF_PUSH_UP), */

#define _PINMUX_UART(pin, port, dir)		\
	PAD(STM32F2_PINMUX_FUNC_##pin##_##port##_##dir, \
	    STM32F2X_PIN_CONFIG_AF_PUSH_UP),

/* Blank pinmux by default */
/* #define PINMUX_PWM2(pin, chan) */
#define PINMUX_UART1(pin, dir)
#define PINMUX_UART2(pin, dir)
#define PINMUX_UART3(pin, dir)
#define PINMUX_UART4(pin, dir)
#define PINMUX_UART5(pin, dir)
#define PINMUX_UART6(pin, dir)
#define PINMUX_UART7(pin, dir)
#define PINMUX_UART8(pin, dir)
#define PINMUX_UART9(pin, dir)
#define PINMUX_UART10(pin, dir)

/* #ifdef CONFIG_PWM_STM32_2 */
/* 	#undef PINMUX_PWM2 */
/* 	#define PINMUX_PWM2(pin, chan)		_PINMUX_PWM(pin, 2, chan) */
/* #endif */

#ifdef CONFIG_UART_STM32_PORT_1
	#undef PINMUX_UART1
	#define PINMUX_UART1(pin, dir)		_PINMUX_UART(pin, USART1, dir)
#endif

#ifdef CONFIG_UART_STM32_PORT_2
	#undef PINMUX_UART2
	#define PINMUX_UART2(pin, dir)		_PINMUX_UART(pin, USART2, dir)
#endif

#ifdef CONFIG_UART_STM32_PORT_3
	#undef PINMUX_UART3
	#define PINMUX_UART3(pin, dir)		_PINMUX_UART(pin, USART3, dir)
#endif

#ifdef CONFIG_UART_STM32_PORT_4
	#undef PINMUX_UART4
	#define PINMUX_UART4(pin, dir)		_PINMUX_UART(pin, UART4, dir)
#endif

#ifdef CONFIG_UART_STM32_PORT_5
	#undef PINMUX_UART5
	#define PINMUX_UART5(pin, dir)		_PINMUX_UART(pin, UART5, dir)
#endif

#ifdef CONFIG_UART_STM32_PORT_6
	#undef PINMUX_UART6
	#define PINMUX_UART6(pin, dir)		_PINMUX_UART(pin, USART6, dir)
#endif

#ifdef CONFIG_UART_STM32_PORT_7
	#undef PINMUX_UART7
	#define PINMUX_UART7(pin, dir)		_PINMUX_UART(pin, UART7, dir)
#endif

#ifdef CONFIG_UART_STM32_PORT_8
	#undef PINMUX_UART8
	#define PINMUX_UART8(pin, dir)		_PINMUX_UART(pin, UART8, dir)
#endif

#ifdef CONFIG_UART_STM32_PORT_9
	#undef PINMUX_UART9
	#define PINMUX_UART9(pin, dir)		_PINMUX_UART(pin, UART9, dir)
#endif

#ifdef CONFIG_UART_STM32_PORT_10
	#undef PINMUX_UART10
	#define PINMUX_UART10(pin, dir)		_PINMUX_UART(pin, UART10, dir)
#endif

/* #define PINMUX_PWM(pin, pwm, chan)		PINMUX_##pwm(pin, chan) */
#define PINMUX_UART(pin, port, dir)		PINMUX_##port(pin, dir)

/* Port A */
static const stm32_pin_func_t pin_pa0_funcs[] = {
	PINMUX_UART(PA0, UART4, TX)
};

static const stm32_pin_func_t pin_pa1_funcs[] = {
	PINMUX_UART(PA1, UART4, RX)
};

static const stm32_pin_func_t pin_pa2_funcs[] = {
	PINMUX_UART(PA2, UART2, TX)
};

static const stm32_pin_func_t pin_pa3_funcs[] = {
	PINMUX_UART(PA3, UART2, RX)
};

static const stm32_pin_func_t pin_pa9_funcs[] = {
	PINMUX_UART(PA9, UART1, TX)
};

static const stm32_pin_func_t pin_pa10_funcs[] = {
	PINMUX_UART(PA10, UART1, RX)
};

/* Port B */
static const stm32_pin_func_t pin_pb6_funcs[] = {
	PINMUX_UART(PB6, UART1, TX)
};

static const stm32_pin_func_t pin_pb7_funcs[] = {
	PINMUX_UART(PB7, UART1, RX)
};

static const stm32_pin_func_t pin_pb10_funcs[] = {
	PINMUX_UART(PB10, UART3, TX)
};

static const stm32_pin_func_t pin_pb11_funcs[] = {
	PINMUX_UART(PB11, UART3, RX)
};

/* Port C */
static const stm32_pin_func_t pin_pc6_funcs[] = {
	PINMUX_UART(PC6, UART6, TX)
};

static const stm32_pin_func_t pin_pc7_funcs[] = {
	PINMUX_UART(PC7, UART6, RX)
};

static const stm32_pin_func_t pin_pc10_funcs[] = {
	PINMUX_UART(PC10, UART3, TX)
	PINMUX_UART(PC10, UART4, TX)
};

static const stm32_pin_func_t pin_pc11_funcs[] = {
	PINMUX_UART(PC11, UART3, RX)
	PINMUX_UART(PC11, UART4, RX)
};

static const stm32_pin_func_t pin_pc12_funcs[] = {
	PINMUX_UART(PC12, UART5, TX)
};

/* Port D */
static const stm32_pin_func_t pin_pd2_funcs[] = {
	PINMUX_UART(PD2, UART5, RX)
};

static const stm32_pin_func_t pin_pd5_funcs[] = {
	PINMUX_UART(PD5, UART2, TX)
};

static const stm32_pin_func_t pin_pd6_funcs[] = {
	PINMUX_UART(PD6, UART2, RX)
};

static const stm32_pin_func_t pin_pd8_funcs[] = {
	PINMUX_UART(PD8, UART3, TX)
};

static const stm32_pin_func_t pin_pd9_funcs[] = {
	PINMUX_UART(PD9, UART3, RX)
};

/* Port G */
static const stm32_pin_func_t pin_pg9_funcs[] = {
	PINMUX_UART(PG9, UART6, RX)
};

static const stm32_pin_func_t pin_pg14_funcs[] = {
	PINMUX_UART(PG14, UART6, TX)
};

/**
 * @brief pin configuration
 */
static const struct stm32_pinmux_conf pins[] = {
	/* Port A */
	STM32_PIN_CONF(STM32_PIN_PA0, pin_pa0_funcs),
	STM32_PIN_CONF(STM32_PIN_PA1, pin_pa1_funcs),
	STM32_PIN_CONF(STM32_PIN_PA2, pin_pa2_funcs),
	STM32_PIN_CONF(STM32_PIN_PA3, pin_pa3_funcs),
	STM32_PIN_CONF(STM32_PIN_PA9, pin_pa9_funcs),
	STM32_PIN_CONF(STM32_PIN_PA10, pin_pa10_funcs),

	/* Port B */
	STM32_PIN_CONF(STM32_PIN_PB6, pin_pb6_funcs),
	STM32_PIN_CONF(STM32_PIN_PB7, pin_pb7_funcs),
	STM32_PIN_CONF(STM32_PIN_PB10, pin_pb10_funcs),
	STM32_PIN_CONF(STM32_PIN_PB11, pin_pb11_funcs),

	/* Port C */
	STM32_PIN_CONF(STM32_PIN_PC6, pin_pc6_funcs),
	STM32_PIN_CONF(STM32_PIN_PC7, pin_pc7_funcs),
	STM32_PIN_CONF(STM32_PIN_PC10, pin_pc10_funcs),
	STM32_PIN_CONF(STM32_PIN_PC11, pin_pc11_funcs),
	STM32_PIN_CONF(STM32_PIN_PC12, pin_pc12_funcs),

	/* Port D */
	STM32_PIN_CONF(STM32_PIN_PD2, pin_pd2_funcs),
	STM32_PIN_CONF(STM32_PIN_PD5, pin_pd5_funcs),
	STM32_PIN_CONF(STM32_PIN_PD6, pin_pd6_funcs),
	STM32_PIN_CONF(STM32_PIN_PD8, pin_pd8_funcs),
	STM32_PIN_CONF(STM32_PIN_PD9, pin_pd9_funcs),

	/* Port G */
	STM32_PIN_CONF(STM32_PIN_PG9, pin_pg9_funcs),
	STM32_PIN_CONF(STM32_PIN_PG14, pin_pg14_funcs),
};

int stm32_get_pin_config(int pin, int func)
{
	/* GPIO function is always available, to save space it is not
	 * listed in alternate functions array
	 */
	if (func == STM32_PINMUX_FUNC_GPIO) {
		return STM32F2X_PIN_CONFIG_BIAS_HIGH_IMPEDANCE;
	}

	/* analog function is another 'known' setting */
	if (func == STM32_PINMUX_FUNC_ANALOG) {
		return STM32F2X_PIN_CONFIG_ANALOG;
	}

	func -= 1;

	for (int i = 0; i < ARRAY_SIZE(pins); i++) {
		if (pins[i].pin == pin) {
			if (func > pins[i].nfuncs) {
				return -EINVAL;
			}

			return pins[i].funcs[func];
		}
	}

	return -EINVAL;
}
