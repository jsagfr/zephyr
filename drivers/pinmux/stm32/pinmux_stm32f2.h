/*
 * Copyright (c) 2017 Jérôme Guéry
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _STM32F2_PINMUX_H_
#define _STM32F2_PINMUX_H_

/**
 * @file Header for STM32F2 pin multiplexing helper
 */

/* Port A */
#define STM32F2_PINMUX_FUNC_PA0_PWM2_CH1	STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP
#define STM32F2_PINMUX_FUNC_PA0_UART4_TX	STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PA1_PWM2_CH2	STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP
#define STM32F2_PINMUX_FUNC_PA1_UART4_RX	STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PA2_PWM2_CH3	STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP
#define STM32F2_PINMUX_FUNC_PA2_USART2_TX	STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PA3_PWM2_CH4	STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP
#define STM32F2_PINMUX_FUNC_PA3_USART2_RX	STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PA5_PWM2_CH1	STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PA8_PWM1_CH1	STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP
#define STM32F2_PINMUX_FUNC_PA8_I2C3_SCL	STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP

#define STM32F2_PINMUX_FUNC_PA9_PWM1_CH2	STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP
#define STM32F2_PINMUX_FUNC_PA9_USART1_TX	STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PA10_PWM1_CH3	STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP
#define STM32F2_PINMUX_FUNC_PA10_USART1_RX	STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PA11_PWM1_CH4	STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PA15_PWM2_CH1	STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP

/* Port B */
#define STM32F2_PINMUX_FUNC_PB3_PWM2_CH2	STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PB6_I2C1_SCL	STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP
#define STM32F2_PINMUX_FUNC_PB6_USART1_TX	STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PB7_I2C1_SDA	STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP
#define STM32F2_PINMUX_FUNC_PB7_USART1_RX	STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PB8_I2C1_SCL	STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP

#define STM32F2_PINMUX_FUNC_PB9_I2C1_SDA	STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP

#define STM32F2_PINMUX_FUNC_PB10_PWM2_CH3	STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP
#define STM32F2_PINMUX_FUNC_PB10_I2C2_SCL	STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP
#define STM32F2_PINMUX_FUNC_PB10_USART3_TX	STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PB11_PWM2_CH4	STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP
#define STM32F2_PINMUX_FUNC_PB11_I2C2_SDA	STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP
#define STM32F2_PINMUX_FUNC_PB11_USART3_RX	STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP

/* Port C */
#define STM32F2_PINMUX_FUNC_PC6_USART6_TX	STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PC7_USART6_RX	STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PC9_I2C3_SDA	STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP

#define STM32F2_PINMUX_FUNC_PC10_USART3_TX	STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP
#define STM32F2_PINMUX_FUNC_PC10_UART4_TX	STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PC11_USART3_RX	STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP
#define STM32F2_PINMUX_FUNC_PC11_UART4_RX	STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PC12_UART5_TX	STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP

/* Port D */
#define STM32F2_PINMUX_FUNC_PD2_UART5_TX	STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PD5_USART2_TX	STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PD6_USART2_RX	STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PD8_USART3_TX	STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PB9_USART3_RX	STM32_PINMUX_ALT_FUNC_7 | STM32_PUSHPULL_PULLUP

/* Port E */
#define STM32F2_PINMUX_FUNC_PE9_PWM1_CH1	STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PE11_PWM1_CH2	STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PE13_PWM1_CH3	STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PE14_PWM1_CH4	STM32_PINMUX_ALT_FUNC_1 | STM32_PUSHPULL_PULLUP

/* Port F */

#define STM32F2_PINMUX_FUNC_PF0_I2C2_SDA	STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP

#define STM32F2_PINMUX_FUNC_PF1_I2C2_SCL	STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP


/* Port G */
#define STM32F2_PINMUX_FUNC_PG9_USART6_RX	STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP

#define STM32F2_PINMUX_FUNC_PG14_USART6_TX	STM32_PINMUX_ALT_FUNC_8 | STM32_PUSHPULL_PULLUP


/* Port H */
#define STM32F2_PINMUX_FUNC_PH4_I2C2_SCL	STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP

#define STM32F2_PINMUX_FUNC_PH5_I2C2_SDA	STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP

#define STM32F2_PINMUX_FUNC_PH5_I2C3_SCL	STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP

#define STM32F2_PINMUX_FUNC_PH6_I2C3_SDA	STM32_PINMUX_ALT_FUNC_4 | STM32_OPENDRAIN_PULLUP


#endif /* _STM32F2_PINMUX_H_ */
