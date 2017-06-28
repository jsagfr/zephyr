/*
 * Copyright (c) 2016 RnDity Sp. z o.o.
 * Copyright (c) 2017 Jérôme Guéry
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Interrupt numbers for STM32F2 family processors.
 *
 * Based on reference manual:
 *   STM32F205xx, STM32F207xx, STM32F215xx and STM32F217xx
 *   advanced ARM ® -based 32-bits MCUs
 *
 * Chapter 8.1.3: Interrupt and exception vectors
 */


#ifndef _STM32F2_SOC_IRQ_H_
#define _STM32F2_SOC_IRQ_H_

/* FIXME: Remove when use of enum line number in IRQ_CONNECT is
 * made possible by ZEP-1165.
 * soc_irq.h, once it is possible, should be removed. */

#define STM32F2_IRQ_WWDG		0
#define STM32F2_IRQ_PVD			1
#define STM32F2_IRQ_TAMPER		2
#define STM32F2_IRQ_RTC			3
#define STM32F2_IRQ_FLASH		4
#define STM32F2_IRQ_RCC			5
#define STM32F2_IRQ_EXTI0		6
#define STM32F2_IRQ_EXTI1		7
#define STM32F2_IRQ_EXTI2		8
#define STM32F2_IRQ_EXTI3		9
#define STM32F2_IRQ_EXTI4		10
#define STM32F2_IRQ_DMA1_STREAM0	11
#define STM32F2_IRQ_DMA1_STREAM1	12
#define STM32F2_IRQ_DMA1_STREAM2	13
#define STM32F2_IRQ_DMA1_STREAM3	14
#define STM32F2_IRQ_DMA1_STREAM4	15
#define STM32F2_IRQ_DMA1_STREAM5	16
#define STM32F2_IRQ_DMA1_STREAM6	17
#define STM32F2_IRQ_ADC			18
#define STM32F2_IRQ_CAN1_TX		19
#define STM32F2_IRQ_CAN1_RX0		20
#define STM32F2_IRQ_CAN1_RX1		21
#define STM32F2_IRQ_CAN1_SCE		22
#define STM32F2_IRQ_EXTI9_5		23
#define STM32F2_IRQ_TIM9		24
#define STM32F2_IRQ_TIM1_BRK		STM32F2_IRQ_TIM9
#define STM32F2_IRQ_TIM10		25
#define STM32F2_IRQ_TIM1_UP		STM32F2_IRQ_TIM10
#define STM32F2_IRQ_TIM11		26
#define STM32F2_IRQ_TIM1_TRG_COM	STM32F2_IRQ_TIM11
#define STM32F2_IRQ_TIM1_CC		27
#define STM32F2_IRQ_TIM2		28
#define STM32F2_IRQ_TIM3		29
#define STM32F2_IRQ_TIM4		30
#define STM32F2_IRQ_I2C1_EV		31
#define STM32F2_IRQ_I2C1_ER		32
#define STM32F2_IRQ_I2C2_EV		33
#define STM32F2_IRQ_I2C2_ER		34
#define STM32F2_IRQ_SPI1		35
#define STM32F2_IRQ_SPI2		36
#define STM32F2_IRQ_USART1		37
#define STM32F2_IRQ_USART2		38
#define STM32F2_IRQ_USART3		39
#define STM32F2_IRQ_EXTI15_10		40
#define STM32F2_IRQ_RTC_ALARM		41
#define STM32F2_IRQ_OTG_FS_EXTI		42
#define STM32F2_IRQ_TIM12		43
#define STM32F2_IRQ_TIM8_BRK		STM32F2_IRQ_TIM12
#define STM32F2_IRQ_TIM13		44
#define STM32F2_IRQ_TIM8_UP		STM32F2_IRQ_TIM13
#define STM32F2_IRQ_TIM14		45
#define STM32F2_IRQ_TIM8_TRG_COM	STM32F2_IRQ_TIM14
#define STM32F2_IRQ_TIM8_CC		46
#define STM32F2_IRQ_DMA1_STREAM7	47
#define STM32F2_IRQ_FSMC		48
#define STM32F2_IRQ_SDIO		49
#define STM32F2_IRQ_TIM5		50
#define STM32F2_IRQ_SPI3		51
#define STM32F2_IRQ_UART4		52
#define STM32F2_IRQ_UART5		53
#define STM32F2_IRQ_TIM6		54
#define STM32F2_IRQ_DAC1_URR		STM32F2_IRQ_TIM6
#define STM32F2_IRQ_DAC2_URR		STM32F2_IRQ_TIM6
#define STM32F2_IRQ_TIM7		55
#define STM32F2_IRQ_DMA2_STREAM0	56
#define STM32F2_IRQ_DMA2_STREAM1	57
#define STM32F2_IRQ_DMA2_STREAM2	58
#define STM32F2_IRQ_DMA2_STREAM3	59
#define STM32F2_IRQ_DMA2_STREAM4	60
#define STM32F2_IRQ_ETH			61
#define STM32F2_IRQ_ETH_EXTI		62
#define STM32F2_IRQ_CAN2_TX		63
#define STM32F2_IRQ_CAN2_RX0		64
#define STM32F2_IRQ_CAN2_RX1		65
#define STM32F2_IRQ_CAN2_SCE		66
#define STM32F2_IRQ_OTG_FS		67
#define STM32F2_IRQ_DMA2_STREAM5	68
#define STM32F2_IRQ_DMA2_STREAM6	69
#define STM32F2_IRQ_DMA2_STREAM7	70
#define STM32F2_IRQ_UART6		71
#define STM32F2_IRQ_I2C3_EV		72
#define STM32F2_IRQ_I2C3_ER		73
#define STM32F2_IRQ_OTG_HS_EP1_OUT	74
#define STM32F2_IRQ_OTG_HS_EP1_IN	75
#define STM32F2_IRQ_OTG_HS_WUP		76
#define STM32F2_IRQ_OTG_HS		77
#define STM32F2_IRQ_DCMI		78
#define STM32F2_IRQ_CRYP		79
#define STM32F2_IRQ_HASH_RNG		80

#endif	/* _STM32F2_SOC_IRQ_H_ */
