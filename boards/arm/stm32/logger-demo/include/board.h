/****************************************************************************
 * boards/arm/stm32/logger-demo/include/board.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32_LOGGER_DEMO_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_LOGGER_DEMO_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#include <stdint.h>
#include <stdbool.h>
#endif

#ifdef __KERNEL__
#include "stm32.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* HSI - Internal 8 MHz RC Oscillator
 * LSI - 32 KHz RC
 * HSE - 8 MHz from MCO output of ST-LINK
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL 8000000ul /* X1 on board */

#define STM32_HSI_FREQUENCY 8000000ul
#define STM32_LSI_FREQUENCY 40000 /* Between 30kHz and 60kHz */
#define STM32_HSE_FREQUENCY STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY 32768 /* X2 on board */

/* PLL source is HSE/1, PLL multipler is 9:
 *   PLL frequency is 8MHz (XTAL) x 9 = 72MHz
 */

#define STM32_CFGR_PLLSRC RCC_CFGR_PLLSRC
#define STM32_CFGR_PLLXTPRE 0
#define STM32_CFGR_PLLMUL RCC_CFGR_PLLMUL_CLKx9
#define STM32_PLL_FREQUENCY (9 * STM32_BOARD_XTAL)

/* Use the PLL and set the SYSCLK source to be the PLL */

#define STM32_SYSCLK_SW RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY STM32_PLL_FREQUENCY

/* AHB clock (HCLK) is SYSCLK (72MHz) */

#define STM32_RCC_CFGR_HPRE RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY STM32_SYSCLK_FREQUENCY

/* APB2 clock (PCLK2) is HCLK (72MHz) */

#define STM32_RCC_CFGR_PPRE2 RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY STM32_HCLK_FREQUENCY
#define STM32_APB2_CLKIN (STM32_PCLK2_FREQUENCY) /* Timers 1 and 8, 15-17 */

/* APB1 clock (PCLK1) is HCLK/2 (36MHz) */

#define STM32_RCC_CFGR_PPRE1 RCC_CFGR_PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY (STM32_HCLK_FREQUENCY / 2)

/* APB2 TIM 1 will receive PCLK2 (72MHz) */

#define STM32_APB2_TIM1_CLKIN (STM32_PCLK2_FREQUENCY)

/* APB1 TIM 2-4 will be twice PCLK1 (72MHz) */

#define STM32_APB1_TIM2_CLKIN (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN (2 * STM32_PCLK1_FREQUENCY)

/* LED definitions **********************************************************/

/* The Nucleo F103RB board has three LEDs.  Two of these are controlled by
 * logic on the board and are not available for software control:
 *
 * LD1 COM:  LD1 default status is red.  LD1 turns to green to indicate that
 *           communications are in progress between the PC and the
 *           ST-LINK/V2-1.
 * LD3 PWR:  red LED indicates that the board is powered.
 *
 * And one can be controlled by software:
 *
 * User LD2: green LED is a user LED connected to the I/O PA5 of the
 *           STM32F103RBT6.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LED in
 * any way.  The following definition is used to access the LED.
 */

/* LED index values for use with board_userled() */

#define BOARD_USER_LED_FAULT 0 /* User LD2 */
#define BOARD_USER_LED_RX_TX 1
#define BOARD_USER_LED_ONLINE 2
#define BOARD_NLEDS 3

/* LED bits for use with board_userled_all() */

#define BOARD_USER_LED_FAULT_BIT (1 << BOARD_USER_LED_FAULT)
#define BOARD_USER_LED_RX_TX_BIT (1 << BOARD_USER_LED_RX_TX)
#define BOARD_USER_LED_ONLINE_BIT (1 << BOARD_USER_LED_ONLINE)
/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the Nucleo F103RB.  The following definitions describe how NuttX controls
 * the LED:
 *
 *   SYMBOL              Meaning                  LED1 state
 *   ------------------  -----------------------  ----------
 *   LED_STARTED         NuttX has been started   OFF
 *   LED_HEAPALLOCATE    Heap has been allocated  OFF
 *   LED_IRQSENABLED     Interrupts enabled       OFF
 *   LED_STACKCREATED    Idle stack created       ON
 *   LED_INIRQ           In an interrupt          No change
 *   LED_SIGNAL          In a signal handler      No change
 *   LED_ASSERTION       An assertion failed      No change
 *   LED_PANIC           The system has crashed   Blinking
 *   LED_IDLE            STM32 is is sleep mode   Not used
 */

#define LED_STARTED                         0
#define LED_HEAPALLOCATE                    0
#define LED_IRQSENABLED                     0
#define LED_STACKCREATED                    1
#define LED_INIRQ                           2
#define LED_SIGNAL                          2
#define LED_ASSERTION                       2
#define LED_PANIC                           1

/* USART1 pin definition. Connected to nsh consol */

// #define GPIO_USART1_RX GPIO_USART1_RX_1 /* PA10 */
// #define GPIO_USART1_TX GPIO_USART1_TX_1 /* PA9  */


// /* USART2 pin definition. Connected to nsh consol */

// #define GPIO_USART2_RX GPIO_USART2_RX_1 /* PA10 */
// #define GPIO_USART2_TX GPIO_USART2_TX_1 /* PA9  */


/* UART4 pin definition. Connected to nsh consol */

#define GPIO_UART4_RX (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN10) /* PA10 */
#define GPIO_UART4_TX (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN11) /* PA9  */


// #  define GPIO_USART3_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN10)
// #  define GPIO_USART3_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN11)

// /* UART5 pin definition. Connected to nsh consol */

// #define GPIO_USART1_RX GPIO_USART1_RX_1 /* PA10 */
// #define GPIO_USART1_TX GPIO_USART1_TX_1 /* PA9  */




#endif /* __BOARDS_ARM_STM32_NUCLEO_F103RB_INCLUDE_BOARD_H */
