/****************************************************************************
 * boards/arm/stm32/nucleo-f103rb/src/stm32_userleds.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "stm32.h"
#include "logger-demo.h"

#ifdef CONFIG_USER_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure LED1 GPIO for output */

  stm32_configgpio(GPIO_FAULT_LED);
  stm32_configgpio(GPIO_RX_TX_LED);
  stm32_configgpio(GPIO_ONLINE_LED);
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  switch (led)
  {
  case BOARD_USER_LED_FAULT:
    stm32_gpiowrite(GPIO_FAULT_LED, ledon);
    break;

  case BOARD_USER_LED_RX_TX:
    stm32_gpiowrite(GPIO_RX_TX_LED, ledon);
    break;

  case BOARD_USER_LED_ONLINE:
    stm32_gpiowrite(GPIO_ONLINE_LED, ledon);
    break;

  default:
    break;
  }
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  // stm32_gpiowrite(GPIO_LED1, (ledset & BOARD_LED1_BIT) != 0);
  // stm32_gpiowrite(GPIO_FAULT_LED, ledon);
}

#endif /* !CONFIG_ARCH_LEDS */
