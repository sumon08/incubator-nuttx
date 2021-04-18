/****************************************************************************
 * boards/arm/stm32/logger-demo/src/logger-demo.h
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

#ifndef __BOARDS_ARM_STM32_LOGGER_DEMO_SRC_LOGGER_DEMO_H
#define __BOARDS_ARM_STM32_LOGGER_DEMO_SRC_LOGGER_DEMO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* The Logger demo board has 5 LEDs.  one of these is directly to power of the board. Other four is software controllable:
 *
 * RUN LED:  Use to indicate running status of the board. Will be used with arch led. so nuttx  
 * running statsus will reflect.
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


#define SYSTEM_RUN_LED      (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                            GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN14)

#define GPIO_FAULT_LED      (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                            GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN5)


#define GPIO_RX_TX_LED      (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                            GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN6)


#define GPIO_ONLINE_LED     (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                            GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN7)




#define LED_DRIVER_PATH "/dev/userleds"



/*Modem SIM800l*/

#ifdef CONFIG_MODEM_SIM800l

#define MODEM_SIM800l_VLT   (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                            GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN11)


#endif


/* PWM */

#define NUCLEOF103RB_PWMTIMER 1

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture specific initialization
 *
 *   CONFIG_LIB_BOARDCTL=y:
 *     If CONFIG_NSH_ARCHINITIALIZE=y:
 *       Called from the NSH library (or other application)
 *     Otherwise, assumed to be called from some other application.
 *
 *   Otherwise CONFIG_BOARD_LATE_INITIALIZE=y:
 *     Called from board_late_initialize().
 *
 *   Otherwise, bad news:  Never called
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int stm32_pwm_setup(void);
#endif

/****************************************************************************
 * Name: stm32_foc_setup
 *
 * Description:
 *  Initialize FOC peripheral for the board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FOC
int stm32_foc_setup(void);
#endif

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int stm32_adc_setup(void);
#endif

#ifdef CONFIG_MODEM_SIM800l
void modem_sim800l_init();
#endif 

#endif /* __BOARDS_ARM_STM32_NUCLEO_F103RB_SRC_NUCLEO_F103RB_H */
