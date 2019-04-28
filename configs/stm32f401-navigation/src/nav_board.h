/****************************************************************************
 * configs/stm32f401-navigation/src/nav_board.h
 *
 *   Copyright (C) 2011-2012, 2015-2016, 2018 Gregory Nutt. All rights
 *     reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __CONFIGS_NAV_BOARD_SRC_NAV_BOARD_H
#define __CONFIGS_NAV_BOARD_SRC_NAV_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <arch/stm32/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NAVIGATION_BOARD_NEW_V1                  (1)

/* Configuration ************************************************************/
#if !NAVIGATION_BOARD_NEW_V1
#define HAVE_AT24 1
/* AT24 Serial EEPROM */
#define AT24_I2C_BUS   1 /* AT24CXX connected to I2C1 */
#define AT24_MINOR     0

//#define PWR_CTRL_PIN      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|
//                           GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN9)

#if !defined(CONFIG_MTD_AT24XX) || !defined(CONFIG_STM32_I2C1)
#  undef HAVE_AT24
#endif

/* Can't support AT24 features if mountpoints are disabled or if we were not
 * asked to mount the AT25 part
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || \
   !defined(CONFIG_NAV_BOARD_AT24_BLOCKMOUNT)
#  undef HAVE_AT24
#endif

/* If we are going to mount the AT24, then they user must also have told
 * us what to do with it by setting one of these.
 */

#ifndef CONFIG_FS_NXFFS
#  undef CONFIG_NAV_BOARD_AT24_NXFFS
#endif

#if !defined(CONFIG_NAV_BOARD_AT24_FTL) && \
    !defined(CONFIG_NAV_BOARD_AT24_NXFFS)
#  undef HAVE_AT24
#endif
#endif /* !(NAVIGATION_BOARD_NEW_V1) */
/* How many SPI modules does this chip support? */

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 3
#  undef CONFIG_STM32_SPI3
#endif

#if NAVIGATION_BOARD_NEW_V1
/* Checking needed by W25 Flash */
#define HAVE_W25      1

/* Can't support the W25 device if it SPI1 or W25 support is not enabled */
#if !defined(CONFIG_STM32_SPI1) || !defined(CONFIG_MTD_W25)
#  undef HAVE_W25
#endif

/* Can't support W25 features if mountpoints are disabled */
#ifdef CONFIG_DISABLE_MOUNTPOINT
#  undef HAVE_W25
#endif

/* Default W25 minor number */
#if defined(HAVE_W25) && !defined(CONFIG_NSH_W25MINOR)
#  define CONFIG_NSH_W25MINOR 0
#endif
#endif /* NAVIGATION_BOARD_NEW_V1 */
/* Assume that we have everything */

#define HAVE_USBDEV     1
#define HAVE_USBHOST    1
#define HAVE_USBMONITOR 1

/* Can't support USB host or device features if USB OTG FS is not enabled */

#ifndef CONFIG_STM32_OTGFS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB device monitor if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB host is USB host is not enabled */

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#if !defined(CONFIG_USBDEV_TRACE) || !defined(CONFIG_USBMONITOR)
#  undef HAVE_USBMONITOR
#endif

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* STM32F401-Navigation Board GPIOs **************************************************/
/* LEDs */
#if NAVIGATION_BOARD_NEW_V1
#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN3)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN4)
#define GPIO_LED3       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN5)
#else
#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN12)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN13)
#define GPIO_LED3       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN14)
#endif /* NAVIGATION_BOARD_NEW_V1 */
/* BUTTONS -- NOTE that all have EXTI interrupts configured */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#if NAVIGATION_BOARD_NEW_V1
#define GPIO_BTN_USER   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN15)
#else
#define GPIO_BTN_USER   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN15)
#endif /* NAVIGATION_BOARD_NEW_V1 */

#if NAVIGATION_BOARD_NEW_V1
#define GPIO_CS_W25Q64       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                              GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)
#define GPIO_WP_W25Q64       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                              GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN0)

#define SPIPORT_LSM9DS1      2
#define GPIO_DEN_LSM9DS1_AG  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                              GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN9)
#define GPIO_DRDY_LSM9DS1_M  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN8)
#define GPIO_INT_LSM9DS1_M   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN12)
#define GPIO_INT1_LSM9DS1_AG (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN8)
#define GPIO_INT2_LSM9DS1_AG (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN9)

#define GPIO_CS_LSM9DS1_AG   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                              GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN2)
#define GPIO_CS_LSM9DS1_M    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                              GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN10)
#else
#define SPIPORT_MPU9250   1
#define SPIMINOR_MPU9250  0
/* SPI chip selects */
#define GPIO_CS_MPU9250    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)
//#define GPIO_EXTI_MPU6000 (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz |
//                           GPIO_OPENDRAIN | GPIO_PORTC | GPIO_PIN4)
#define DEVNODE_MPU9250   "/dev/imu0"
#endif /* NAVIGATION_BOARD_NEW_V1 */
/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the stm32f401-navigation
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void);

/************************************************************************************
 * Name: stm32_w25initialize
 *
 * Description:
 *   Called to initialize Winbond W25 memory
 *
 ************************************************************************************/
#ifdef HAVE_W25
int stm32_w25initialize(int minor);
#endif /* HAVE_W25 */

/****************************************************************************
 * Name: stm32_at24_automount
 *
 * Description:
 *   Called from stm32_at24_automount very early in initialization to setup
 *   AT24C04 EEPROM for the STM32F401-Navigation board.
 *
 ****************************************************************************/
#ifdef HAVE_AT24
int stm32_at24_automount(int minor);
#endif /* HAVE_AT24 */

#ifdef CONFIG_SENSORS_MPU9250
int stm32_mpu9250_initialize(void);
#endif

#ifdef CONFIG_SENSORS_LSM9DS1_SPI
//int stm32_lsm9ds1_initialize(void);
#endif
/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in initialization to setup
 *   USB-related GPIO pins for the STM32F4Discovery board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_OTGFS
void weak_function stm32_usbinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality. This function will start a thread that will monitor for
 *   device connection/disconnection events.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_OTGFS) && defined(CONFIG_USBHOST)
int stm32_usbhost_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_NAV_BOARD_SRC_NAV_BOARD_H */
