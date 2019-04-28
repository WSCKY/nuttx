/************************************************************************************
 * configs/stm32f401-navigation/src/stm32_lsm9ds1.c
 *
 *   Copyright (C) 2015 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/sensors/lsm9ds1_spi.h>

#include "stm32.h"
#include "stm32_spi.h"
#include "nav_board.h"

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_LSM9DS1_SPI) && !defined(CONFIG_SENSORS_LSM9DS1)

int stm32_lsm9ds1_initialize(void)
{
  int ret = OK;
  int port = SPIPORT_LSM9DS1;

  /* Get the spi bus instance. */

  struct spi_dev_s *spi = stm32_spibus_initialize(port);
  if (spi == NULL)
    {
      return -ENODEV;
    }

  /* TODO: configure EXTI pin */

  /* Register the chip with the device driver. */
  ret = lsm9ds1accel_register("/dev/acc0", spi, 0); if(ret != OK) return ret;
  ret = lsm9ds1gyro_register("/dev/gyr0", spi, 0); if(ret != OK) return ret;
  ret = lsm9ds1mag_register("/dev/mag0", spi, 1); if(ret != OK) return ret;

  return ret;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_LSM9DS1_SPI */
