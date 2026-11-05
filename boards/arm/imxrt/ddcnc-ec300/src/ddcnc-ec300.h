/****************************************************************************
 * boards/arm/imxrt/ddcnc-ec300/src/ddcnc-ec300.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __BOARDS_ARM_IMXRT_DDCNC_EC300_SRC_DDCNC_EC300_H
#define __BOARDS_ARM_IMXRT_DDCNC_EC300_SRC_DDCNC_EC300_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EC300 GPIO Pin Definitions ***********************************************/

/* LEDs
 *
 * The CONN LED is connected to GPIO_SD_B1_00 (GPIO3_IO00).  It is lit when
 * the pin is driven low.
 */

#define GPIO_LED        (GPIO_OUTPUT | IOMUX_LED_DEFAULT | \
                         GPIO_OUTPUT_ONE | GPIO_PORT3 | GPIO_PIN0)  /* SD_B1_00 */

/* Machine I/O (from the Remora-RT1052 EC300 configuration).  All of it is
 * opto-isolated on the board.
 *
 * Step and direction outputs: all of them are on GPIO1, so the step
 * generator drives them with single writes to the GPIO1 DR_SET / DR_CLEAR
 * registers.  Pin numbers, axis order X Y Z A B C.
 */

#define EC300_STEP_PINS     { 22, 24, 18, 27, 16, 20 }  /* GPIO1_IOxx */
#define EC300_DIR_PINS      { 17, 31, 25, 21, 23, 19 }  /* GPIO1_IOxx */

/* Digital outputs OT01..OT10, all on GPIO4 */

#define EC300_OUT_PINS      { 2, 1, 3, 4, 5, 6, 7, 8, 9, 10 } /* GPIO4_IOxx */
#define EC300_NOUTPUTS      10

#define IOMUX_STEP_DEFAULT  (IOMUX_CMOS_OUTPUT | IOMUX_DRIVE_40OHM | \
                             IOMUX_SLEW_FAST | IOMUX_SPEED_MEDIUM)
#define IOMUX_OUT_DEFAULT   (IOMUX_CMOS_OUTPUT | IOMUX_DRIVE_50OHM | \
                             IOMUX_SLEW_SLOW | IOMUX_SPEED_LOW)
#define IOMUX_IN_DEFAULT    (IOMUX_SCHMITT_TRIGGER | IOMUX_PULL_UP_47K)

/* Digital inputs: GPIO port index and pin of each input of the EC300.
 * The inputs are active low.  The function each one has in the input word
 * of the step generator is assigned in imxrt_stepgen.c.
 */

#define EC300_IN01          GPIO4, 26
#define EC300_IN02          GPIO4, 27
#define EC300_IN03          GPIO4, 21
#define EC300_IN04          GPIO4, 24
#define EC300_IN05          GPIO4, 28
#define EC300_IN06          GPIO4, 25
#define EC300_IN07          GPIO4, 29
#define EC300_IN08          GPIO4, 23
#define EC300_IN09          GPIO4, 31
#define EC300_IN10          GPIO3, 18
#define EC300_IN11          GPIO3, 21
#define EC300_IN12          GPIO3, 20
#define EC300_FHA           GPIO3, 26   /* 2nd handwheel A                */
#define EC300_FHB           GPIO4, 30   /* 2nd handwheel B                */
#define EC300_SRO           GPIO3, 19
#define EC300_SRJ           GPIO4, 20
#define EC300_INDEX         GPIO3, 22   /* spindle index                  */
#define EC300_WHA           GPIO3, 16   /* MPG handwheel A                */
#define EC300_WHB           GPIO3, 17   /* MPG handwheel B                */
#define EC300_CIN           GPIO4, 18   /* MPG axis selector              */
#define EC300_XIN           GPIO4, 17
#define EC300_YIN           GPIO4, 16
#define EC300_ZIN           GPIO4, 15
#define EC300_AIN           GPIO4, 14
#define EC300_X100IN        GPIO4, 13   /* MPG step selector              */
#define EC300_X10IN         GPIO4, 12
#define EC300_EP            GPIO4, 11   /* MPG E-stop                     */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int imxrt_bringup(void);
#endif

/****************************************************************************
 * Name: imxrt_autoled_initialize
 *
 * Description:
 *   Initialize NuttX-controlled LED logic
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void imxrt_autoled_initialize(void);
#endif

/****************************************************************************
 * Name: imxrt_stepgen_initialize
 *
 * Description:
 *   Initialize the CNC step generator (step/dir/IO pins, step timer and
 *   its zero latency interrupt).
 *
 ****************************************************************************/

#ifdef CONFIG_DDCNC_EC300_STEPGEN
int imxrt_stepgen_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_IMXRT_DDCNC_EC300_SRC_DDCNC_EC300_H */
