/****************************************************************************
 * boards/arm/imxrt/ddcnc-ec300/include/ec300_stepgen.h
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

/* Real-time step generator of the EC300 CNC firmware.
 *
 * A FIFO of setpoints (step deltas for six axes over a number of ticks)
 * is executed by a DDA in a timer interrupt.  The interrupt runs at twice
 * the step tick rate: one interrupt raises the step pulses, the next one
 * lowers them and computes the next tick.  The default rate is 10 MHz /
 * period, period 100, so a 100 kHz interrupt and a 50 kHz step tick
 * (10 us step pulses).
 *
 * The interrupt is a NuttX "zero latency" (high priority) interrupt: it is
 * attached directly to the vector table, it runs above the priority masked
 * by the OS critical sections and it cannot be delayed by the OS.  For the
 * same reason it cannot call any OS service.  All communication with it is
 * lock free:
 *
 *   - setpoints go through a single producer / single consumer ring;
 *   - commands that change the state of the step generator are posted in a
 *     mailbox that the interrupt executes at its next tick;
 *   - the status is published by the interrupt under a sequence counter.
 *
 * The API is meant to be called from one thread at a time (the protocol
 * server); a mutex serializes the mailbox users.
 */

#ifndef __BOARDS_ARM_IMXRT_DDCNC_EC300_INCLUDE_EC300_STEPGEN_H
#define __BOARDS_ARM_IMXRT_DDCNC_EC300_INCLUDE_EC300_STEPGEN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STEPGEN_NAXES        6
#define STEPGEN_FIFO_SLOTS   252        /* ring size                     */
#define STEPGEN_FIFO_USABLE  250        /* free slots reported when empty */
#define STEPGEN_BASE_CLOCK   10000000u  /* interrupt rate = clock/period  */
#define STEPGEN_DEF_PERIOD   100        /* -> 100 kHz IRQ, 50 kHz ticks   */
#define STEPGEN_MIN_PERIOD   25         /* -> 400 kHz IRQ, 200 kHz ticks  */

/* Status flags (status byte 0x05 and error replies of the protocol) */

#define STEPGEN_F_RUNNING    0x01  /* run enabled                        */
#define STEPGEN_F_CAPT_ARMED 0x02  /* capture armed                      */
#define STEPGEN_F_CAPT_DONE  0x04  /* capture triggered                  */
#define STEPGEN_F_UNDERRUN   0x10  /* FIFO ran dry inside a move         */
#define STEPGEN_F_OVERRUN    0x20  /* setpoint written to a full FIFO    */
#define STEPGEN_F_ESTOP      0x40  /* E-stop latched                     */

/* Input word.  Bits 0..27 are active low (0 = active); bits 29..31 are
 * active high.  Two bit layouts exist (CONFIG_DDCNC_EC300_STEPGEN_LAYOUT_*):
 *
 *   - "V1": the V1 layout (StepStream "board = v1");
 *   - "EC300": the EC300 layout (the default).
 */

#define STEPGEN_IN_ACTIVE_LOW_MASK 0x0fffffffu

#ifdef CONFIG_DDCNC_EC300_STEPGEN_LAYOUT_V1
#  define STEPGEN_IN_EXTERR    0
#  define STEPGEN_IN_ESTOP1    1
#  define STEPGEN_IN_ESTOP2    2
#  define STEPGEN_IN_PROBE     3
#  define STEPGEN_IN_SYNC      4     /* spindle index                    */
#  define STEPGEN_IN_RUN       5
#  define STEPGEN_IN_PAUSE     6
#  define STEPGEN_IN_HOME(n)   (7 + (n))   /* n = 0..5 */
#  define STEPGEN_IN_HW1A      13
#  define STEPGEN_IN_HW1B      14
#  define STEPGEN_IN_HW2A      15
#  define STEPGEN_IN_HW2B      16
#  define STEPGEN_IN_AUX(n)    (17 + (n))  /* n = 0..5 */
#  define STEPGEN_IN_DRIVEERR  27
#  define STEPGEN_IN_NAUX      6
#else
#  define STEPGEN_IN_EXTERR    0
#  define STEPGEN_IN_ESTOP1    1
#  define STEPGEN_IN_PROBE     2
#  define STEPGEN_IN_SYNC      3     /* spindle index                    */
#  define STEPGEN_IN_RUN       4
#  define STEPGEN_IN_PAUSE     5
#  define STEPGEN_IN_HOME(n)   (6 + (n))   /* n = 0..5 */
#  define STEPGEN_IN_HW1A      12
#  define STEPGEN_IN_HW1B      13
#  define STEPGEN_IN_AUX9      14
#  define STEPGEN_IN_AUX10     15
#  define STEPGEN_IN_AUX(n)    ((n) < 8 ? 16 + (n) : 6 + (n)) /* n = 0..9 */
#  define STEPGEN_IN_DRIVEERR  24
#  define STEPGEN_IN_DRIVEWARN 25
#  define STEPGEN_IN_HW2A      26
#  define STEPGEN_IN_HW2B      27
#  define STEPGEN_IN_NAUX      10
#endif

/* E-stop cause (status cause byte) of a software E-stop, stepgen_estop().
 * Input E-stops report the input bit + 1.
 */

#define STEPGEN_CAUSE_SOFTWARE 0xff

/* Output word bits (protocol "V1" layout) */

#define STEPGEN_OUT_MACHON   0
#define STEPGEN_OUT_DRIVEEN  1
#define STEPGEN_OUT_SPINDLE  2
#define STEPGEN_OUT_COOL1    3
#define STEPGEN_OUT_COOL2    4
#define STEPGEN_OUT_SPINDIR  5
#define STEPGEN_OUT_AUX(n)   (6 + (n))   /* n = 0..9 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* One setpoint: the steps of each axis during "ticks" step ticks */

struct stepgen_seg_s
{
  int16_t  delta[STEPGEN_NAXES]; /* steps per axis (|delta| <= ticks)   */
  uint16_t ticks;                /* duration in step ticks               */
  uint16_t last;                 /* 1: last setpoint of a move           */
  uint32_t tag;                  /* reported in the status while running */
};

/* Status snapshot */

struct stepgen_status_s
{
  uint32_t tag;                  /* tag of the executing setpoint        */
  uint16_t fifo_free;            /* free FIFO slots                      */
  uint8_t  flags;                /* STEPGEN_F_*                          */
  uint8_t  cause;                /* input bit + 1 that latched E-stop    */
  int32_t  pos[STEPGEN_NAXES];   /* actual position, steps               */
  int32_t  hw[2];                /* handwheel quadrature counters        */
  uint32_t spindle_cnt;          /* spindle index pulses                 */
  uint32_t spindle_period;       /* step ticks per revolution, 0=stopped */
  uint32_t outputs;              /* output word                          */
  uint32_t inputs;               /* input word                           */
  uint32_t ticks;                /* free running step tick counter       */
  bool     moving;               /* a setpoint is executing or queued    */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Queue setpoints.  Returns 0, or -ENOSPC if the FIFO had no room for all
 * of them: then nothing is queued, the motion stops and the overrun flag
 * is set.
 */

int stepgen_push(FAR const struct stepgen_seg_s *seg, int count);

/* Report a FIFO overrun (malformed setpoint block): stop, set the
 * overrun flag.
 */

void stepgen_overrun(void);

/* Free FIFO slots (STEPGEN_FIFO_USABLE when empty) */

int stepgen_fifo_free(void);

/* Consistent snapshot of the step generator state */

void stepgen_get_status(FAR struct stepgen_status_s *st);

/* Start (consume the FIFO) or stop.  Returns -EPERM while the E-stop is
 * latched.  With feed sync armed a start waits for the next spindle index.
 */

int stepgen_run_enable(bool run);

/* Empty the FIFO; clear run, errors, capture, tag and spindle counter */

void stepgen_flush(void);

/* Clear the error flags and stop; unlatch the E-stop unless an input is
 * still at its trip level.
 */

void stepgen_clear_error(void);

/* Set the actual position (steps) */

void stepgen_set_position(FAR const int32_t *pos);

/* Set (on != 0) or clear all the output bits in mask.  Ignored while the
 * E-stop is latched (all outputs are then off).
 */

void stepgen_set_outputs(uint32_t on, uint32_t mask);

/* Arm the position capture: it triggers when "one of the inputs in mask
 * (input word bits 0..15) is active" equals level.  stop: stop the motion
 * on the trigger.  mask = 0 disarms.
 */

void stepgen_set_capture(uint32_t mask, uint8_t level, uint8_t stop);

/* Captured position.  Returns 1 if the capture triggered since the last
 * call (and clears that flag), else 0.
 */

uint32_t stepgen_get_capture(FAR int32_t *pos);

/* Latch the E-stop now (e.g. the E-stop key of a pendant): stop at the
 * next step tick, switch all outputs off and refuse run enable, as an
 * input E-stop does.  stepgen_clear_error() unlatches it.
 */

void stepgen_estop(void);

/* Set the E-stop mode: the step generator latches its E-stop (stops, all
 * outputs off, run refused) when an input bit in trip_hi is 1 or an input
 * bit in trip_lo is 0 (input word levels).  The inputs are checked at
 * every step tick.  Unlatches the E-stop unless an input is still at its
 * trip level.
 */

void stepgen_set_estop_mode(uint32_t trip_hi, uint32_t trip_lo);

/* Set the interrupt period: rate = STEPGEN_BASE_CLOCK / period, step ticks
 * at half that rate.  polarity: 1 = step pulses active high (idle low),
 * 0 = active low (idle high), as the flag of the protocol command 0x0a.
 */

int stepgen_set_timer(uint16_t period, uint8_t polarity);

/* Arm (or disarm) the spindle synchronized start (G33) */

void stepgen_arm_feedsync(bool arm);

/* Spindle PWM.  period and duty are in 80 MHz timer ticks, the units of
 * the protocol.  Returns -ENOSYS when the board has no PWM output.
 */

int stepgen_set_pwm(int channel, uint32_t period, uint32_t duty);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_IMXRT_DDCNC_EC300_INCLUDE_EC300_STEPGEN_H */
