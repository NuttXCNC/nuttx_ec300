/****************************************************************************
 * boards/arm/imxrt/ddcnc-ec300/src/imxrt_stepgen.c
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

/* CNC step generator for the EC300:
 *
 *   - Timer interrupt at 10 MHz / period.
 *     The interrupts alternate between a "rise" phase, which raises the
 *     step outputs computed before, and a "tick" phase, which lowers them
 *     and runs one step tick.  Step pulses are one interrupt period wide
 *     (10 us at the default rate) and the direction outputs change one
 *     interrupt period before the first step of a setpoint.
 *   - Midpoint DDA per axis: err = |d| - T, inc = 2|d|, dec = 2T; at each
 *     tick an axis steps when err >= 0 (then err -= dec), then err += inc.
 *     Exactly |d| steps are made in T ticks.
 *   - 252-slot setpoint ring, 250 usable.  A FIFO that runs dry after a
 *     setpoint without the "last" mark sets the underrun flag and stops.
 *   - Handwheel quadrature decoded at every tick with a table;
 *     spindle index counted with a 100-tick dead time, its period measured
 *     in ticks; feed synchronized start on the index.
 *   - Position capture and E-stop checks at every tick.
 *
 * The interrupt is a zero latency (CONFIG_ARCH_HIPRI_INTERRUPT) interrupt:
 * it is attached to the RAM vector table and runs above the OS, so it never
 * waits for a critical section.  It runs from RAM (.ramfunc) because the
 * board executes in place from the QSPI flash.  It may not call any OS
 * service: all communication with it is lock free (see ec300_stepgen.h).
 *
 * Define STEPGEN_HOST_TEST to build the logic on a host for unit tests;
 * the test then provides the register access functions.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifndef STEPGEN_HOST_TEST
#  include <debug.h>
#  include <nuttx/arch.h>
#  include <nuttx/irq.h>
#  include <nuttx/mutex.h>
#  include <arch/barriers.h>
#  include <arch/irq.h>
#  include <arch/armv7-m/nvicpri.h>

#  include "arm_internal.h"
#  include "ram_vectors.h"
#  include "nvic.h"
#  include "hardware/imxrt_ccm.h"
#  include "hardware/imxrt_gpio.h"
#  include "hardware/imxrt_gpt.h"
#  include "imxrt_gpio.h"
#  include "imxrt_periphclks.h"
#  include "ddcnc-ec300.h"
#  include <arch/board/board.h>
#endif

#include <arch/board/ec300_stepgen.h>

#ifdef CONFIG_DDCNC_EC300_STEPGEN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef STEPGEN_HOST_TEST
#  if IMXRT_PERCLK_CLK_SEL != CCM_CSCMR1_PERCLK_CLK_SEL_IPG_CLK_ROOT
#    error The step timer expects PERCLK from IPG_CLK_ROOT
#  endif
#  if (BOARD_CPU_FREQUENCY / IMXRT_AHB_PODF_DIVIDER / \
       IMXRT_IPG_PODF_DIVIDER) % 1000000 != 0
#    error The step timer expects IPG_CLK_ROOT in whole MHz
#  endif
#  ifndef CONFIG_ARCH_HIPRI_INTERRUPT
#    error CONFIG_ARCH_HIPRI_INTERRUPT is required
#  endif
#  ifndef CONFIG_ARCH_RAMVECTORS
#    error CONFIG_ARCH_RAMVECTORS is required
#  endif
#  ifndef CONFIG_ARCH_IRQPRIO
#    error CONFIG_ARCH_IRQPRIO is required
#  endif
#  ifndef CONFIG_ARCH_RAMFUNCS
#    error CONFIG_ARCH_RAMFUNCS is required (the board runs from QSPI)
#  endif

#  define SG_RAMFUNC        locate_code(".ramfunc") noinline_function
#  define SG_BARRIER()      UP_DMB()

/* Registers */

#  define SG_GPIO1_SET      (IMXRT_GPIO1_BASE + IMXRT_GPIO_SET_OFFSET)
#  define SG_GPIO1_CLR      (IMXRT_GPIO1_BASE + IMXRT_GPIO_CLEAR_OFFSET)
#  define SG_GPIO3_PSR      (IMXRT_GPIO3_BASE + IMXRT_GPIO_PSR_OFFSET)
#  define SG_GPIO4_PSR      (IMXRT_GPIO4_BASE + IMXRT_GPIO_PSR_OFFSET)
#  define SG_GPIO4_SET      (IMXRT_GPIO4_BASE + IMXRT_GPIO_SET_OFFSET)
#  define SG_GPIO4_CLR      (IMXRT_GPIO4_BASE + IMXRT_GPIO_CLEAR_OFFSET)
#  define SG_GPT_CR         (IMXRT_GPT2_BASE + IMXRT_GPT_CR_OFFSET)
#  define SG_GPT_PR         (IMXRT_GPT2_BASE + IMXRT_GPT_PR_OFFSET)
#  define SG_GPT_SR         (IMXRT_GPT2_BASE + IMXRT_GPT_SR_OFFSET)
#  define SG_GPT_IR         (IMXRT_GPT2_BASE + IMXRT_GPT_IR_OFFSET)
#  define SG_GPT_OCR1       (IMXRT_GPT2_BASE + IMXRT_GPT_OCR1_OFFSET)
#  define SG_GPT_CNT        (IMXRT_GPT2_BASE + IMXRT_GPT_CNT_OFFSET)
#  define SG_GPT_SR_OF1     GPT_SR_OF1
#  define SG_PORT_GPIO3     GPIO3
#  define SG_PORT_GPIO4     GPIO4
#endif

/* GPT2 counts PERCLK_CLK_ROOT = IPG_CLK_ROOT / PERCLK_PODF (528 MHz / 4 / 9
 * = 14.667 MHz on this board).  One interrupt period, period / 10 MHz, is
 * SG_CNT_NUM * period / SG_CNT_DEN counts (132 * period / 90); the
 * fraction is carried over, so the average rate is exact.
 *
 * Note: the GPT "24 MHz crystal" clock source (CLKSRC 5, EN_24M,
 * PRESCALER24M = 1) was measured on the EC300 at 2.667 MHz instead of the
 * expected 12 MHz, so it is not used.
 */

#define SG_IPG_MHZ          (BOARD_CPU_FREQUENCY / IMXRT_AHB_PODF_DIVIDER / \
                             IMXRT_IPG_PODF_DIVIDER / 1000000u)
#define SG_CNT_NUM          SG_IPG_MHZ                         /* 132 */
#define SG_CNT_DEN          (IMXRT_PERCLK_PODF_DIVIDER * \
                             (STEPGEN_BASE_CLOCK / 1000000u))  /* 90 */

/* Spindle index dead time, step ticks */

#define SG_INDEX_DEADTIME   101

/* Board options */

#ifndef CONFIG_DDCNC_EC300_STEPGEN_DIR_INVERT
#  define CONFIG_DDCNC_EC300_STEPGEN_DIR_INVERT 0
#endif

#ifndef CONFIG_DDCNC_EC300_STEPGEN_INPUT_INVERT
#  define CONFIG_DDCNC_EC300_STEPGEN_INPUT_INVERT 0
#endif

#ifdef CONFIG_DDCNC_EC300_STEPGEN_STEP_INVERT
#  define SG_STEP_INVERT    1
#else
#  define SG_STEP_INVERT    0
#endif

#ifdef CONFIG_DDCNC_EC300_STEPGEN_OUTPUT_ACTIVE_LOW
#  define SG_OUT_ACTIVE_LOW 1
#else
#  define SG_OUT_ACTIVE_LOW 0
#endif

#define SG_NINPUTS          (sizeof(g_inmap) / sizeof(g_inmap[0]))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Mailbox commands, executed by the interrupt */

enum sg_op_e
{
  SG_OP_NONE = 0,
  SG_OP_RUN,
  SG_OP_FLUSH,
  SG_OP_CLEAR,
  SG_OP_SETPOS,
  SG_OP_OUTPUTS,
  SG_OP_CAPTURE,
  SG_OP_GETCAPT,
  SG_OP_ESTOP,
  SG_OP_FEEDSYNC,
  SG_OP_OVERRUN,
  SG_OP_TIMER
};

struct sg_axis_s
{
  int32_t err;
  int32_t inc;
  int32_t step;                 /* +1 or -1 */
};

struct sg_mbox_s
{
  volatile uint32_t op;         /* enum sg_op_e, cleared by the interrupt */
  int32_t  ret;
  uint32_t a;
  uint32_t b;
  uint32_t c;
  int32_t  pos[STEPGEN_NAXES];
};

/* An input of the board and its bit in the input word */

struct sg_inmap_s
{
  uint8_t port;                 /* GPIO3 or GPIO4 */
  uint8_t pin;
  uint8_t bit;
};

struct sg_state_s
{
  /* Setpoint ring: wr is written by the producer only, rd by the
   * interrupt only.
   */

  struct stepgen_seg_s fifo[STEPGEN_FIFO_SLOTS];
  volatile uint32_t wr;
  volatile uint32_t rd;

  /* Current setpoint */

  struct sg_axis_s ax[STEPGEN_NAXES];
  int32_t  dec;
  uint32_t remain;              /* ticks left in the setpoint */
  uint32_t tag;
  uint16_t last;                /* "last" mark of the latest setpoint */

  /* Step output */

  uint32_t stepmask;            /* GPIO1 bits to pulse at the next rise */
  bool     phase;               /* true: the next interrupt is a rise */
  bool     step_high;           /* step pulses are high at the pins */
  uint8_t  polarity;            /* cmd 0x0a flag                      */

  /* Motion state */

  bool     running;
  bool     fs_armed;            /* feed sync armed                    */
  bool     fs_wait;             /* start waits for the spindle index  */
  bool     estop;
  uint8_t  err;                 /* STEPGEN_F_UNDERRUN | _OVERRUN      */
  uint8_t  cause;
  int32_t  pos[STEPGEN_NAXES];
  uint32_t ticks;

  /* Capture */

  bool     capt_armed;
  bool     capt_done;
  uint8_t  capt_level;
  uint8_t  capt_stop;
  uint32_t capt_mask;
  int32_t  capt_pos[STEPGEN_NAXES];

  /* E-stop mode */

  uint32_t trip_hi;
  uint32_t trip_lo;

  /* I/O */

  uint32_t inputs;
  uint32_t outputs;
  uint8_t  hwstate[2];
  int32_t  hw[2];
  int8_t   quad[16];            /* quadrature decoder table           */
  bool     idx_prev;            /* index input was active             */
  uint32_t idx_age;             /* ticks since the last index         */
  uint32_t sp_cnt;
  uint32_t sp_period;

  /* Timer */

  uint32_t ocr;
  uint32_t inc_q;
  uint32_t inc_r;
  uint32_t inc_acc;

  /* Thread interface */

  struct sg_mbox_s mb;
  volatile uint32_t seq;        /* odd while pub is being written     */
  struct stepgen_status_s pub;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef STEPGEN_HOST_TEST
void stepgen_isr(void);
#else
static void stepgen_isr(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Board inputs and their input word bits.  The assignment of the EC300
 * inputs to functions follows the usual wiring of the Mach3 firmware of the
 * EC300 where it has a fixed function (MPG, spindle index) and is otherwise
 * arbitrary: change it here to suit the machine.
 */

static const struct sg_inmap_s g_inmap[] =
{
  { EC300_IN01,   STEPGEN_IN_ESTOP1    },
  { EC300_IN02,   STEPGEN_IN_PROBE     },
  { EC300_IN03,   STEPGEN_IN_HOME(0)   },
  { EC300_IN04,   STEPGEN_IN_HOME(1)   },
  { EC300_IN05,   STEPGEN_IN_HOME(2)   },
  { EC300_IN06,   STEPGEN_IN_HOME(3)   },
  { EC300_IN07,   STEPGEN_IN_HOME(4)   },
  { EC300_IN08,   STEPGEN_IN_HOME(5)   },
  { EC300_IN09,   STEPGEN_IN_RUN       },
  { EC300_IN10,   STEPGEN_IN_PAUSE     },
  { EC300_IN11,   STEPGEN_IN_EXTERR    },
  { EC300_IN12,   STEPGEN_IN_DRIVEERR  },
  { EC300_INDEX,  STEPGEN_IN_SYNC      },
  { EC300_WHA,    STEPGEN_IN_HW1A      },
  { EC300_WHB,    STEPGEN_IN_HW1B      },
  { EC300_FHA,    STEPGEN_IN_HW2A      },
  { EC300_FHB,    STEPGEN_IN_HW2B      },
  { EC300_XIN,    STEPGEN_IN_AUX(0)    },
  { EC300_YIN,    STEPGEN_IN_AUX(1)    },
  { EC300_ZIN,    STEPGEN_IN_AUX(2)    },
  { EC300_AIN,    STEPGEN_IN_AUX(3)    },
  { EC300_CIN,    STEPGEN_IN_AUX(4)    },
  { EC300_X10IN,  STEPGEN_IN_AUX(5)    },
#ifdef CONFIG_DDCNC_EC300_STEPGEN_LAYOUT_V1
  { EC300_EP,     STEPGEN_IN_ESTOP2    },
#else
  { EC300_X100IN, STEPGEN_IN_AUX(6)    },
  { EC300_SRO,    STEPGEN_IN_AUX(7)    },
  { EC300_EP,     STEPGEN_IN_AUX(8)    },
  { EC300_SRJ,    STEPGEN_IN_AUX(9)    },
#endif
};

static const uint8_t g_step_pins[STEPGEN_NAXES] = EC300_STEP_PINS;
static const uint8_t g_dir_pins[STEPGEN_NAXES]  = EC300_DIR_PINS;
static const uint8_t g_out_pins[EC300_NOUTPUTS] = EC300_OUT_PINS;

/* Everything the interrupt reads is in RAM (.data/.bss), not in flash */

static struct sg_state_s g_sg;

static uint32_t g_step_bit[STEPGEN_NAXES];
static uint32_t g_dir_bit[STEPGEN_NAXES];
static uint32_t g_step_all;
static uint32_t g_dir_inv;               /* GPIO1 bits of inverted dirs  */
static uint32_t g_out_bit[EC300_NOUTPUTS];
static uint32_t g_out_all;
static uint8_t  g_in_src[SG_NINPUTS];    /* bit 5: GPIO4, bits 0-4: pin  */
static uint8_t  g_in_bit[SG_NINPUTS];
static uint32_t g_in_base;               /* levels of unmapped inputs    */

/* Quadrature decoder, index (previous << 2) | (B << 1) | A.
 * Copied to g_sg.quad (RAM) for the interrupt.
 */

static const int8_t g_quad[16] =
{
  0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0
};

#ifndef STEPGEN_HOST_TEST
static mutex_t g_sg_lock = NXMUTEX_INITIALIZER;
#endif

/****************************************************************************
 * Private Functions: interrupt level
 ****************************************************************************/

/****************************************************************************
 * Name: sg_apply_outputs
 *
 * Description:
 *   Drive the output pins from the output word.
 *
 ****************************************************************************/

static SG_RAMFUNC void sg_apply_outputs(uint32_t word)
{
  uint32_t on = 0;
  int i;

  for (i = 0; i < EC300_NOUTPUTS; i++)
    {
      if (word & (1u << (STEPGEN_OUT_MACHON + i)))
        {
          on |= g_out_bit[i];
        }
    }

#if SG_OUT_ACTIVE_LOW
  putreg32(on, SG_GPIO4_CLR);
  putreg32(g_out_all & ~on, SG_GPIO4_SET);
#else
  putreg32(on, SG_GPIO4_SET);
  putreg32(g_out_all & ~on, SG_GPIO4_CLR);
#endif
}

/****************************************************************************
 * Name: sg_idle_steps
 *
 * Description:
 *   Put all the step outputs at their idle level.
 *
 ****************************************************************************/

static SG_RAMFUNC void sg_idle_steps(void)
{
  putreg32(g_step_all, g_sg.step_high ? SG_GPIO1_CLR : SG_GPIO1_SET);
}

/****************************************************************************
 * Name: sg_tripped
 *
 * Description:
 *   The input bits at their E-stop trip level.
 *
 ****************************************************************************/

static SG_RAMFUNC uint32_t sg_tripped(void)
{
  return (g_sg.inputs & g_sg.trip_hi) | (~g_sg.inputs & g_sg.trip_lo);
}

/****************************************************************************
 * Name: sg_set_timer
 *
 * Description:
 *   Set the interrupt period and the step polarity.
 *
 ****************************************************************************/

static SG_RAMFUNC void sg_set_timer(uint32_t period, uint32_t polarity)
{
  uint32_t n = SG_CNT_NUM * period;

  g_sg.inc_q    = n / SG_CNT_DEN;
  g_sg.inc_r    = n % SG_CNT_DEN;
  g_sg.inc_acc  = 0;
  g_sg.polarity = polarity != 0;
  g_sg.step_high = (g_sg.polarity != 0) ^ SG_STEP_INVERT;
  sg_idle_steps();

  /* Restart the spindle measurement too */

  g_sg.idx_age   = 0;
  g_sg.sp_cnt    = 0;
  g_sg.sp_period = 0;
}

/****************************************************************************
 * Name: sg_mailbox
 *
 * Description:
 *   Execute a command posted by the thread interface.
 *
 ****************************************************************************/

static SG_RAMFUNC void sg_mailbox(uint32_t op)
{
  struct sg_mbox_s *mb = &g_sg.mb;
  int i;

  mb->ret = 0;

  switch (op)
    {
      case SG_OP_RUN:
        if (g_sg.estop)
          {
            mb->ret = -EPERM;
          }
        else if (mb->a)
          {
            if (g_sg.fs_armed && !g_sg.running)
              {
                g_sg.fs_wait = true;
              }
            else
              {
                g_sg.running = true;
              }
          }
        else
          {
            g_sg.running = false;
            g_sg.fs_wait = false;
          }
        break;

      case SG_OP_FLUSH:
        g_sg.rd         = g_sg.wr;
        g_sg.remain     = 0;
        g_sg.last       = 1;
        g_sg.tag        = 0;
        g_sg.stepmask   = 0;
        g_sg.running    = false;
        g_sg.fs_armed   = false;
        g_sg.fs_wait    = false;
        g_sg.err        = 0;
        g_sg.capt_armed = false;
        g_sg.capt_done  = false;
        g_sg.sp_cnt     = 0;
        for (i = 0; i < STEPGEN_NAXES; i++)
          {
            g_sg.capt_pos[i] = 0;
          }
        break;

      case SG_OP_CLEAR:
        g_sg.err      = 0;
        g_sg.running  = false;
        g_sg.fs_armed = false;
        g_sg.fs_wait  = false;
        if (sg_tripped() == 0)
          {
            g_sg.estop = false;
            g_sg.cause = 0;
          }
        break;

      case SG_OP_SETPOS:
        for (i = 0; i < STEPGEN_NAXES; i++)
          {
            g_sg.pos[i] = mb->pos[i];
          }
        break;

      case SG_OP_OUTPUTS:
        if (mb->a)
          {
            /* Nothing is switched on while the E-stop is latched */

            if (!g_sg.estop)
              {
                g_sg.outputs |= mb->b;
              }
          }
        else
          {
            g_sg.outputs &= ~mb->b;
          }

        sg_apply_outputs(g_sg.outputs);
        break;

      case SG_OP_CAPTURE:
        g_sg.capt_mask  = mb->a;
        g_sg.capt_level = (uint8_t)mb->b;
        g_sg.capt_stop  = (uint8_t)mb->c;
        g_sg.capt_armed = mb->a != 0;
        g_sg.capt_done  = false;
        break;

      case SG_OP_GETCAPT:
        for (i = 0; i < STEPGEN_NAXES; i++)
          {
            mb->pos[i] = g_sg.capt_pos[i];
          }

        mb->ret = g_sg.capt_done;
        g_sg.capt_done = false;
        break;

      case SG_OP_ESTOP:
        g_sg.trip_hi = mb->a;
        g_sg.trip_lo = mb->b;
        if (sg_tripped() == 0)
          {
            g_sg.estop = false;
            g_sg.cause = 0;
          }
        break;

      case SG_OP_FEEDSYNC:
        g_sg.fs_armed = mb->a != 0;
        if (!g_sg.fs_armed)
          {
            g_sg.fs_wait = false;
          }
        break;

      case SG_OP_OVERRUN:
        g_sg.err    |= STEPGEN_F_OVERRUN;
        g_sg.running = false;
        g_sg.fs_wait = false;
        break;

      case SG_OP_TIMER:
        sg_set_timer(mb->a, mb->b);
        break;

      default:
        mb->ret = -EINVAL;
        break;
    }
}

/****************************************************************************
 * Name: sg_read_inputs
 *
 * Description:
 *   Build the input word from the input pins.
 *
 ****************************************************************************/

static SG_RAMFUNC uint32_t sg_read_inputs(void)
{
  uint32_t p3 = getreg32(SG_GPIO3_PSR);
  uint32_t p4 = getreg32(SG_GPIO4_PSR);
  uint32_t in = g_in_base;
  unsigned int i;

  for (i = 0; i < SG_NINPUTS; i++)
    {
      uint32_t src = g_in_src[i];
      uint32_t lvl = ((src & 0x20) ? p4 : p3) >> (src & 0x1f);

      in |= (lvl & 1u) << g_in_bit[i];
    }

  return in ^ (uint32_t)CONFIG_DDCNC_EC300_STEPGEN_INPUT_INVERT;
}

/****************************************************************************
 * Name: sg_load
 *
 * Description:
 *   Start the next setpoint of the FIFO: set up the DDA and the direction
 *   outputs.
 *
 ****************************************************************************/

static SG_RAMFUNC void sg_load(void)
{
  uint32_t rd = g_sg.rd;
  const struct stepgen_seg_s *seg = &g_sg.fifo[rd];
  uint32_t dset = 0;
  uint32_t dclr = 0;
  int32_t ticks = seg->ticks;
  int32_t a[STEPGEN_NAXES];
  int i;

  /* Never lose steps: a setpoint with more steps than ticks (or no ticks)
   * is stretched to one step per tick.
   */

  for (i = 0; i < STEPGEN_NAXES; i++)
    {
      int32_t d = seg->delta[i];

      if (d < 0)
        {
          a[i] = -d;
          g_sg.ax[i].step = -1;
          dclr |= g_dir_bit[i];
        }
      else
        {
          a[i] = d;
          g_sg.ax[i].step = 1;
          dset |= g_dir_bit[i];
        }

      if (a[i] > ticks)
        {
          ticks = a[i];
        }
    }

  for (i = 0; i < STEPGEN_NAXES; i++)
    {
      g_sg.ax[i].inc = 2 * a[i];
      g_sg.ax[i].err = a[i] - ticks;
    }

  g_sg.dec    = 2 * ticks;
  g_sg.remain = (uint32_t)ticks;
  g_sg.tag    = seg->tag;
  g_sg.last   = seg->last;

  /* Direction outputs: high = positive, unless inverted */

  putreg32((dset & ~g_dir_inv) | (dclr & g_dir_inv), SG_GPIO1_SET);
  putreg32((dclr & ~g_dir_inv) | (dset & g_dir_inv), SG_GPIO1_CLR);

  SG_BARRIER();
  g_sg.rd = (rd + 1 >= STEPGEN_FIFO_SLOTS) ? 0 : rd + 1;
}

/****************************************************************************
 * Name: sg_publish
 *
 * Description:
 *   Publish the status for the thread interface.
 *
 ****************************************************************************/

static SG_RAMFUNC void sg_publish(void)
{
  struct stepgen_status_s *p = &g_sg.pub;
  uint32_t used;
  uint8_t flags;
  int i;

  used = (g_sg.wr + STEPGEN_FIFO_SLOTS - g_sg.rd) % STEPGEN_FIFO_SLOTS;

  flags = g_sg.err;
  if (g_sg.running)
    {
      flags |= STEPGEN_F_RUNNING;
    }

  if (g_sg.capt_armed)
    {
      flags |= STEPGEN_F_CAPT_ARMED;
    }

  if (g_sg.capt_done)
    {
      flags |= STEPGEN_F_CAPT_DONE;
    }

  if (g_sg.estop)
    {
      flags |= STEPGEN_F_ESTOP;
    }

  g_sg.seq++;
  SG_BARRIER();

  p->tag       = g_sg.tag;
  p->fifo_free = used >= STEPGEN_FIFO_USABLE ? 0 :
                 STEPGEN_FIFO_USABLE - used;
  p->flags     = flags;
  p->cause     = g_sg.cause;
  for (i = 0; i < STEPGEN_NAXES; i++)
    {
      p->pos[i] = g_sg.pos[i];
    }

  p->hw[0]          = g_sg.hw[0];
  p->hw[1]          = g_sg.hw[1];
  p->spindle_cnt    = g_sg.sp_cnt;
  p->spindle_period = g_sg.sp_period;
  p->outputs        = g_sg.outputs;
  p->inputs         = g_sg.inputs;
  p->ticks          = g_sg.ticks;

  SG_BARRIER();
  g_sg.seq++;
}

/****************************************************************************
 * Name: sg_tick
 *
 * Description:
 *   One step tick: inputs, E-stop, capture, handwheels, spindle index,
 *   mailbox, then the DDA, which leaves the step outputs of the next rise
 *   in g_sg.stepmask.
 *
 ****************************************************************************/

static SG_RAMFUNC void sg_tick(void)
{
  uint32_t in;
  uint32_t act;
  uint32_t op;
  uint32_t mask;
  bool idx;
  int i;

  g_sg.ticks++;

  /* Inputs; act has a 1 for every active input */

  in = sg_read_inputs();
  g_sg.inputs = in;
  act = in ^ STEPGEN_IN_ACTIVE_LOW_MASK;

  /* Handwheels */

  i = (g_sg.hwstate[0] << 2) | (((in >> STEPGEN_IN_HW1B) & 1) << 1) |
      ((in >> STEPGEN_IN_HW1A) & 1);
  g_sg.hw[0] += g_sg.quad[i];
  g_sg.hwstate[0] = i & 3;

  i = (g_sg.hwstate[1] << 2) | (((in >> STEPGEN_IN_HW2B) & 1) << 1) |
      ((in >> STEPGEN_IN_HW2A) & 1);
  g_sg.hw[1] += g_sg.quad[i];
  g_sg.hwstate[1] = i & 3;

  /* Spindle index: count the edges that make it active, at least
   * SG_INDEX_DEADTIME ticks apart; the period is the ticks between them.
   */

  idx = (act >> STEPGEN_IN_SYNC) & 1;
  if (g_sg.idx_age != UINT32_MAX)
    {
      g_sg.idx_age++;
    }

  if (idx && !g_sg.idx_prev && g_sg.idx_age >= SG_INDEX_DEADTIME)
    {
      g_sg.sp_period = g_sg.idx_age;
      g_sg.idx_age   = 0;
      g_sg.sp_cnt++;
      if (g_sg.fs_wait)
        {
          g_sg.fs_wait = false;
          g_sg.running = true;
        }
    }
  else if (g_sg.sp_period != 0 && g_sg.idx_age / 3 > g_sg.sp_period)
    {
      g_sg.sp_period = 0;                    /* spindle stopped */
    }

  g_sg.idx_prev = idx;

  /* Commands from the thread interface */

  op = g_sg.mb.op;
  if (op != SG_OP_NONE)
    {
      sg_mailbox(op);
      SG_BARRIER();
      g_sg.mb.op = SG_OP_NONE;
    }

  /* E-stop: stop, all outputs off, latched until cleared */

  mask = sg_tripped();
  if (mask != 0 && !g_sg.estop)
    {
      g_sg.estop   = true;
      g_sg.cause   = (uint8_t)(__builtin_ctz(mask) + 1);
      g_sg.running = false;
      g_sg.fs_wait = false;
      g_sg.outputs = 0;
      sg_apply_outputs(0);
    }

  /* Capture */

  if (g_sg.capt_armed &&
      ((act & g_sg.capt_mask) != 0) == (g_sg.capt_level != 0))
    {
      for (i = 0; i < STEPGEN_NAXES; i++)
        {
          g_sg.capt_pos[i] = g_sg.pos[i];
        }

      g_sg.capt_armed = false;
      g_sg.capt_done  = true;
      if (g_sg.capt_stop)
        {
          g_sg.running = false;
        }
    }

  /* Motion */

  mask = 0;
  if (g_sg.running)
    {
      if (g_sg.remain == 0)
        {
          if (g_sg.rd != g_sg.wr)
            {
              sg_load();
            }
          else if (!g_sg.last)
            {
              g_sg.err    |= STEPGEN_F_UNDERRUN;
              g_sg.running = false;
            }
        }

      if (g_sg.running && g_sg.remain != 0)
        {
          for (i = 0; i < STEPGEN_NAXES; i++)
            {
              struct sg_axis_s *ax = &g_sg.ax[i];

              if (ax->err >= 0)
                {
                  mask       |= g_step_bit[i];
                  g_sg.pos[i] += ax->step;
                  ax->err    -= g_sg.dec;
                }

              ax->err += ax->inc;
            }

          g_sg.remain--;
        }
    }

  g_sg.stepmask = mask;
  sg_publish();
}

/****************************************************************************
 * Name: stepgen_isr
 *
 * Description:
 *   GPT2 compare interrupt, zero latency.  Schedules the next compare and
 *   runs a rise or a tick phase.
 *
 ****************************************************************************/

#ifndef STEPGEN_HOST_TEST
static
#endif
SG_RAMFUNC void stepgen_isr(void)
{
  uint32_t inc;

  putreg32(SG_GPT_SR_OF1, SG_GPT_SR);

  inc = g_sg.inc_q;
  g_sg.inc_acc += g_sg.inc_r;
  if (g_sg.inc_acc >= SG_CNT_DEN)
    {
      g_sg.inc_acc -= SG_CNT_DEN;
      inc++;
    }

  g_sg.ocr += inc;
  putreg32(g_sg.ocr, SG_GPT_OCR1);

  /* Never let the compare fall behind the counter (it would wait for the
   * counter to wrap: 6 minutes).
   */

  if ((int32_t)(g_sg.ocr - getreg32(SG_GPT_CNT)) <= 0)
    {
      g_sg.ocr = getreg32(SG_GPT_CNT) + inc;
      putreg32(g_sg.ocr, SG_GPT_OCR1);
    }

  if (g_sg.phase)
    {
      if (g_sg.stepmask != 0)
        {
          putreg32(g_sg.stepmask,
                   g_sg.step_high ? SG_GPIO1_SET : SG_GPIO1_CLR);
        }

      g_sg.phase = false;
    }
  else
    {
      sg_idle_steps();
      sg_tick();
      g_sg.phase = true;
    }

  /* Make sure that the flag is clear before the exception returns */

  getreg32(SG_GPT_SR);
}

/****************************************************************************
 * Private Functions: thread level
 ****************************************************************************/

/****************************************************************************
 * Name: sg_post
 *
 * Description:
 *   Post a command to the interrupt and wait for it to be executed (at
 *   most two interrupt periods).
 *
 ****************************************************************************/

static int sg_post(uint32_t op, uint32_t a, uint32_t b, uint32_t c,
                   FAR const int32_t *pos, FAR int32_t *posout)
{
  struct sg_mbox_s *mb = &g_sg.mb;
  int ret;
  int i;

#ifndef STEPGEN_HOST_TEST
  nxmutex_lock(&g_sg_lock);
#endif

  mb->a = a;
  mb->b = b;
  mb->c = c;
  if (pos != NULL)
    {
      for (i = 0; i < STEPGEN_NAXES; i++)
        {
          mb->pos[i] = pos[i];
        }
    }

  SG_BARRIER();
  mb->op = op;

#ifdef STEPGEN_HOST_TEST
  while (mb->op != SG_OP_NONE)
    {
      stepgen_isr();
    }
#else
  /* The slowest interrupt rate (period 65535) is 152 Hz */

  for (i = 0; i < 50000 && mb->op != SG_OP_NONE; i++)
    {
      up_udelay(1);
    }
#endif

  if (mb->op != SG_OP_NONE)
    {
      mb->op = SG_OP_NONE;
      ret = -ETIMEDOUT;
    }
  else
    {
      SG_BARRIER();
      ret = mb->ret;
      if (posout != NULL)
        {
          for (i = 0; i < STEPGEN_NAXES; i++)
            {
              posout[i] = mb->pos[i];
            }
        }
    }

#ifndef STEPGEN_HOST_TEST
  nxmutex_unlock(&g_sg_lock);
#endif
  return ret;
}

/****************************************************************************
 * Name: sg_pendsv
 *
 * Description:
 *   With CONFIG_ARCH_HIPRI_INTERRUPT, arm_doirq() hands PendSV to
 *   irq_dispatch() (it is the hook a high priority interrupt can use to get
 *   back into the OS).  But the OS also pends PendSV itself, for example to
 *   deliver a signal to the running task (up_schedule_sigaction()).  With
 *   nothing attached that ends in irq_unexpected_isr() and a kernel panic:
 *   on this board the first PHY notification signal to the netinit thread
 *   halted the system.  This empty handler lets arm_doirq() go on with the
 *   normal PendSV processing.
 *
 ****************************************************************************/

#ifndef STEPGEN_HOST_TEST
static int sg_pendsv(int irq, FAR void *context, FAR void *arg)
{
  return OK;
}
#endif

/****************************************************************************
 * Name: sg_tables
 *
 * Description:
 *   Build the pin tables used by the interrupt.
 *
 ****************************************************************************/

static void sg_tables(void)
{
  unsigned int i;

  g_step_all = 0;
  g_dir_inv  = 0;
  for (i = 0; i < STEPGEN_NAXES; i++)
    {
      g_step_bit[i] = 1u << g_step_pins[i];
      g_dir_bit[i]  = 1u << g_dir_pins[i];
      g_step_all   |= g_step_bit[i];
      if (CONFIG_DDCNC_EC300_STEPGEN_DIR_INVERT & (1 << i))
        {
          g_dir_inv |= g_dir_bit[i];
        }
    }

  g_out_all = 0;
  for (i = 0; i < EC300_NOUTPUTS; i++)
    {
      g_out_bit[i] = 1u << g_out_pins[i];
      g_out_all   |= g_out_bit[i];
    }

  /* Unmapped inputs read inactive: 1 for the active low bits */

  g_in_base = STEPGEN_IN_ACTIVE_LOW_MASK;
  for (i = 0; i < SG_NINPUTS; i++)
    {
      g_in_src[i] = (g_inmap[i].port == SG_PORT_GPIO4 ? 0x20 : 0) |
                    g_inmap[i].pin;
      g_in_bit[i] = g_inmap[i].bit;
      g_in_base  &= ~(1u << g_inmap[i].bit);
    }
}

/****************************************************************************
 * Name: sg_state_init
 *
 * Description:
 *   Reset state: stopped, empty FIFO, nothing monitored, default timer.
 *
 ****************************************************************************/

static void sg_state_init(void)
{
  memset(&g_sg, 0, sizeof(g_sg));
  memcpy(g_sg.quad, g_quad, sizeof(g_sg.quad));
  g_sg.last    = 1;
  g_sg.trip_hi = 0;
  g_sg.trip_lo = 0;
  sg_set_timer(STEPGEN_DEF_PERIOD, 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef STEPGEN_HOST_TEST

/****************************************************************************
 * Name: imxrt_stepgen_initialize
 ****************************************************************************/

int imxrt_stepgen_initialize(void)
{
  unsigned int i;
  int ret;

  sg_tables();
  sg_state_init();

  /* Pins */

  imxrt_clockall_gpio1();
  imxrt_clockall_gpio3();
  imxrt_clockall_gpio4();

  for (i = 0; i < STEPGEN_NAXES; i++)
    {
      imxrt_config_gpio(GPIO_OUTPUT | IOMUX_STEP_DEFAULT |
                        (g_sg.step_high ? GPIO_OUTPUT_ZERO :
                                          GPIO_OUTPUT_ONE) |
                        GPIO_PORT1 | (g_step_pins[i] << GPIO_PIN_SHIFT));
      imxrt_config_gpio(GPIO_OUTPUT | IOMUX_STEP_DEFAULT | GPIO_OUTPUT_ZERO |
                        GPIO_PORT1 | (g_dir_pins[i] << GPIO_PIN_SHIFT));
    }

  for (i = 0; i < EC300_NOUTPUTS; i++)
    {
      imxrt_config_gpio(GPIO_OUTPUT | IOMUX_OUT_DEFAULT |
                        (SG_OUT_ACTIVE_LOW ? GPIO_OUTPUT_ONE :
                                             GPIO_OUTPUT_ZERO) |
                        GPIO_PORT4 | (g_out_pins[i] << GPIO_PIN_SHIFT));
    }

  for (i = 0; i < SG_NINPUTS; i++)
    {
      imxrt_config_gpio(GPIO_INPUT | IOMUX_IN_DEFAULT |
                        ((uint32_t)g_inmap[i].port << GPIO_PORT_SHIFT) |
                        ((uint32_t)g_inmap[i].pin << GPIO_PIN_SHIFT));
    }

  g_sg.inputs = sg_read_inputs();
  g_sg.idx_prev = true;           /* no index count at start-up */

  /* GPT2: free running from PERCLK, compare 1 interrupt */

  imxrt_clockall_gpt2_bus();
  imxrt_clockall_gpt2_serial();

  putreg32(0, SG_GPT_CR);
  putreg32(0, SG_GPT_IR);
  putreg32(GPT_CR_SWR, SG_GPT_CR);
  while (getreg32(SG_GPT_CR) & GPT_CR_SWR)
    {
    }

  putreg32(GPT_PR_PRESCALER(0), SG_GPT_PR);
  putreg32(GPT_CR_CLKSRC_IPG | GPT_CR_FRR | GPT_CR_ENMOD |
           GPT_CR_WAITEN | GPT_CR_DBGEN, SG_GPT_CR);
  putreg32(0x3f, SG_GPT_SR);

  /* Keep the OS working with PendSV dispatched (see sg_pendsv()) */

  ret = irq_attach(NVIC_IRQ_PENDSV, sg_pendsv, NULL);
  if (ret < 0)
    {
      _err("ERROR: irq_attach(PendSV) failed: %d\n", ret);
      return ret;
    }

  /* Zero latency interrupt: in the RAM vector table, above the priority
   * that the OS masks.
   */

  ret = arm_ramvec_attach(IMXRT_IRQ_GPT2, stepgen_isr);
  if (ret < 0)
    {
      _err("ERROR: arm_ramvec_attach failed: %d\n", ret);
      return ret;
    }

  ret = up_prioritize_irq(IMXRT_IRQ_GPT2, NVIC_SYSH_HIGH_PRIORITY);
  if (ret < 0)
    {
      _err("ERROR: up_prioritize_irq failed: %d\n", ret);
      return ret;
    }

  g_sg.ocr = g_sg.inc_q;
  putreg32(g_sg.ocr, SG_GPT_OCR1);
  putreg32(GPT_IR_OF1IE, SG_GPT_IR);
  modifyreg32(SG_GPT_CR, 0, GPT_CR_EN);
  up_enable_irq(IMXRT_IRQ_GPT2);

  _info("step generator: %u Hz interrupt, zero latency\n",
        STEPGEN_BASE_CLOCK / STEPGEN_DEF_PERIOD);
  return OK;
}

#endif /* !STEPGEN_HOST_TEST */

/****************************************************************************
 * Name: stepgen_push
 ****************************************************************************/

int stepgen_push(FAR const struct stepgen_seg_s *seg, int count)
{
  uint32_t wr = g_sg.wr;
  uint32_t used;
  int i;

  used = (wr + STEPGEN_FIFO_SLOTS - g_sg.rd) % STEPGEN_FIFO_SLOTS;
  if (count < 0 || used + (uint32_t)count > STEPGEN_FIFO_USABLE)
    {
      stepgen_overrun();
      return -ENOSPC;
    }

  for (i = 0; i < count; i++)
    {
      g_sg.fifo[wr] = seg[i];
      wr = (wr + 1 >= STEPGEN_FIFO_SLOTS) ? 0 : wr + 1;
    }

  SG_BARRIER();
  g_sg.wr = wr;
  return 0;
}

/****************************************************************************
 * Name: stepgen_overrun
 ****************************************************************************/

void stepgen_overrun(void)
{
  sg_post(SG_OP_OVERRUN, 0, 0, 0, NULL, NULL);
}

/****************************************************************************
 * Name: stepgen_fifo_free
 ****************************************************************************/

int stepgen_fifo_free(void)
{
  uint32_t used = (g_sg.wr + STEPGEN_FIFO_SLOTS - g_sg.rd) %
                  STEPGEN_FIFO_SLOTS;

  return used >= STEPGEN_FIFO_USABLE ? 0 : STEPGEN_FIFO_USABLE - used;
}

/****************************************************************************
 * Name: stepgen_get_status
 ****************************************************************************/

void stepgen_get_status(FAR struct stepgen_status_s *st)
{
  uint32_t seq;

  do
    {
      seq = g_sg.seq;
      SG_BARRIER();
      *st = g_sg.pub;
      SG_BARRIER();
    }
  while ((seq & 1) != 0 || seq != g_sg.seq);

  /* The producer side is more recent than the interrupt's snapshot */

  st->fifo_free = (uint16_t)stepgen_fifo_free();
}

/****************************************************************************
 * Name: stepgen_run_enable
 ****************************************************************************/

int stepgen_run_enable(bool run)
{
  return sg_post(SG_OP_RUN, run, 0, 0, NULL, NULL);
}

/****************************************************************************
 * Name: stepgen_flush
 ****************************************************************************/

void stepgen_flush(void)
{
  sg_post(SG_OP_FLUSH, 0, 0, 0, NULL, NULL);
}

/****************************************************************************
 * Name: stepgen_clear_error
 ****************************************************************************/

void stepgen_clear_error(void)
{
  sg_post(SG_OP_CLEAR, 0, 0, 0, NULL, NULL);
}

/****************************************************************************
 * Name: stepgen_set_position
 ****************************************************************************/

void stepgen_set_position(FAR const int32_t *pos)
{
  sg_post(SG_OP_SETPOS, 0, 0, 0, pos, NULL);
}

/****************************************************************************
 * Name: stepgen_set_outputs
 ****************************************************************************/

void stepgen_set_outputs(uint32_t on, uint32_t mask)
{
  sg_post(SG_OP_OUTPUTS, on != 0, mask, 0, NULL, NULL);
}

/****************************************************************************
 * Name: stepgen_set_capture
 ****************************************************************************/

void stepgen_set_capture(uint32_t mask, uint8_t level, uint8_t stop)
{
  sg_post(SG_OP_CAPTURE, mask, level, stop, NULL, NULL);
}

/****************************************************************************
 * Name: stepgen_get_capture
 ****************************************************************************/

uint32_t stepgen_get_capture(FAR int32_t *pos)
{
  int ret = sg_post(SG_OP_GETCAPT, 0, 0, 0, NULL, pos);

  return ret > 0 ? 1 : 0;
}

/****************************************************************************
 * Name: stepgen_set_estop_mode
 ****************************************************************************/

void stepgen_set_estop_mode(uint32_t trip_hi, uint32_t trip_lo)
{
  sg_post(SG_OP_ESTOP, trip_hi, trip_lo, 0, NULL, NULL);
}

/****************************************************************************
 * Name: stepgen_set_timer
 ****************************************************************************/

int stepgen_set_timer(uint16_t period, uint8_t polarity)
{
  if (period < STEPGEN_MIN_PERIOD)
    {
      return -EINVAL;
    }

  return sg_post(SG_OP_TIMER, period, polarity, 0, NULL, NULL);
}

/****************************************************************************
 * Name: stepgen_arm_feedsync
 ****************************************************************************/

void stepgen_arm_feedsync(bool arm)
{
  sg_post(SG_OP_FEEDSYNC, arm, 0, 0, NULL, NULL);
}

/****************************************************************************
 * Name: stepgen_set_pwm
 *
 * Description:
 *   The pin of the 0-10 V spindle output of the EC300 is not known yet.
 *
 ****************************************************************************/

int stepgen_set_pwm(int channel, uint32_t period, uint32_t duty)
{
  (void)channel;
  (void)period;
  (void)duty;
  return -ENOSYS;
}

#endif /* CONFIG_DDCNC_EC300_STEPGEN */
