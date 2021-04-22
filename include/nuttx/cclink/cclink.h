/****************************************************************************
 * include/nuttx/cclink/u-blox.h
 *
 *   Copyright (C) 2021 Md. Mahmududul Hasan. All rights reserved.
 *   Author: Md. Mahmudul Hasan Sumon <sumonipe08@gmail.com>
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

#ifndef __INCLUDE_NUTTX_CCLINK_FM60_80_H
#define __INCLUDE_NUTTX_CCLINK_FM60_80_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdbool.h>


#include <nuttx/cclink/ioctl.h>

#ifdef CONFIG_CCLINK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* See IOCTL definitions in include/nuttx/modem/ioctl.h */

/****************************************************************************
 * Public Types
 ****************************************************************************/
enum cclink_mx_error
{
  CCLINK_MX_LOW_VAC_OUTPUT,
  CCLINK_MX_STACKING_ERROR,
  CCLINK_MX_OVER_TEMP,
  CCLINK_MX_LOW_BATTERY,
  CCLINK_MX_PHASE_LOSS,
  CCLINK_MX_HIGH_BATTERY,
  CCLINK_MX_SHORTED_OUTPUT,
  CCLINK_MX_BACK_FEED
};

enum cclink_mx_state
{
  CCLINK_MX_STATE_SLEEPING,
  CCLINK_MX_STATE_FLOATING,
  CCLINK_MX_STATE_BULK,
  CCLINK_MX_STATE_ABSORB,
  CCLINK_MX_STATE_EQUALIZE
};

struct cclink_mx_status
{
  float in_voltage;
  float out_voltage;
  float kwh;
  bool error;
  enum cclink_mx_state state;
  uint8_t aux_mode;
  float ah;
  float out_amps_dc;
  float in_amps_dc;
};
enum cclink_mx_channel
{
  CCLINK_MX_CHANNEL_UNKNOWN,
  CCLINK_MX_CHANNEL_ONE,
  CCLINK_MX_CHANNEL_TWO,
  CCLINK_MX_CHANNEL_THREE,
};

struct cclink_mx_logpage
{

};

struct cclink_lower;

struct cclink_ops
{
  CODE int (*cclink_init)(FAR struct cclink_lower *lower);
  CODE int (*cclink_scan)(FAR struct cclink_lower *lower);
  CODE int (*cclink_get_logpage)(FAR struct cclink_lower *lower, 
                        FAR struct cclink_mx_logpage * log);
  CODE int (*cclink_get_status)(FAR struct cclink_lower *lower,
                        FAR struct cclink_mx_status *status);
  CODE int (*ioctl)(FAR struct cclink_lower *lower,
                    int cmd,
                    unsigned long arg);
};

struct cclink_lower
{
  enum cclink_mx_channel channel;
  FAR const struct cclink_ops *ops;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

  /****************************************************************************
 * "Upper-Half" SIM800l Driver Interfaces
 ****************************************************************************/

  FAR void *cclink_register(FAR const char *path,
                             FAR struct cclink_lower *lower);

  void cclink_unregister(FAR void *handle);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_MODEM_SIM800l*/

#endif /* __INCLUDE_NUTTX_MODEM_U_BLOX_H */
