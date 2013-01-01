/****************************************************************************
 * examples/pwm/pwm_main.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/pwm.h>

#include "pwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pwm_state_s
{
  bool      initialized;
  FAR char *devpath;
  uint8_t   duty;
  uint32_t  freq;
#ifdef CONFIG_PWM_PULSECOUNT
  uint32_t  count;
#endif
  int       duration;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct pwm_state_s g_pwmstate;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_devpath
 ****************************************************************************/

static void pwm_devpath(FAR struct pwm_state_s *pwm, FAR const char *devpath)
{
  /* Get rid of any old device path */

  if (pwm->devpath)
    {
      free(pwm->devpath);
    }

  /* Then set-up the new device path by copying the string */

  pwm->devpath = strdup(devpath);
}

/****************************************************************************
 * Name: pwm_help
 ****************************************************************************/

static void pwm_help(FAR struct pwm_state_s *pwm)
{
  message("Usage: pwm [OPTIONS]\n");
  message("\nArguments are \"sticky\".  For example, once the PWM frequency is\n");
  message("specified, that frequency will be re-used until it is changed.\n");
  message("\n\"sticky\" OPTIONS include:\n");
  message("  [-p devpath] selects the PWM device.  "
         "Default: %s Current: %s\n",
         CONFIG_EXAMPLES_PWM_DEVPATH, pwm->devpath ? pwm->devpath : "NONE");
  message("  [-f addr] selects the pulse frequency.  "
         "Default: %d Hz Current: %d Hz\n",
         CONFIG_EXAMPLES_PWM_FREQUENCY, pwm->freq);
  message("  [-d duty] selcts the pulse duty as a percentage.  "
          "Default: %d %% Current: %d %%\n",
          CONFIG_EXAMPLES_PWM_DUTYPCT, pwm->duty);
#ifdef CONFIG_PWM_PULSECOUNT
  message("  [-n count] selects the pulse count.  "
         "Default: %d Current: %d\n",
         CONFIG_EXAMPLES_PWM_COUNT, pwm->count);
#endif
  message("  [-t duration] is the duration of the pulse train in seconds.  "
          "Default: %d Current: %d\n",
         CONFIG_EXAMPLES_PWM_DURATION, pwm->duration);
  message("  [-h] shows this message and exits\n");
}

/****************************************************************************
 * Name: arg_string
 ****************************************************************************/

static int arg_string(FAR char **arg, FAR char **value)
{
  FAR char *ptr = *arg;

  if (ptr[2] == '\0')
    {
      *value = arg[1];
      return 2;
    }
  else
    {
      *value = &ptr[2];
      return 1;
    }
}

/****************************************************************************
 * Name: arg_decimal
 ****************************************************************************/

static int arg_decimal(FAR char **arg, FAR long *value)
{
  FAR char *string;
  int ret;

  ret = arg_string(arg, &string);
  *value = strtol(string, NULL, 10);
  return ret;
}

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

static void parse_args(FAR struct pwm_state_s *pwm, int argc, FAR char **argv)
{
  FAR char *ptr;
  FAR char *str;
  long value;
  int index;
  int nargs;
 
  for (index = 1; index < argc; )
    {
      ptr = argv[index];
      if (ptr[0] != '-')
        {
          message("Invalid options format: %s\n", ptr);
          exit(0);
        }

      switch (ptr[1])
        {
          case 'f':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 1)
              {
                message("Frequency out of range: %ld\n", value);
                exit(1);
              }

            pwm->freq = (uint32_t)value;
            index += nargs;
            break;

          case 'd':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 1 || value > 99)
              {
                message("Duty out of range: %ld\n", value);
                exit(1);
              }

            pwm->duty = (uint8_t)value;
            index += nargs;
            break;

#ifdef CONFIG_PWM_PULSECOUNT
          case 'n':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 0)
              {
                message("Count must be non-negative: %ld\n", value);
                exit(1);
              }

            pwm->count = (uint32_t)value;
            index += nargs;
            break;

#endif
          case 'p':
            nargs = arg_string(&argv[index], &str);
            pwm_devpath(pwm, str);
            index += nargs;
            break;

          case 't':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 1 || value > INT_MAX)
              {
                message("Duration out of range: %ld\n", value);
                exit(1);
              }

            pwm->duration = (int)value;
            index += nargs;
            break;

          case 'h':
            pwm_help(pwm);
            exit(0);
        
          default:
            message("Unsupported option: %s\n", ptr);
            pwm_help(pwm);
            exit(1);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_main
 ****************************************************************************/

int pwm_main(int argc, char *argv[])
{
  struct pwm_info_s info;
  int fd;
  int ret;

  /* Initialize the state data */

  if (!g_pwmstate.initialized)
    {
      g_pwmstate.duty        = CONFIG_EXAMPLES_PWM_DUTYPCT;
      g_pwmstate.freq        = CONFIG_EXAMPLES_PWM_FREQUENCY;
      g_pwmstate.duration    = CONFIG_EXAMPLES_PWM_DURATION;
#ifdef CONFIG_PWM_PULSECOUNT
      g_pwmstate.count       = CONFIG_EXAMPLES_PWM_COUNT;
#endif
      g_pwmstate.initialized = true;
    }

  /* Parse the command line */

  parse_args(&g_pwmstate, argc, argv);

  /* Has a device been assigned? */

  if (!g_pwmstate.devpath)
    {
      /* No.. use the default device */

      pwm_devpath(&g_pwmstate, CONFIG_EXAMPLES_PWM_DEVPATH);
    }

  /* Initialization of the PWM hardware is performed by logic external to
   * this test.
   */

  ret = pwm_devinit();
  if (ret != OK)
    {
      message("pwm_main: pwm_devinit failed: %d\n", ret);
      goto errout;
    }

  /* Open the PWM device for reading */

  fd = open(g_pwmstate.devpath, O_RDONLY);
  if (fd < 0)
    {
      message("pwm_main: open %s failed: %d\n", g_pwmstate.devpath, errno);
      goto errout;
    }

  /* Configure the characteristics of the pulse train */

  info.frequency = g_pwmstate.freq;
  info.duty      = ((uint32_t)g_pwmstate.duty << 16) / 100;
#ifdef CONFIG_PWM_PULSECOUNT
  info.count     = g_pwmstate.count;

  message("pwm_main: starting output with frequency: %d duty: %08x count: %d\n",
          info.frequency, info.duty, info.count);

#else
  message("pwm_main: starting output with frequency: %d duty: %08x\n",
          info.frequency, info.duty);

#endif
  
  ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
  if (ret < 0)
    {
      message("pwm_main: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
      goto errout_with_dev;
    }

  /* Then start the pulse train.  Since the driver was opened in blocking
   * mode, this call will block if the count value is greater than zero.
   */

  ret = ioctl(fd, PWMIOC_START, 0);
  if (ret < 0)
    {
      message("pwm_main: ioctl(PWMIOC_START) failed: %d\n", errno);
      goto errout_with_dev;
    }

  /* It a non-zero count was not specified, then wait for the selected
   * duration, then stop the PWM output.
   */
  
#ifdef CONFIG_PWM_PULSECOUNT
  if (info.count == 0)
#endif
    {
      /* Wait for the specified duration */

      sleep(g_pwmstate.duration);

      /* Then stop the  pulse train */

      message("pwm_main: stopping output\n", info.frequency, info.duty);

      ret = ioctl(fd, PWMIOC_STOP, 0);
      if (ret < 0)
        {
          message("pwm_main: ioctl(PWMIOC_STOP) failed: %d\n", errno);
          goto errout_with_dev;
        }
    }

 close(fd);
 msgflush();
 return OK;

errout_with_dev:
  close(fd);
errout:
  msgflush();
  return ERROR;
}
