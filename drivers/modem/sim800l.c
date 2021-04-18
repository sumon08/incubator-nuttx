/****************************************************************************
 * drivers/modem/sim800l.c
 *
 *   Copyright (C) Pi Labs Bangladesh Ltd
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
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/modem/sim800l.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


#ifdef CONFIG_MODEM_SIM800l

/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing the modem driver */
#ifdef CONFIG_MODEM_SIM800L_DEBUG
#  define m_err     _err
#  define m_info    _info
#else
#  define m_err(x...)
#  define m_info(x...)
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* The type of upper half driver state. */

struct sim800l_upper
{
  FAR char* path;     /* Registration path */

  /* The contained lower-half driver. */

  FAR struct sim800l_lower* lower;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t sim800l_read (FAR struct file* filep,
                            FAR char* buffer,
                            size_t buflen);
static ssize_t sim800l_write(FAR struct file* filep,
                            FAR const char* buffer,
                            size_t buflen);
static int     sim800l_ioctl(FAR struct file* filep,
                            int cmd,
                            unsigned long arg);

#ifndef CONFIG_DISABLE_POLL
static int     sim800l_poll (FAR struct file* filep,
                            FAR struct pollfd* fds,
                            bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations sim800l_fops =
{
  0,  /* open */
  0,            /* close */
  sim800l_read,  /* read */
  sim800l_write, /* write */
  0,            /* seek */
  sim800l_ioctl, /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  sim800l_poll,  /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/


static ssize_t sim800l_read(FAR struct file* filep,
                           FAR char* buffer,
                           size_t len)
{
  return 0; /* Return EOF */
}

static ssize_t sim800l_write(FAR struct file* filep,
                            FAR const char* buffer,
                            size_t len)
{
  return len; /* Say that everything was written */
}

static int sim800l_ioctl(FAR struct file* filep,
                        int cmd,
                        unsigned long arg)
{
  FAR struct inode*         inode = filep->f_inode;
  FAR struct sim800l_upper*  upper;
  FAR struct sim800l_lower*  lower;
  int                       ret;
  FAR struct sim800l_status* status;

  m_info("cmd: %d arg: %ld\n", cmd, arg);
  upper = inode->i_private;
  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  switch (cmd)
    {
      /* cmd:         sim800l_IOC_START
       * Description:
       * arg:         Ignored
       */

    case MODEM_IOC_POWERON:
      if (lower->ops->poweron)
        {
          ret = lower->ops->poweron(lower);
        }
      else
        {
          ret = -ENOSYS;
        }

      break;

      /* cmd:         sim800l_IOC_STOP
       * Description:
       * arg:         Ignored
       */

    case MODEM_IOC_POWEROFF:
      if (lower->ops->poweroff)
        {
          ret = lower->ops->poweroff(lower);
        }
      else
        {
          ret = -ENOSYS;
        }

      break;

      /* cmd:         sim800l_IOC_RESET
       * Description:
       * arg:         Ignored
       */

    case MODEM_IOC_RESET:
      if (lower->ops->reset)
        {
          ret = lower->ops->reset(lower);
        }
      else
        {
          ret = -ENOSYS;
        }

      break;

      /* cmd:         sim800l_IOC_GETSTATUS
       * Description:
       * arg:         Writeable pointer to struct sim800l_status.
       */

    case MODEM_IOC_GETSTATUS:
      if (lower->ops->getstatus)
        {
          status = (FAR struct sim800l_status*) ((uintptr_t) arg);
          if (status)
            {
              ret = lower->ops->getstatus(lower, status);
            }
          else
            {
              ret = -EINVAL;
            }
        }
      else
        {
          ret = -ENOSYS;
        }

      break;

      /* Unrecognized IOCTL commands are forwarded to the lower-half IOCTL
       * handler, if defined.
       */

	case MODEM_IOC_INIT:
	if(lower->ops->mdm_open)
	{
		ret = lower->ops->mdm_open(lower);
	}
	else
	{
		ret = -ENOSYS;
	}
	   break;
    default:
      m_info("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);

      if (lower->ops->ioctl)
        {
          ret = lower->ops->ioctl(lower, cmd, arg);
        }
      else
        {
          ret = -ENOSYS;
        }

      break;
    }

  return ret;
}

#ifndef CONFIG_DISABLE_POLL
static int sim800l_poll(FAR struct file* filep,
                       FAR struct pollfd* fds,
                       bool setup)
{
  if (setup)
    {
      fds->revents |= (fds->events & (POLLIN | POLLOUT));
      if (fds->revents != 0)
        {
          nxsem_post(fds->sem);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR void* sim800l_register(FAR const char *path,
                          FAR struct sim800l_lower *lower)
{
  FAR struct sim800l_upper *upper;
  int ret;

  DEBUGASSERT(path && lower);

  upper = (FAR struct sim800l_upper*)
    kmm_zalloc(sizeof(struct sim800l_upper));
  if (!upper)
    {
      m_err("ERROR: Upper half allocation failed\n");
      goto errout;
    }

  upper->lower = lower;
  upper->path = strdup(path);
  if (!upper->path)
    {
      m_err("ERROR: Path allocation failed\n");
      goto errout_with_upper;
    }

  ret = register_driver(path, &sim800l_fops, 0666, upper);
  if (ret < 0)
    {
      m_err("ERROR: register_driver failed: %d\n", ret);
      goto errout_with_path;
    }

  return (FAR void*) upper;

errout_with_path:
  kmm_free(upper->path);

errout_with_upper:
  kmm_free(upper);

errout:
  return NULL;
}

void sim800l_unregister(FAR void *handle)
{
  FAR struct sim800l_upper *upper;
  FAR struct sim800l_lower *lower;

  upper = (FAR struct sim800l_upper*) handle;
  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  m_info("Unregistering: %s\n", upper->path);

  DEBUGASSERT(lower->ops->poweroff);
  (void) lower->ops->poweroff(lower);

  (void) unregister_driver(upper->path);

  kmm_free(upper->path);
  kmm_free(upper);
}


#endif //CONFIG_MODEM_SIM800l