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
#include <nuttx/cclink/cclink.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


#ifdef CONFIG_CCLINK

/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing the modem driver */
#ifdef CONFIG_MODEM_CCLINK_DEBUG
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

struct cclink_upper
{
  FAR char* path;     /* Registration path */

  /* The contained lower-half driver. */

  FAR struct cclink_lower* lower;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t cclink_read (FAR struct file* filep,
                            FAR char* buffer,
                            size_t buflen);
static ssize_t cclink_write(FAR struct file* filep,
                            FAR const char* buffer,
                            size_t buflen);
static int     cclink_ioctl(FAR struct file* filep,
                            int cmd,
                            unsigned long arg);

#ifndef CONFIG_DISABLE_POLL
static int     cclink_poll (FAR struct file* filep,
                            FAR struct pollfd* fds,
                            bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations cclink_fops =
{
  0,  /* open */
  0,            /* close */
  cclink_read,  /* read */
  cclink_write, /* write */
  0,            /* seek */
  cclink_ioctl, /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  cclink_poll,  /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/


static ssize_t cclink_read(FAR struct file* filep,
                           FAR char* buffer,
                           size_t len)
{
  return 0; /* Return EOF */
}

static ssize_t cclink_write(FAR struct file* filep,
                            FAR const char* buffer,
                            size_t len)
{
  return len; /* Say that everything was written */
}

static int cclink_ioctl(FAR struct file* filep,
                        int cmd,
                        unsigned long arg)
{
  FAR struct inode*         inode = filep->f_inode;
  FAR struct cclink_upper*  upper;
  FAR struct cclink_lower*  lower;
  int                       ret;
  FAR struct cclink_mx_status* status;
  FAR struct cclink_mx_logpage* logpage;

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

    case CCLINK_IOC_SCAN:
      if (lower->ops->cclink_scan)
        {
          ret = lower->ops->cclink_scan(lower);
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

    case CCLINK_IOC_GETLOGPAGE:
      if (lower->ops->cclink_get_logpage)
        {
          logpage = (FAR struct cclink_mx_logpage *) ((uintptr_t)arg);
          ret = lower->ops->cclink_get_logpage(lower, logpage);
        }
      else
        {
          ret = -ENOSYS;
        }

      break;

    case CCLINK_IOC_CONFIG: 
      if(lower->ops->cclink_init)
      {
        ret = lower->ops->cclink_init(lower);
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

    case CCLINK_IOC_GETSTATUS:
      if (lower->ops->cclink_get_status)
        {
          status = (FAR struct cclink_mx_status *) ((uintptr_t) arg);
          if (status)
            {
              ret = lower->ops->cclink_get_status(lower, status);
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
static int cclink_poll(FAR struct file* filep,
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

FAR void* cclink_register(FAR const char *path,
                          FAR struct cclink_lower *lower)
{
  FAR struct cclink_upper *upper;
  int ret;

  DEBUGASSERT(path && lower);

  upper = (FAR struct cclink_upper*)kmm_zalloc(sizeof(struct cclink_upper));
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

  ret = register_driver(path, &cclink_fops, 0666, upper);
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

void cclink_unregister(FAR void *handle)
{
  FAR struct cclink_upper *upper;
  FAR struct cclink_lower *lower;

  upper = (FAR struct cclink_upper*) handle;
  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  m_info("Unregistering: %s\n", upper->path);


  (void) unregister_driver(upper->path);

  kmm_free(upper->path);
  kmm_free(upper);
}


#endif //CONFIG_CCLINK