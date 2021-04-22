

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <poll.h>
#include <errno.h>
#include <unistd.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/modem/cclink.h>

#include <arch/board/board.h>
#include "stm32_gpio.h"
#include "logger-demo.h"

#ifdef CONFIG_CCLINK
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing the modem driver */

#ifdef CONFIG_MODEM_SIM800l_DEBUG
#define m_err _err
#define m_info _info
#else
#define m_err(x...)
#define m_info(x...)
#endif

/*#define UBXMDM_REGISTER_COUNT                           \
  (sizeof(lpc17_ubxmdm_name_pins) /                     \
   sizeof(struct lpc17_name_pin))
*/
/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure type provides the private representation of the "lower-half"
 * driver state.  This type must be coercible to type 'ubxmdm_lower'.
 */

// struct modem_sim800l_lower
// {
//   FAR const struct sim800l_ops *ops; /* Lower half operations */
// };

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* "Lower half" driver methods **********************************************/
static int cclink_scan(FAR struct cclink_lower *lower);
static int cclink_init(FAR struct cclink_lower *lower);
static int cclink_get_logpage(FAR struct cclink_lower *lower, 
                             FAR struct cclink_mx_logpage * logpage);
static int cclink_get_status(FAR struct cclink_lower *lower,
                             FAR struct cclink_status *status);
static int cclink_ioctl(FAR struct cclink_lower *lower,
                         int cmd,
                         unsigned long arg);

/* "Lower half" driver state */

static struct cclink_lower _lower, _lower1, _lower2;

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* "Lower half" driver methods */

static const struct cclink_ops ops =
    {
        .cclink_init = cclink_init,
        .cclink_scan = cclink_scan,
        .cclink_get_logpage = cclink_get_logpage,
        .cclink_get_status = cclink_get_status,
        .ioctl = cclink_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int cclink_init(FAR struct cclink_lower *lower)
{
  (void)lower;

  // stm32_configgpio(MODEM_SIM800l_VLT);

  return OK;
}

static int cclink_scan(FAR struct cclink_lower *lower)
{
  (void)lower;
  //puts("board/src/sim868.c -> power_on()");
  //printf("Powering on.....\n");
  // stm32_gpiowrite(MODEM_SIM800l_VLT, false);
  nxsig_usleep(2000 * 1000);
  nxsig_nanosleep()

  return OK;
}

static int cclink_get_logpage(FAR struct cclink_lower *lower, FAR struct cclink_mx_logpage)
{
  //puts("board/src/sim868.c -> power_off()");

  (void)lower;
  // stm32_gpiowrite(MODEM_SIM800l_VLT, true);
  nxsig_usleep(2000 * 1000);
  return OK;
}



static int cclink_get_status(FAR struct cclink_lower *lower,
                             FAR struct cclink_mx_status *status)
{
  (void)lower;
  //puts("board/src/sim868.c -> get_status()");
  bool result = true;

  status->on = result;

  return OK;
}

static int cclink_ioctl(FAR struct cclink_lower *lower,
                         int cmd,
                         unsigned long arg)
{
  /* No platform-specific IOCTL at the moment. */

  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_ubxmdm_init
 *
 * Description:
 *   Initialize the modem.  The modem is initialized and
 *   registered at '/dev/ubxmdm'.
 *
 * Input Parameters:
 *   usb_used - enables the USB sense pin if 'true'
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void cclink_mx_init()
{
  FAR struct cclink_lower *priv = &_lower;

  //DEBUGASSERT(priv->ops == NULL && priv->pins == NULL);

  /* Initialize the driver state structure.  Here we assume: (1) the state
   * structure lies in .bss and was zeroed at reset time.  (2) This function
   * is only called once so it is never necessary to re-zero the structure.
   */

  priv->ops = &ops;
  priv->channel = CCLINK_MX_CHANNEL_ONE;

  (void)cclink_register("/dev/cclink_mx_ch0", priv);

  priv = &_lower1;
  priv->ops = &ops;
  priv->channel = CCLINK_MX_CHANNEL_TWO;
  (void)cclink_register("/dev/cclink_mx_ch1", priv);

  priv = &_lower2;
  priv->ops = &ops;
  priv->channel = CCLINK_MX_CHANNEL_THREE;
  (void)cclink_register("/dev/cclink_mx_ch1", priv);
}

#endif // CONFIG_MODEM_SIM800l