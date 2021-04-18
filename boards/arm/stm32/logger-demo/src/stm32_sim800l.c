

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
#include <nuttx/modem/sim868.h>

#include <arch/board/board.h>
#include "stm32_gpio.h"
#include "logger-demo.h"

#ifdef CONFIG_MODEM_SIM800l
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

struct modem_sim800l_lower
{
  FAR const struct sim800l_ops *ops; /* Lower half operations */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* "Lower half" driver methods **********************************************/
static int sim800l_config(FAR struct sim800l_lower *lower);
static int sim800l_poweron(FAR struct sim800l_lower *lower);
static int sim800l_poweroff(FAR struct sim800l_lower *lower);
static int sim800l_reset(FAR struct sim800l_lower *lower);
static int sim800l_getstatus(FAR struct sim800l_lower *lower,
                             FAR struct sim800l_status *status);
static int sim800l_ioctl(FAR struct sim800l_lower *lower,
                         int cmd,
                         unsigned long arg);

/* "Lower half" driver state */

static struct modem_sim800l_lower modem_lower;

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* "Lower half" driver methods */

static const struct sim800l_ops modem_sim800l_ops =
    {
        .mdm_open = sim800l_config,
        .poweron = sim800l_poweron,
        .poweroff = sim800l_poweroff,
        .reset = sim800l_reset,
        .getstatus = sim800l_getstatus,
        .ioctl = sim800l_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int sim800l_config(FAR struct sim800l_lower *lower)
{
  (void)lower;

  stm32_configgpio(MODEM_SIM800l_VLT);

  return OK;
}

static int sim800l_poweron(FAR struct sim800l_lower *lower)
{
  (void)lower;
  //puts("board/src/sim868.c -> power_on()");
  //printf("Powering on.....\n");
  stm32_gpiowrite(MODEM_SIM800l_VLT, false);
  nxsig_usleep(2000 * 1000);

  return OK;
}

static int sim800l_poweroff(FAR struct sim800l_lower *lower)
{
  //puts("board/src/sim868.c -> power_off()");

  (void)lower;
  stm32_gpiowrite(MODEM_SIM800l_VLT, true);
  nxsig_usleep(2000 * 1000);
  return OK;
}

static int sim800l_reset(FAR struct sim800l_lower *lower)
{
  (void)lower;

  stm32_gpiowrite(MODEM_SIM800l_VLT, true);
  nxsig_usleep(2000 * 1000);
  stm32_gpiowrite(MODEM_SIM800l_VLT, false);
  nxsig_usleep(2000 * 1000);
  return OK;
}

static int sim800l_getstatus(FAR struct sim800l_lower *lower,
                             FAR struct sim800l_status *status)
{
  (void)lower;
  //puts("board/src/sim868.c -> get_status()");
  bool result = true;

  status->on = result;

  return OK;
}

static int sim800l_ioctl(FAR struct sim800l_lower *lower,
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

void modem_sim800l_init()
{
  FAR struct modem_sim800l_lower *priv = &modem_lower;

  //DEBUGASSERT(priv->ops == NULL && priv->pins == NULL);

  /* Initialize the driver state structure.  Here we assume: (1) the state
   * structure lies in .bss and was zeroed at reset time.  (2) This function
   * is only called once so it is never necessary to re-zero the structure.
   */

  priv->ops = &modem_sim800l_ops;

  (void)sim800l_register("/dev/sim800l", (FAR struct sim800l_lower *)priv);
}

#endif // CONFIG_MODEM_SIM800l