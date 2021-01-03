
#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/leds/userled.h>

#include "board_config.h"


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static userled_set_t userled_supported(FAR const struct userled_lowerhalf_s *lower);
static void userled_led(FAR const struct userled_lowerhalf_s *lower,
                        int led, bool ledon);
static void userled_ledset(FAR const struct userled_lowerhalf_s *lower,
                           userled_set_t ledset);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the user LED lower half driver interface */

static const struct userled_lowerhalf_s g_userled_lower =
{
  .ll_supported = userled_supported,
  .ll_led       = userled_led,
  .ll_ledset    = userled_ledset,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: userled_supported
 *
 * Description:
 *   Return the set of LEDs supported by the board
 *
 ****************************************************************************/

static userled_set_t userled_supported(FAR const struct userled_lowerhalf_s *lower)
{
  ledinfo("BOARD_NLEDS: %02x\n", BOARD_NLEDS);
  return (userled_set_t)((1 << BOARD_NLEDS) - 1);
}

/****************************************************************************
 * Name: userled_led
 *
 * Description:
 *   Set the current state of one LED
 *
 ****************************************************************************/

static void userled_led(FAR const struct userled_lowerhalf_s *lower,
                        int led, bool ledon)
{
  board_userled(led, ledon);
}

/****************************************************************************
 * Name: userled_led
 *
 * Description:
 *   Set the state of all LEDs
 *
 ****************************************************************************/

static void userled_ledset(FAR const struct userled_lowerhalf_s *lower,
                           userled_set_t ledset)
{
  board_userled_all(ledset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: userled_lower_initialize
 *
 * Description:
 *   Initialize the generic LED lower half driver, bind it and register
 *   it with the upper half LED driver as devname.
 *
 ****************************************************************************/

int userled_lower_initialize(FAR const char *devname)
{
  board_userled_initialize();
  return userled_register(devname, &g_userled_lower);
}
