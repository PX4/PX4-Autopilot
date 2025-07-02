/**
 * Boom LED brightness
 *
 * Overall brightness of all boom LEDs
 * 0-1, with 0 being off and 1 being brightest
 *
 * @min 0
 * @max 1
 * @group Freefly
 */
PARAM_DEFINE_FLOAT(BOOM_BRT, 1.f);

/**
 * Boom 1 LED Color
 *
 * Color when disarmed
 *
 * @value 0 Off
 * @value 1 Red
 * @value 2 Orange
 * @value 3 Yellow
 * @value 4 Green
 * @value 5 Cyan
 * @value 6 Blue
 * @value 7 Purple
 * @value 8 White
 * @value 9 RGB Wheel
 *
 * @group Freefly
 */
PARAM_DEFINE_INT32(BOOM1_COLOR, 4);

/**
 * Boom 2 LED Color
 *
 * Color when disarmed
 *
 * @value 0 Off
 * @value 1 Red
 * @value 2 Orange
 * @value 3 Yellow
 * @value 4 Green
 * @value 5 Cyan
 * @value 6 Blue
 * @value 7 Purple
 * @value 8 White
 * @value 9 RGB Wheel
 *
 * @group Freefly
 */
PARAM_DEFINE_INT32(BOOM2_COLOR, 1);

/**
 * Boom 3 LED Color
 *
 * Color when disarmed
 *
 * @value 0 Off
 * @value 1 Red
 * @value 2 Orange
 * @value 3 Yellow
 * @value 4 Green
 * @value 5 Cyan
 * @value 6 Blue
 * @value 7 Purple
 * @value 8 White
 * @value 9 RGB Wheel
 *
 * @group Freefly
 */
PARAM_DEFINE_INT32(BOOM3_COLOR, 1);

/**
 * Boom 4 LED Color
 *
 * Color when disarmed
 * 0 = Off
 * 1 = RED
 * 2 = ORANGE
 * 3 = YELLOW
 * 4 = GREEN
 * 5 = CYAN
 * 6 = BLUE
 * 7 = PURPLE
 * 8 = WHITE
 *
 * @value 0 Off
 * @value 1 Red
 * @value 2 Orange
 * @value 3 Yellow
 * @value 4 Green
 * @value 5 Cyan
 * @value 6 Blue
 * @value 7 Purple
 * @value 8 White
 * @value 9 RGB Wheel
 *
 * @group Freefly
 */
PARAM_DEFINE_INT32(BOOM4_COLOR, 4);
