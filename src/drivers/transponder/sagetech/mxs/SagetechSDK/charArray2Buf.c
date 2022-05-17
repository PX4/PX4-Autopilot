/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file charArray2Buf.c
 * @author Jacob.Garrison
 *
 * @date Mar 2, 2021
 *      
 */

#include "sgUtil.h"
#include <ctype.h>

/**
 * given ASCII input determines whether input is a lower case value.
 *
 * @param[in] c  The character to test if it is a lower case character.
 *
 * @return true if lower case; false otherwise.
 *
 * @warning data in c parameter must be pre-validated (valid ASCII char).
 */

/*
static inline int islower(int c)
{
	if ((c >= 97) && (c <= 122))
	{
		return 1;
	}
	return 0;
}
*/

/**
 * given ASCII input changes value to upper case unless upper case.
 *
 * @param[in] c  The character to switch to upper case if it is a lower
 *               case character.
 *
 * @return upper case ASCII.
 *
 * @warning data in c parameter must be pre-validated (valid ASCII char).
 */
/*
static inline int toupper(int c)
{
	return islower(c) ? c - 'a' + 'A' : c;
}
*/
/*
 * Documented in the header file.
 */
void charArray2Buf(uint8_t *bufferPos, char arr[], uint8_t len)
{
   for (uint8_t i = 0; i < len; ++i)
   {
      bufferPos[i] = toupper(arr[i]);
   }
}
