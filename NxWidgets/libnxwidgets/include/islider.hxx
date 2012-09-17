/****************************************************************************
 * NxWidgets/libnxwidgets/include/islider.hxx
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name NuttX, NxWidgets, nor the names of its contributors
 *    me be used to endorse or promote products derived from this software
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
 ****************************************************************************
 *
 * Portions of this package derive from Woopsi (http://woopsi.org/) and
 * portions are original efforts.  It is difficult to determine at this
 * point what parts are original efforts and which parts derive from Woopsi.
 * However, in any event, the work of  Antony Dzeryn will be acknowledged
 * in most NxWidget files.  Thanks Antony!
 *
 *   Copyright (c) 2007-2011, Antony Dzeryn
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the names "Woopsi", "Simian Zombie" nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Antony Dzeryn ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Antony Dzeryn BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_ISLIDER_HXX
#define __INCLUDE_ISLIDER_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * Abstract Base Classes
 ****************************************************************************/
 
#if defined(__cplusplus)

namespace NXWidgets
{
  /**
   * Defines the interface for slider widgets.
   */
  class ISlider
  {
  public:
    /**
     * A virtual destructor is required in order to override the ISlider
     * destructor.  We do this because if we delete ISlider, we want the
     * destructor of the class that inherits from ISlider to run, not this
     * one.
     */

    virtual ~ISlider(void) { }

    /**
     * Get the smallest value that the slider can represent.
     *
     * @return The smallest value.
     */

    virtual const nxgl_coord_t getMinimumValue(void) const = 0;

    /**
     * Get the largest value that the slider can represent.
     *
     * @return The largest value.
     */

    virtual const nxgl_coord_t getMaximumValue(void) const = 0;

    /**
     * Get the current value of the slider.
     *
     * return The current slider value.
     */

    virtual const nxgl_coord_t getValue(void) const = 0;

    /**
     * Get the value represented by the height of the grip.
     * For sliders, this would typically be 1 (so each new
     * grip position is worth 1).  For scrollbars, this
     * would be the height of the scrolling widget.
     *
     * @return The page size.
     */

    virtual const nxgl_coord_t getPageSize(void) const = 0;

    /**
     * Set the smallest value that the slider can represent.
     *
     * @param value The smallest value.
     */

    virtual void setMinimumValue(const nxgl_coord_t value) = 0;

    /**
     * Set the largest value that the slider can represent.
     *
     * @param value The largest value.
     */

    virtual void setMaximumValue(const nxgl_coord_t value) = 0;

    /**
     * Set the value that of the slider.  This will reposition
     * and redraw the grip.
     *
     * @param value The new value.
     */

    virtual void setValue(const nxgl_coord_t value) = 0;

    /**
     * Set the value that of the slider.  This will reposition and redraw
     * the grip.  The supplied value should be bitshifted left 16 places.
     * This ensures greater accuracy than the standard setValue() method if
     * the slider is being used as a scrollbar.
     *
     * @param value The new value.
     */

    virtual void setValueWithBitshift(const int32_t value) = 0;

    /**
     * Set the page size represented by the grip.
     *
     * @param pageSize The page size.
     * @see getPageSize().
     */

    virtual void setPageSize(const nxgl_coord_t pageSize) = 0;
  };
}

#endif // __cplusplus

#endif // __INCLUDE_ISLIDER_HXX

