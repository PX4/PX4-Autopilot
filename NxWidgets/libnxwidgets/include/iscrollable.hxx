/****************************************************************************
 * NxWidgets/libnxwidgets/include/iscrollable.hxx
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

#ifndef __INCLUDE_ISCROLLABLE_HXX
#define __INCLUDE_ISCROLLABLE_HXX

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
 * Implementation Classes
 ****************************************************************************/
 
#if defined(__cplusplus)

namespace NXWidgets
{
  /**
   * Abstract class defining basic functionality of scrolling widgets.
   * Scrolling regions are modelled as a virtual "canvas", or rectangular
   * region, with height/width dimensions and x/y coordinates.
   */

  class IScrollable
  {
  public:
    /**
     * A virtual destructor is required in order to override the IScrollable
     * destructor.  We do this because if we delete IScrollable, we want the
     * destructor of the class that inherits from IScrollable to run, not this
     * one.
     */

    virtual ~IScrollable(void) { }

    /**
     * Gets the x coordinate of the virtual canvas.
     *
     * @return The x coordinate of the virtual canvas.
     */

    virtual const int32_t getCanvasX(void) const = 0;
    
    /**
     * Gets the y coordinate of the virtual canvas.
     *
     * @return The y coordinate of the virtual canvas.
     */

    virtual const int32_t getCanvasY(void) const = 0;

    /**
     * Gets the width of the virtual canvas.
     *
     * @return The width of the virtual canvas.
     */

    virtual const int32_t getCanvasWidth(void) const = 0;
    
    /**
     * Gets the height of the virtual canvas.
     *
     * @return The height of the virtual canvas.
     */

    virtual const int32_t getCanvasHeight(void) const = 0;

    /**
     * Scrolls the virtual canvas by the specified amounts.
     *
     * @param dx Distance to scroll horizontally.
     * @param dy Distance to scroll vertically.
     */

    virtual void scroll(int32_t dx, int32_t dy) = 0;
    
    /**
     * Repositions the virtual canvas to the specified coordinates.
     *
     * @param x New x coordinate of the virtual canvas.
     * @param y New y coordinate of the virtual canvas.
     */

    virtual void jump(int32_t x, int32_t y) = 0;

    /**
     * Returns true if vertical scrolling is allowed.
     *
     * @return True if vertical scrolling is allowed.
     */

    virtual bool allowsVerticalScroll(void) const = 0;

    /**
     * Returns true if horizontal scrolling is allowed.
     *
     * @return True if horizontal scrolling is allowed.
     */

    virtual bool allowsHorizontalScroll(void) const = 0;

    /**
     * Set whether or not horizontal scrolling is allowed.
     *
     * @param allow True to allow horizontal scrolling; false to deny it.
     */

    virtual void setAllowsVerticalScroll(bool allow) = 0;

    /**
     * Set whether or not horizontal scrolling is allowed.
     *
     * @param allow True to allow horizontal scrolling; false to deny it.
     */

    virtual void setAllowsHorizontalScroll(bool allow) = 0;

    /**
     * Sets the width of the virtual canvas.
     *
     * @param width The width of the virtual canvas.
     */

    virtual void setCanvasWidth(const int32_t width) = 0;
    
    /**
     * Sets the height of the virtual canvas.
     *
     * @param height The height of the virtual canvas.
     */

    virtual void setCanvasHeight(const int32_t height) = 0;
  };
}

#endif // __cplusplus

#endif // __INCLUDE_ISCROLLABLE_HXX

