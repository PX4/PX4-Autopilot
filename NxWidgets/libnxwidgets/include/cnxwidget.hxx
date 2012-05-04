/****************************************************************************
 * include/cnxwidget.hxx
 * NxWidgets/libnxwidgets/
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

#ifndef __INCLUDE_CNXWIDGET_HXX
#define __INCLUDE_CNXWIDGET_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <ctime>

#include <nuttx/nx/nxglib.h>

#include "cnxstring.hxx"
#include "cnxfont.hxx"
#include "crect.hxx"
#include "cwidgetstyle.hxx"
#include "cwidgeteventargs.hxx"
#include "cwidgeteventhandler.hxx"
#include "cwidgeteventhandlerlist.hxx"
#include "tnxarray.hxx"

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
   * Forward references
   */

  class CWidgetControl;
  class CGraphicsPort;
  class CNxFont;
  class CRectCache;
  class CWidgetEventHandlerList;

  /**
   * Class providing all the basic functionality of a NxWidget.  All other
   * widgets must must inherit from this class.
   */

  class CNxWidget
  {
  public:

    /**
     * Enum listing flags that can be set in the constructor's "flags" parameter.
     */

    enum WidgetFlagType
    {
      WIDGET_BORDERLESS       = 0x0001,  /**< Widget has no border */
      WIDGET_DRAGGABLE        = 0x0002,  /**< Widget can be dragged by the user */
      WIDGET_PERMEABLE        = 0x0004,  /**< Widget's children can exceed this widget's edges */
      WIDGET_DOUBLE_CLICKABLE = 0x0008,  /**< Widget can be double-clicked */
      WIDGET_NO_RAISE_EVENTS  = 0x0010,  /**< Widget does not raise events */
    };

    /**
     * Struct describing some basic properties of a widget.
     */

    typedef struct
    {
      uint8_t clicked         : 1;       /**< True if the widget is currently clicked. */
      uint8_t hasFocus        : 1;       /**< True if the widget has focus. */
      uint8_t dragging        : 1;       /**< True if the widget is being dragged. */
      uint8_t deleted         : 1;       /**< True if the widget has been deleted. */
      uint8_t borderless      : 1;       /**< True if the widget is borderless. */
      uint8_t draggable       : 1;       /**< True if the widget can be dragged. */
      uint8_t drawingEnabled  : 1;       /**< True if the widget can be drawn. */
      uint8_t enabled         : 1;       /**< True if the widget is enabled. */
      uint8_t permeable       : 1;       /**< True if the widget's children can exceed its dimensions. */
      uint8_t erased          : 1;       /**< True if the widget is currently erased from the frame buffer. */
      uint8_t visibleRegionCacheInvalid : 1;  /**< True if the region cache is invalid. */
      uint8_t hidden          : 1;       /**< True if the widget is hidden. */
      uint8_t doubleClickable : 1;       /**< True if the widget can be double-clicked. */
      uint8_t modal           : 1;       /**< True if the widget is modal. */
    } Flags;

    /**
     * Struct describing the size of all four borders of a widget.
     */

    typedef struct
    {
      uint8_t top;                    /**< Height of the top border. */
      uint8_t right;                  /**< Width of the right border. */
      uint8_t bottom;                 /**< Height of the bottom border. */
      uint8_t left;                   /**< Width of the left border. */
    } WidgetBorderSize;

  protected:
    CWidgetControl *m_widgetControl;  /**< The controlling widget for the display */
    CRect m_rect;                     /**< Rectange bounding the widget. */

    // Dragging variables

    nxgl_coord_t m_grabPointX;        /**< Physical space x coordinate where dragging began. */
    nxgl_coord_t m_grabPointY;        /**< Physical space y coordinate where dragging began. */
    nxgl_coord_t m_newX;              /**< Physical x coordinate where widget is being dragged to. */
    nxgl_coord_t m_newY;              /**< Physical y coordinate where widget is being dragged to. */

    // Style

    CWidgetStyle m_style;             /**< All style information used by a widget. */

    // Status

    Flags m_flags;                    /**< Flags struct. */

    // Event handling

    CWidgetEventHandlerList *m_widgetEventHandlers; /**< List of event handlers. */

    // Double-clicking
    
    struct timespec m_lastClickTime;  /**< System timer when last clicked. */
    nxgl_coord_t m_lastClickX;        /**< X coordinate of last click. */
    nxgl_coord_t m_lastClickY;        /**< Y coordinate of last click. */
    int m_doubleClickBounds;          /**< Area in which a click is assumed to be a double-click. */

    // Hierarchy control

    CNxWidget *m_parent;              /**< Pointer to the widget's parent. */
    CNxWidget *m_focusedChild;        /**< Pointer to the child widget that has focus. */
    TNxArray<CNxWidget*> m_children;  /**< List of child widgets. */

    // Visible regions

    CRectCache *m_rectCache;          /**< List of the widget's visible regions. */
    WidgetBorderSize m_borderSize;    /**< Size of the widget borders. */

    /**
     * Use the provided widget style
     */

    void useWidgetStyle(const CWidgetStyle *style);

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     *
     * @param port The CGraphicsPort to draw to.
     * @see redraw().
     */

    virtual inline void drawContents(CGraphicsPort* port) { }

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     *
     * @param port The CGraphicsPort to draw to.
     * @see redraw().
     */

    virtual void drawBorder(CGraphicsPort* port) { }

    /**
     * Checks if the supplied coordinates collide with a portion of this widget
     * that is not obscured by its siblings, but that may be obscured by
     * its children.
     *
     * @param x X coordinate of the click.
     * @param y Y coordinate of the click.
     * @return True if a collision occurred; false if not.
     */

    bool checkCollisionWithForegroundRects(nxgl_coord_t x, nxgl_coord_t y) const;

    /**
     * Draw all visible regions of this widget's children.
     */

    void drawChildren(void);

    /**
     * Erase and remove the supplied child widget from this widget and
     * send it to the deletion queue.
     *
     * @param widget The widget to close.
     * @see close().
     */

    void closeChild(CNxWidget *widget);

    /**
     * Redraws all regions of child widgets that fall within the invalidRects
     * regions.
     *
     * @param invalidRects List of invalid regions that need to be redrawn.
     * @param sender Pointer to the widget that initiated the redraw.
     */

    void redrawDirtyChildren(TNxArray<CRect>* invalidRects, CNxWidget *sender);

    /**
     * Get the index of the next visible widget higher up the z-order.
     *
     * @param startIndex The starting index.
     * @return The index of the next highest visible widget.
     */

    const int getHigherVisibleWidget(const int startIndex) const;

    /**
     * Get the index of the next visible widget lower down the z-order.
     *
     * @param startIndex The starting index.
     * @return The index of the next lowest visible widget.
     */

    const int getLowerVisibleWidget(const int startIndex) const;

    /**
     * Notify this widget that it is being dragged, and set its drag point.
     *
     * @param x The x coordinate of the drag position relative to this widget.
     * @param y The y coordinate of the drag position relative to this widget.
     */

    void startDragging(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Notify this widget that it is no longer being dragged.
     */

    void stopDragging(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Copy constructor is protected to prevent usage.
     */

    inline CNxWidget(const CNxWidget &widget) { }

    /**
     * Called when the widget is clicked.  Override this when creating new
     * widgets if the widget should exhibit additional behaviour when it is
     * clicked.
     *
     * @param x The x coordinate of the click.
     * @param y The y coordinate of the click.
     */

    virtual inline void onClick(nxgl_coord_t x, nxgl_coord_t y) { }
    
    /**
     * Called when the widget is double-clicked.  Override this when
     * creating new widgets if the widget should exhibit additional
     * behaviour when it is double-clicked.  To change the conditions that
     * apply in detecting a double-click, override the isDoubleClicked()
     * method.
     *
     * @param x The x coordinate of the click.
     * @param y The y coordinate of the click.
     */

    virtual inline void onDoubleClick(nxgl_coord_t x, nxgl_coord_t y) { }

    /**
     * Called when the widget is released.  Override this when
     * creating new widgets if the widget should exhibit additional
     * behaviour when it is released.
     *
     * @param x The x coordinate of the mouse when released.
     * @param y The y coordinate of the mouse when released.
     */

    virtual inline void onRelease(nxgl_coord_t x, nxgl_coord_t y) { }
    
    /**
     * Called when the widget is released outside of its boundaries.
     * Override this when creating new widgets if the widget should exhibit
     * additional behaviour when it is released outside of its boundaries.
     *
     * @param x The x coordinate of the mouse when released.
     * @param y The y coordinate of the mouse when released.
     */

    virtual inline void onReleaseOutside(nxgl_coord_t x, nxgl_coord_t y) { }
    
    /**
     * Called when the widget is dragged.  Override this when creating new
     * widgets if the widget should exhibit additional behaviour when it is
     * dragged.
     *
     * @param x The x coordinate of the mouse.
     * @param y The y coordinate of the mouse.
     * @param vX X distance dragged.
     * @param vY Y distance dragged.
     */

    virtual inline void onDrag(nxgl_coord_t x, nxgl_coord_t y,
                               nxgl_coord_t vX, nxgl_coord_t vY) { }
    
    /**
     * Called when the widget starts being dragged.  Override this when
     * creating new widgets if the widget should exhibit additional
     * behaviour when dragging starts.
     */

    virtual inline void onDragStart(void) { }
    
    /**
     * Called when the widget stops being dragged.  Override this when
     * creating new widgets if the widget should exhibit additional
     * behaviour when dragging stops.
     */

    virtual inline void onDragStop(void) { }
    
    /**
     * Called when the widget gains focus.  Override this when creating new
     * widgets if the widget should exhibit additional behaviour when
     * gaining focus.
     */

    virtual inline void onFocus(void) { }
    
    /**
     * Called when the widget loses focus.  Override this when creating new
     * widgets if the widget should exhibit additional behaviour when
     * losing focus.
     */

    virtual inline void onBlur(void) { }

    /**
     * Called when the widget is enabled.  Override this when creating new
     * widgets if the widget should exhibit additional behaviour when
     * enabled.
     */

    virtual inline void onEnable(void) { }
    
    /**
     * Called when the widget is disabled.  Override this when creating new
     * widgets if the widget should exhibit additional behaviour when
     * disabled.
     */

    virtual inline void onDisable(void) { }
    
    /**
     * Called when the widget is resized.  Override this when creating new
     * widgets if the widget should exhibit additional behaviour when
     * resized.
     *
     * @param width The new width.
     * @param height The new height.
     */

    virtual inline void onResize(nxgl_coord_t width, nxgl_coord_t height) { }

  public:

    /**
     * CNxWidget constructor.
     *
     * @param pWidgetControl The controllwing widget for the display
     * @param x The x coordinate of the widget.
     * @param y The y coordinate of the widget.
     * @param width The width of the widget.
     * @param height The height of the widget.
     * @param flags Bitmask specifying some set-up values for the widget.
     * @param style The style that the button should use.  If this is not
     *        specified, the button will use the global default widget
     *        style.
     * @see WidgetFlagType.
     */

    CNxWidget(CWidgetControl *pWidgetControl,
              nxgl_coord_t x, nxgl_coord_t y,
              nxgl_coord_t width, nxgl_coord_t height,
              uint32_t flags,
              FAR const CWidgetStyle *style = (FAR const CWidgetStyle *)NULL);

    /**
     * Destructor.
     */

    virtual ~CNxWidget(void);

    /**
     * Get the x coordinate of the widget in "Widget space".
     *
     * @return Widget space x coordinate.
     */

    const nxgl_coord_t getX(void) const;

    /**
     * Get the y coordinate of the widget in "Widget space".
     *
     * @return Widget space y coordinate.
     */

    const nxgl_coord_t getY(void) const;

    /**
     * Get the x coordinate of the widget relative to its parent.
     *
     * @return Parent-space x coordinate.
     */

    const nxgl_coord_t getRelativeX(void) const;

    /**
     * Get the y coordinate of the widget relative to its parent.
     *
     * @return Parent-space y coordinate.
     */

    const nxgl_coord_t getRelativeY(void) const;

    /**
     * Is the widget active?
     * A value of true indicates that this widget has focus or is an ancestor
     * of the widget with focus.
     *
     * @return True if active.
     */

    inline const bool hasFocus(void) const
    {
      return m_flags.hasFocus;
    }

    /**
     * Has the widget been marked for deletion?  This function recurses up the widget
     * hierarchy and only returns true if all of the widgets in the ancestor
     * chain are not deleted.  
     *
     * Widgets marked for deletion are automatically deleted and should not be
     * interacted with.
     *
     * @return True if marked for deletion.
     */

    const bool isDeleted(void) const;

    /**
     * Is the widget allowed to draw?  This function recurses up the widget
     * hierarchy and only returns true if all of the widgets in the ancestor
     * chain are visible.
     *
     * @return True if drawing is enabled.
     */

    const bool isDrawingEnabled(void) const;

    /**
     * Is the widget hidden?  This function recurses up the widget
     * hierarchy and returns true if any of the widgets in the ancestor
     * chain are hidden.
     *
     * @return True if hidden.
     */

    const bool isHidden(void) const;

    /**
     * Is the widget enabled?  This function recurses up the widget
     * hierarchy and only returns true if all of the widgets in the ancestor
     * chain are enabled.
     *
     * @return True if enabled.
     */

    const bool isEnabled(void) const;

    /**
     * Are the widget's edges permeable or solid?
     * Permeable widgets do not enforce their dimensions on the
     * coordinates and dimensions of child widgets.
     *
     * @return True if permeable.
     */

    inline const bool isPermeable(void) const
    {
      return m_flags.permeable;
    }

    /**
     * IS the widget double-clickable?
     * @return True if the widget watches for double-clicks.
     */

    inline const bool isDoubleClickable(void) const
    {
      return m_flags.doubleClickable;
    }

    /**
     * Does the widget have a border?
     *
     * @return True if the widget does not have a border.
     */

    inline const bool isBorderless(void) const
    {
      return m_flags.borderless;
    }

    /**
     * Is the widget clicked?
     *
     * @return True if the widget is currently clicked.
     */

    inline const bool isClicked(void) const
    {
      return m_flags.clicked;
    }

    /**
     * Is the widget being dragged?
     *
     * @return True if the widget is currently being dragged.
     */

    inline const bool isBeingDragged(void) const
    {
      return m_flags.dragging;
    }

    /**
     * Is the widget modal?  Only true if the Widget singleton is also modal.
     *
     * @return True if the widget is modal.
     */

    const bool isModal(void) const;

    /**
     * Get the width of the widget.
     *
     * @return The widget width.
     */

    inline nxgl_coord_t getWidth(void) const
    {
      return m_rect.getWidth();
    }

    /**
     * Get the height of the widget.
     *
     * @return The widget height.
     */

    inline nxgl_coord_t getHeight(void) const
    {
      return m_rect.getHeight();
    }

     /**
     * Get the size of the widget
     *
     * @return The widgets's size
     */

    inline void getSize(struct nxgl_size_s &size) const
    {
      size.h = m_rect.getHeight();
      size.w = m_rect.getWidth();
    }

    /**
     * Get the position of the widget
     *
     * @return The widgets's position
     */

    inline void getPos(struct nxgl_point_s &pos) const
    {
      pos.x = m_rect.getX();
      pos.y = m_rect.getY();
    }

    /**
     * Get the window bounding box in physical display coordinated.
     *
     * @return This function returns the window handle.
     */

    inline CRect getBoundingBox(void)
    {
      return CRect(m_rect);
    }

    /**
     * Get the dimensions of the border
     *
     */

    inline void getBorderSize(WidgetBorderSize &borderSize)
    {
      borderSize.top    = m_borderSize.top;
      borderSize.left   = m_borderSize.left;
      borderSize.bottom = m_borderSize.bottom;
      borderSize.right  = m_borderSize.right;
    }

    /**
     * Get a pointer to this widget's parent.
     *
     * @return This widget's parent.
     */

    inline CNxWidget *getParent(void) const
    {
      return m_parent;
    }

    /**
     * Get a pointer to this widget's focused child.
     *
     * @return This widget's focused child.
     */

    inline CNxWidget *getFocusedWidget(void)
    {
      return m_focusedChild;
    }

    /**
     * Check if this widget raises events or not.
     *
     * @return True if events are enabled.
     */

    inline const bool raisesEvents(void) const
    {
      return m_widgetEventHandlers->isEnabled();
    }

    /**
     * Insert the dimensions that this widget wants to have into the rect
     * passed in as a parameter.  All coordinates are relative to the widget's
     * parent.
     *
     * @param rect Reference to a rect to populate with data.
     */

    virtual void getPreferredDimensions(CRect &rect) const;

    /**
     * Insert the properties of the space within this widget that is
     * available for children into the rect passed in as a parameter.
     * All coordinates are relative to this widget.
     *
     * @param rect Reference to a rect to populate with data.
     */

    void getClientRect(CRect &rect) const;

    /**
     * Insert the properties of the space within this widget that is
     * available for children into the rect passed in as a parameter.
     * Identical to getClientRect() except that all coordinates are
     * absolute positions within the window.
     *
     * @param rect Reference to a rect to populate with data.
     */

    void getRect(CRect &rect) const;

    /**
     * Clips the supplied rect to the boundaries defined by this widget and
     * this widget's parents.
     *
     * @param rect Reference to a rect to populate with data.
     */

    void getRectClippedToHierarchy(CRect &rect) const;

    /**
     * Gets a pointer to the vector of all of the visible regions of this widget,
     * including any covered by children.
     *
     * @return A pointer to a vector of all visible regions.
     */

    TNxArray<CRect> *getForegroundRegions(void);

    /**
     * Gets a pointer to the widget's font.
     *
     * @return A pointer to the widget's font.
     */

    inline CNxFont *getFont(void) const
    {
      return m_style.font;
    }

    /**
     * Gets the color used for the normal background fill.
     *
     * @return Background fill color.
     */

    inline const nxgl_mxpixel_t getBackgroundColor(void) const
    {
      return m_style.colors.background;
    }

    /**
     * Gets the color used for the background fill when the widget is selected.
     *
     * @return Dark color.
     */

    inline const nxgl_mxpixel_t getSelectedBackgroundColor(void) const
    {
      return m_style.colors.selectedBackground;
    }

    /**
     * Gets the color used as the light edge in bevelled boxes.
     *
     * @return Shine color.
     */

    inline const nxgl_mxpixel_t getShineEdgeColor(void) const
    {
      return m_style.colors.shineEdge;
    }

    /**
     * Gets the color used as the dark edge in bevelled boxes.
     *
     * @return Shadow color.
     */

    inline const nxgl_mxpixel_t getShadowEdgeColor(void) const
    {
      return m_style.colors.shadowEdge;
    }

    /**
     * Gets the color used as the fill in focused window borders.
     *
     * @return Highlight color.
     */

    inline const nxgl_mxpixel_t getHighlightColor(void) const
    {
      return m_style.colors.highlight;
    }

    /**
     * Gets the color used for text in a disabled widget.
     *
     * @return Disabled text color.
     */

    inline const nxgl_mxpixel_t getDisabledTextColor(void) const
    {
      return m_style.colors.disabledText;
    }

    /**
     * Gets the color used for text in a enabled widget.
     *
     * @return Enabled text color.
     */

    inline const nxgl_mxpixel_t getEnabledTextColor(void) const
    {
      return m_style.colors.enabledText;
    }

    /**
     * Gets the color used for text in a clicked widget.
     *
     * @return Selected text color.
     */

    inline const nxgl_mxpixel_t getSelectedTextColor(void) const
    {
      return m_style.colors.selectedText;
    }

    /**
     * Sets this widget's border state.
     *
     * @param isBorderless The border state.
     */

    void setBorderless(bool isBorderless);

    /**
     * Sets whether or not this widget can be dragged.
     *
     * @param isDraggable The draggable state.
     */

    inline void setDraggable(const bool isDraggable)
    {
      m_flags.draggable = isDraggable;
    }

    /**
     * Sets whether or not child widgets can exceed this widget's dimensions.
     *
     * @param permeable The permeable state.
     */

    inline void setPermeable(const bool permeable)
    {
      m_flags.permeable = permeable;
    }

    /**
     * Sets whether or not the widgets processes double-clicks.
     *
     * @param doubleClickable The double-clickable state.
     */

    inline void setDoubleClickable(const bool doubleClickable)
    {
      m_flags.doubleClickable = doubleClickable;
    }

    /**
     * Adds a widget event handler.  The event handler will receive
     * all events raised by this widget.
     *
     * @param eventHandler A pointer to the event handler.
     */

    inline void addWidgetEventHandler(CWidgetEventHandler *eventHandler)
    {
      m_widgetEventHandlers->addWidgetEventHandler(eventHandler);
    }

    /**
     * Remove a widget event handler.
     *
     * @param eventHandler A pointer to the event handler to remove.
     */

    inline void removeWidgetEventHandler(CWidgetEventHandler* eventHandler)
    {
      m_widgetEventHandlers->removeWidgetEventHandler(eventHandler);
    }

    /**
     * Enables or disables event firing for this widget.
     *
     * @param raises True to enable events, false to disable.
     */

    inline void setRaisesEvents(const bool raises)
    {
      raises ? m_widgetEventHandlers->enable() : m_widgetEventHandlers->disable();
    }

    /**
     * Disabled drawing of this widget. Widgets hidden using this method will still
     * be processed.
     */

    inline void disableDrawing(void)
    {
      m_flags.drawingEnabled = false;
    }

    /**
     * Enables drawing of this widget.
     */

    inline void enableDrawing(void)
    {
      m_flags.drawingEnabled = true;
    }

    /**
     * Sets the normal background color.
     *
     * @param color The new background color.
     */

    inline void setBackgroundColor(const nxgl_mxpixel_t color)
    {
      m_style.colors.background = color;
    }

     /**
     * Sets the background color for a selected widget.
     *
     * @param color The new selected background color.
     */

    inline void setSelectedBackgroundColor(const nxgl_mxpixel_t color)
    {
      m_style.colors.selectedBackground = color;
    }

   /**
     * Sets the shiny edge color.
     *
     * @param color The new shine edge color.
     */

    inline void setShineEdgeColor(const nxgl_mxpixel_t color)
    {
      m_style.colors.shineEdge = color;
    }

    /**
     * Sets the shadow edge color.
     *
     * @param color The new shadow edge color.
     */

    inline void setShadowEdgeColor(const nxgl_mxpixel_t color)
    {
      m_style.colors.shadowEdge = color;
    }

    /**
     * Sets the highlight color.
     *
     * @param color The new highlight color.
     */

    inline void setHighlightColor(const nxgl_mxpixel_t color)
    {
      m_style.colors.highlight = color;
    }

    /**
     * Sets the text color to use when the widget is disabled.
     *
     * @param color The new text color.
     */

    inline void setDisabledTextColor(const nxgl_mxpixel_t color)
    {
      m_style.colors.disabledText = color;
    }

    /**
     * Sets the text color to use when the widget is enabled.
     *
     * @param color The new text color.
     */

    inline void setEnabledTextColor(const nxgl_mxpixel_t color)
    {
      m_style.colors.enabledText = color;
    }

    /**
     * Sets the text color to use when the widget is highlighted or clicked.
     *
     * @param color The new selected text color.
     */

    inline void setSelectedTextColor(const nxgl_mxpixel_t color)
    {
      m_style.colors.selectedText = color;
    }

    /**
     * Sets the font.
     *
     * @param font A pointer to the font to use.
     */

    virtual void setFont(CNxFont *font);

    /**
     * Draws the visible regions of the widget and the widget's child widgets.
     */

    void redraw(void);

    /**
     * Erases the visible regions of the widget by redrawing the widgets
     * behind it.
     */

    void erase(void);

    /**
     * Enables the widget.
     *
     * @return True if the widget was enabled.
     */

    bool enable(void);

    /**
     * Disabled the widget.
     *
     * @return True if the widget was disabled.
     */

    bool disable(void);

    /**
     * Erases the widget, marks it as deleted, and moves it to the CNxWidget
     * deletion queue.  Widgets are automatically deleted by the framework and
     * should not be deleted externally.
     */

    void close(void);

    /**
     * Draws the widget and makes it visible.
     * Does not steal focus from other widgets.
     *
     * @return True if the widget was shown.
     * @see hide()
     */

    bool show(void);

    /**
     * Erases the widget and makes it invisible.
     * Does not re-assign focus to another widget.
     *
     * @return True if the widget was hidden.
     * @see show()
     */

    bool hide(void);

    /**
     * Click this widget at the supplied coordinates.  This should only be
     * overridden in subclasses if the default click behaviour needs to be changed.
     * If the subclassed widget should just respond to a standard click,
     * the onClick() method should be overridden instead.
     *
     * @param x X coordinate of the click.
     * @param y Y coordinate of the click.
     * @return True if the click was successful.
     */

    bool click(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Check if the click is a double-click.
     *
     * @param x X coordinate of the click.
     * @param y Y coordinate of the click.
     * @return True if the click is a double-click.
     */

    virtual bool isDoubleClick(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Double-click this widget at the supplied coordinates.  This
     * should only be overridden in subclasses if the default
     * double-click behaviour needs to be changed.  If the subclassed
     * widget should just respond to a standard double-click, the
     * onDoubleClick() method should be overridden instead.
     *
     * @param x X coordinate of the click.
     * @param y Y coordinate of the click.
     * @return True if the click was successful.
     */

    bool doubleClick(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Release this widget at the supplied coordinates.  This
     * should only be overridden in subclasses if the default
     * release behaviour needs to be changed.  If the subclassed
     * widget should just respond to a standard release, the
     * onRelease() method should be overridden instead.
     *
     * @param x X coordinate of the release.
     * @param y Y coordinate of the release.
     * @return True if the release was successful.
     */

    bool release(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Drag the widget to the supplied coordinates.
     *
     * @param x The x coordinate of the mouse.
     * @param y The y coordinate of the mouse.
     * @param vX The horizontal distance that the mouse was dragged.
     * @param vY The vertical distance that the mouse was dragged.
     * @return True if the drag was successful.
     */

    bool drag(nxgl_coord_t x, nxgl_coord_t y,
              nxgl_coord_t vX, nxgl_coord_t vY);

    /**
     * Send a keypress to the widget.
     *
     * @param key The keycode to send to the widget.
     * @return True if the keypress was processed.
     */

    bool keyPress(nxwidget_char_t key);

    /**
     * Send a cursor control event to the widget.
     *
     * @param control The cursor control code to send to the widget.
     * @return True if the cursor control was processed.
     */

    bool cursorControl(ECursorControl control);

    /**
     * Give the widget focus.
     *
     * @return True if the widget received focus correctly.
     */

    bool focus(void);

    /**
     * Remove focus from the widget.
     *
     * @return True if the widget lost focus correctly.
     */

    bool blur(void);

    /**
     * Move the widget to the new coordinates.
     * Co-ordinates are relative to the parent widget.
     *
     * @param x The new x coordinate.
     * @param y The new y coordinate.
     * @return True if the move was successful.
     */

    bool moveTo(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Resize the widget to the new dimensions.
     *
     * @param width The new width.
     * @param height The new height.
     * @return True if the resize was successful.
     */

    bool resize(nxgl_coord_t width, nxgl_coord_t height);

    /**
     * Resize and move the widget in one operation.
     * Only performs one redraw so it is faster than calling the
     * two separate functions.
     *
     * @param x The new x coordinate.
     * @param y The new y coordinate.
     * @param width The new width.
     * @param height The new height.
     * @return True if the widget was adjusted successfully.
     */

    bool changeDimensions(nxgl_coord_t x, nxgl_coord_t y,
                          nxgl_coord_t width, nxgl_coord_t height);

    /**
     * Raises the widget to the top of its parent's widget stack.
     *
     * @return True if the raise was successful.
     */

    bool raiseToTop(void);

    /**
     * Lowers the widget to the bottom of its parent's widget stack.
     *
     * @return True if the lower was successful.
     */

    bool lowerToBottom(void);

    /**
     * Raises the supplied widget to the top of this widget's child stack.
     * The supplied widget pointer must be a child of this widget.
     *
     * @param widget A pointer to the child widget to raise.
     * @return True if the raise was successful.
     */

    bool raiseWidgetToTop(CNxWidget *widget);

    /**
     * Lowers the supplied widget to the bottom of this widget's child stack.
     * The supplied widget pointer must be a child of this widget.
     *
     * @param widget A pointer to the child widget to lower.
     * @return True if the lower was successful.
     */

    bool lowerWidgetToBottom(CNxWidget *widget);

    /**
     * Moves the supplied child widget to the deletion queue.
     * For framework use only.
     *
     * @param widget A pointer to the child widget.
     */

    void moveChildToDeleteQueue(CNxWidget *widget);

    /**
     * Sets the supplied widget as the focused child.  The widget must
     * be a child of this widget.
     *
     * @param widget A pointer to the child widget.
     * @see getFocusedWidget()
     */

    void setFocusedWidget(CNxWidget *widget);

    /**
     * Checks if the supplied coordinates collide with this widget.
     *
     * @param x The x coordinate to check.
     * @param y The y coordinate to check.
     * @return True if a collision occurred.
     */

    bool checkCollision(nxgl_coord_t x, nxgl_coord_t y) const;

    /**
     * Checks if the supplied rectangle definition collides with this widget.
     *
     * @param x The x coordinate of the rectangle to check.
     * @param y The y coordinate of the rectangle to check.
     * @param width The width of the rectangle to check.
     * @param height The height of the rectangle to check.
     * @return True if a collision occurred.
     */

    bool checkCollision(nxgl_coord_t x, nxgl_coord_t y,
                        nxgl_coord_t width, nxgl_coord_t height) const;

    /**
     * Checks if the supplied widget collides with this widget.
     *
     * @param widget A pointer to another widget to check for collisions with.
     * @return True if a collision occurred.
     */

    bool checkCollision(CNxWidget *widget) const;

    /**
     * Invalidate the visible region cache for all widgets below the supplied
     * widget in this widget's child stack.  This will cause those widgets to
     *
     * recalculate their visible regions next time they try to draw themselves.
     * @param widget A pointer to a child widget.
     */

    void invalidateLowerWidgetsVisibleRectCache(CNxWidget *widget);

    /**
     * Adds a widget to this widget's child stack.  The widget is added to the
     * top of the stack.  Note that the widget can only be added if it is not
     * already a child of another widget.
     *
     * @param widget A pointer to the widget to add to the child list.
     * @see insertWidget()
     */

    void addWidget(CNxWidget *widget);

    /**
     * Inserts a widget into this widget's child stack at the bottom of the
     * stack.  Note that the widget can only be added if it is not already
     * a child of another widget.
     *
     * @param widget A pointer to the widget to add to the child list.
     * @see addWidget()
     */

    void insertWidget(CNxWidget *widget);

    /**
     * Set the widget's parent to the widget passed in as a parameter.
     * Called automatically when a widget is added as a child.
     *
     * @param parent A pointer to the parent widget.
     */

    inline void setParent(CNxWidget *parent)
    {
      m_parent = parent;
    }

    /**
     * Rebuild the list of this widget's visible regions
     */

    void cacheVisibleRects(void) const;

    /**
     * Mark this widget's visible region cache as invalid, and do the same
     * to its child widgets.
     */

    void invalidateVisibleRectCache(void);

    /**
     * Erase a child widget by drawing the widgets behind it.
     *
     * @param widget The child widget to erase.
     */

    void eraseWidget(CNxWidget *widget);

    /**
     * Redraw any visible regions of this widget that have become corrupted.
     *
     * @param invalidRects A list of corrupt regions.
     * @param sender A pointer to the widget that corrupted the regions.
     */

    void redrawDirty(TNxArray<CRect>* invalidRects, CNxWidget *sender);

    /**
     * Clips a rectangular region to the dimensions of this widget and its ancestors.
     *
     * @param rect The region that needs to be clipped.
     */

    void clipRectToHierarchy(CRect &rect) const;

    /**
     * Swaps the depth of the supplied child widget.
     *
     * @param widget A pointer to the child widget that needs to swap depths.
     * @return True if the swap was successful.
     */

    virtual bool swapWidgetDepth(CNxWidget *widget);

    /**
     * Swap the depth of this widget.
     *
     * @return True if the swap was successful.
     */

    bool swapDepth(void);

    /**
     * Delete this widget.  This should never be called in user code; widget
     * deletion is handled internally.
     */

    inline void destroy(void)
    {
      delete this;
    }

    /**
     * Remove this widget from the widget hierarchy.  Returns
     * responsibility for deleting the widget back to the developer.
     * Does not unregister the widget from the VBL system.
     * Does not erase the widget from the display.
     *
     * @return True if the widget was successfully removed.
     */

    bool remove(void);

    /**
     * Remove a child widget from the widget hierarchy.  Returns
     * responsibility for deleting the widget back to the developer.
     * Does not unregister the widget from the VBL system.
     * Does not erase the widget from the display.
     *
     * @param widget Pointer to the widget to remove from the hierarchy.
     * @return True if the widget was succesfully removed.
     */

    bool removeChild(CNxWidget *widget);

    /**
     * Run the widget modally.
     */

    void goModal(void);

    /**
     * Stop the widget running modally.
     */

    inline void stopModal(void)
    {
      m_flags.modal = false;
    }

    /**
     * Get the index of the specified child widget.
     *
     * @param widget The widget to get the index of.
     * @return The index of the widget.  -1 if the widget is not found.
     */

    const int getWidgetIndex(const CNxWidget *widget) const;

    /**
     * Get the child widget at the specified index.
     *
     * @param index Index of the child to retrieve.
     * @return Pointer to the child at the specified index.
     */

    const CNxWidget *getChild(int index) const;

    /**
     * Get the number of child widgets.
     *
     * @return The number of child widgets belonging to this widget.
     */

    const int getChildCount(void) const
    {
      return m_children.size();
    }

    /**
     * Get a pointer to the cache of visible rects.
     *
     * @return A pointer to the cache of visible rects.
     */

    inline CRectCache *getCRectCache(void) const
    {
      return m_rectCache;
    }

    /**
     * Sets the border size.  The border cannot be drawn over in the
     * drawContents() method.
     *
     * @param borderSize The new border size.
     */

    void setBorderSize(const WidgetBorderSize &borderSize);
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CNXWIDGET_HXX

