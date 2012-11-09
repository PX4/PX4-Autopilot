/********************************************************************************************
 * NxWidgets/nxwm/src/chexcalculator.cxx
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
 ********************************************************************************************/

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include <cstdio>
#include <debug.h>

#include "cwidgetcontrol.hxx"

#include "nxwmconfig.hxx"
#include "nxwmglyphs.hxx"
#include "chexcalculator.hxx"

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

/********************************************************************************************
 * Private Types
 ********************************************************************************************/

/********************************************************************************************
 * Private Data
 ********************************************************************************************/

namespace NxWM
{
  /**
   * This enumeration value describes a key
   */

  enum EKeyType
  {
    KEY_NONE = 0,    // Used to represent no pending operation

    // Values: {0-9, A-F}

    KEY_VALUE,       // Key is a value

    // Unary operators

    KEY_NOT,         // 1's complement
    KEY_NEGATE,      // 2's complement

    // Binary operators

    KEY_XOR,         // Exclusive OR
    KEY_DIVIDE,      // Division
    KEY_RSH,         // Right shift
    KEY_LSH,         // Left shift
    KEY_MULTIPLY,    // Multiplication
    KEY_AND,         // Bit-wise AND
    KEY_OR,          // Bit-wise OR
    KEY_MINUS,       // Subtraction
    KEY_PLUS,        // Additions

    // Special operations
 
    KEY_EQUAL,       // Equal/Enter key
 
    KEY_DECIMAL,     // Decimal mode
    KEY_HEXADECIMAL, // Hexadecimal mode

    KEY_MRECALL,   // Recall from memory
    KEY_MPLUS,     // Add to memory
    KEY_MMINUS,    // Subtract from memory

    KEY_MCLR,      // Clear memory
    KEY_CLRENTRY,  // Clear entry
    KEY_CLR        // Clear all
  };

  /**
   * This structure value describes a key
   */

  struct SKeyDesc
  {
    uint16_t hexMode : 1;  // Key applies in hex mode
    uint16_t decMode : 1;  // Key applies in decimal mode
    uint16_t keyType : 5;  // Describes the key (see enum EKeyType)
    uint16_t value   : 4;  // Value (if the key has an associated value)
  };

  /**
   * This array provides the possible labels for each key
   */

  static FAR const char *g_labels[NXWM_HEXCALCULATOR_NCOLUMNS*NXWM_HEXCALCULATOR_NROWS] =
  {
     "MR",  "M+",  "M-",  "MC", "CE", "C",
     "A",   "B",   "C",   "D",  "E",  "F",
     "NOT", "XOR", "7",   "8",  "9",  "/",
     "RSH", "LSH", "4",   "5",  "6",  "*",
     "AND", "OR",  "1",   "2",  "3",  "-",
     "DEC", "HEX", "+/-", "0",  "=",  "+"
  };

  /**
   * This array manages the behavior when each key is pressed
   */

  static struct SKeyDesc g_keyDesc[NXWM_HEXCALCULATOR_NCOLUMNS*NXWM_HEXCALCULATOR_NROWS] =
  {
    {1, 1, KEY_MRECALL,     0},
    {1, 1, KEY_MPLUS,       0},
    {1, 1, KEY_MMINUS,      0},
    {1, 1, KEY_MCLR,        0},
    {1, 1, KEY_CLRENTRY,    0},
    {1, 1, KEY_CLR,         0},

    {1, 0, KEY_VALUE,       10},
    {1, 0, KEY_VALUE,       11},
    {1, 0, KEY_VALUE,       12},
    {1, 0, KEY_VALUE,       13},
    {1, 0, KEY_VALUE,       14},
    {1, 0, KEY_VALUE,       15},

    {1, 0, KEY_NOT,         0},
    {1, 0, KEY_XOR,         0},
    {1, 1, KEY_VALUE,       7},
    {1, 1, KEY_VALUE,       8},
    {1, 1, KEY_VALUE,       9},
    {1, 1, KEY_DIVIDE,      0},

    {1, 0, KEY_RSH,         0},
    {1, 0, KEY_LSH,         0},
    {1, 1, KEY_VALUE,       4},
    {1, 1, KEY_VALUE,       5},
    {1, 1, KEY_VALUE,       6},
    {1, 1, KEY_MULTIPLY,    0},

    {1, 0, KEY_AND,         0},
    {1, 0, KEY_OR,          0},
    {1, 1, KEY_VALUE,       1},
    {1, 1, KEY_VALUE,       2},
    {1, 1, KEY_VALUE,       3},
    {1, 1, KEY_MINUS,       0},

    {1, 0, KEY_DECIMAL,     0},
    {0, 1, KEY_HEXADECIMAL, 0},
    {1, 1, KEY_NEGATE,      0},
    {1, 1, KEY_VALUE,       0},
    {1, 1, KEY_EQUAL,       0},
    {1, 1, KEY_PLUS,        0}
  };
}

/********************************************************************************************
 * Private Functions
 ********************************************************************************************/

/********************************************************************************************
 * CHexCalculator Method Implementations
 ********************************************************************************************/

extern const struct NXWidgets::SRlePaletteBitmap CONFIG_NXWM_HEXCALCULATOR_ICON;

using namespace NxWM;

/**
 * CHexCalculator constructor
 *
 * @param window.  The application window
 */

CHexCalculator::CHexCalculator(CTaskbar *taskbar, CApplicationWindow *window)
{
  // Save the constructor data

  m_taskbar = taskbar;
  m_window  = window;

  // Nullify widgets that will be instantiated when the window is started

  m_keypad  = (NXWidgets::CButtonArray *)0;
  m_text    = (NXWidgets::CLabel       *)0;
  m_font    = (NXWidgets::CNxFont      *)0;

  // Reset other values

  m_accum          = 0;                  // The accumulator is initially zero
  m_memory         = 0;                  // No value in memory
  m_high.operation = (uint8_t)KEY_NONE;  // No pending high precedence operation
  m_high.value     = 0;
  m_low.operation  = (uint8_t)KEY_NONE;  // No pending high precedence operation
  m_low.value      = 0;
  m_hexMode        = false;              // Decimal mode
  m_result         = false;              // Accumulator does not hot a result

  // Add our personalized window label

  NXWidgets::CNxString myName = getName();
  window->setWindowLabel(myName);

  // Add our callbacks with the application window

  window->registerCallbacks(static_cast<IApplicationCallback *>(this));

  // Set the geometry of the calculator

  setGeometry();
}

/**
 * CHexCalculator destructor
 *
 * @param window.  The application window
 */

CHexCalculator::~CHexCalculator(void)
{
  // Destroy widgets 

  if (m_text)
    {
      delete m_text;
    }

  if (m_keypad)
    {
      delete m_keypad;
    }

  if (m_font)
    {
      delete m_font;
    }

  // Although we didn't create it, we are responsible for deleting the
  // application window

  delete m_window;
}

/**
 * Each implementation of IApplication must provide a method to recover
 * the contained CApplicationWindow instance.
 */

IApplicationWindow *CHexCalculator::getWindow(void) const
{
  return static_cast<IApplicationWindow*>(m_window);
}

/**
 * Get the icon associated with the application
 *
 * @return An instance if IBitmap that may be used to rend the
 *   application's icon.  This is an new IBitmap instance that must
 *   be deleted by the caller when it is no long needed.
 */

NXWidgets::IBitmap *CHexCalculator::getIcon(void)
{
  NXWidgets::CRlePaletteBitmap *bitmap =
    new NXWidgets::CRlePaletteBitmap(&CONFIG_NXWM_HEXCALCULATOR_ICON);

  return bitmap;
}

/**
 * Get the name string associated with the application
 *
 * @return A copy if CNxString that contains the name of the application.
 */

NXWidgets::CNxString CHexCalculator::getName(void)
{
  return NXWidgets::CNxString("Hex Calculator");
}

/**
 * Start the application (perhaps in the minimized state).
 *
 * @return True if the application was successfully started.
 */

bool CHexCalculator::run(void)
{
  // Create the widgets (if we have not already done so)

  if (!m_keypad)
    {
      // Create the widgets

    if (!createCalculator())
      {
        gdbg("ERROR: Failed to create widgets\n");
        return false;
      }

      // Apply initial labels

      labelKeypad();
    }

  return true;
}

/**
 * Stop the application.
 */

void CHexCalculator::stop(void)
{
  // Just disable further drawing

  m_keypad->disableDrawing();
  m_keypad->setRaisesEvents(false);

  m_text->disableDrawing();
}

/**
 * Destroy the application and free all of its resources.  This method
 * will initiate blocking of messages from the NX server.  The server
 * will flush the window message queue and reply with the blocked
 * message.  When the block message is received by CWindowMessenger,
 * it will send the destroy message to the start window task which
 * will, finally, safely delete the application.
 */

void CHexCalculator::destroy(void)
{
  // Make sure that the widgets are stopped

  stop();

  // Block any further window messages

  m_window->block(this);
}

/**
 * The application window is hidden (either it is minimized or it is
 * maximized, but not at the top of the hierarchy
 */

void CHexCalculator::hide(void)
{
  // Disable drawing and events

  stop();
}

/**
 * Redraw the entire window.  The application has been maximized or
 * otherwise moved to the top of the hierarchy.  This method is call from
 * CTaskbar when the application window must be displayed
 */

void CHexCalculator::redraw(void)
{
  // Get the widget control associated with the application window

  NXWidgets::CWidgetControl *control = m_window->getWidgetControl();

  // Get the CCGraphicsPort instance for this window

  NXWidgets::CGraphicsPort *port = control->getGraphicsPort();

  // Fill the entire window with the background color

  port->drawFilledRect(0, 0, m_windowSize.w, m_windowSize.h,
                       CONFIG_NXWM_HEXCALCULATOR_BACKGROUNDCOLOR);

  // Enable and redraw widgets

  m_keypad->enableDrawing();
  m_keypad->redraw();
  m_keypad->setRaisesEvents(true);

  m_text->enableDrawing();
  m_text->redraw();
}

/**
 * Report of this is a "normal" window or a full screen window.  The
 * primary purpose of this method is so that window manager will know
 * whether or not it show draw the task bar.
 *
 * @return True if this is a full screen window.
 */

bool CHexCalculator::isFullScreen(void) const
{
  return m_window->isFullScreen();
}

/**
 * Select the geometry of the calculator given the current window size.
 */

void CHexCalculator::setGeometry(void)
{
  // Recover the NXTK window instance contained in the application window

  NXWidgets::INxWindow *window = m_window->getWindow();

  // Get the size of the window

  (void)window->getSize(&m_windowSize);

  // Pick a height and width of a button to fill the entire window.
  // For the height, we will assume that the text window is 1.5 times
  // as high as a button

  m_buttonSize.w = m_windowSize.w / NXWM_HEXCALCULATOR_NCOLUMNS;
  m_buttonSize.h = (m_windowSize.h << 1) / (2 * NXWM_HEXCALCULATOR_NROWS + 3);

  // Limit aspect ratio.  (1) Button should no be taller than it is wide,
  // (2) Button should be no more than twice as wide as it is tall.

  if (m_buttonSize.h > m_buttonSize.w)
    {
      m_buttonSize.h = m_buttonSize.w;
    }
  else if (m_buttonSize.w > (m_buttonSize.h << 1))
    {
      m_buttonSize.w = (m_buttonSize.h << 1);
    }

  // Get the size of the entry calculator m_keypad

  m_keypadSize.w = NXWM_HEXCALCULATOR_NCOLUMNS * m_buttonSize.w;
  m_keypadSize.h = NXWM_HEXCALCULATOR_NROWS * m_buttonSize.h;

  // Get the size of the text box.  Same width as the m_keypad

  m_textSize.w   = m_keypadSize.w;
  m_textSize.h   = m_windowSize.h - m_keypadSize.h;

  // Limit the height of the text box to twice the height of a button.

  if (m_textSize.h > (m_buttonSize.h << 1))
    {
      m_textSize.h = (m_buttonSize.h << 1);
    }

  // Pick an X/Y position such that the m_keypad+textbox will be centered
  // in the display

  struct nxgl_point_s calculatorPos;
  calculatorPos.x = (m_windowSize.w - m_keypadSize.w) >> 1;
  calculatorPos.y = (m_windowSize.h - m_textSize.h - m_keypadSize.h) >> 1;

  // Now pick the set of the text box and the m_keypad

  m_textPos.x = calculatorPos.x;
  m_textPos.y = calculatorPos.y;

  m_keypadPos.x = calculatorPos.x;
  m_keypadPos.y = calculatorPos.y + m_textSize.h;
}

/**
 * Create the calculator widgets.  Only start as part of the applicaiton
 * start method.
 */

bool CHexCalculator::createCalculator(void)
{
  // Select a font for the calculator

  m_font = new NXWidgets::CNxFont((nx_fontid_e)CONFIG_NXWM_HEXCALCULATOR_FONTID,
                                  CONFIG_NXWM_DEFAULT_FONTCOLOR,
                                  CONFIG_NXWM_TRANSPARENT_COLOR);
  if (!m_font)
    {
      gdbg("ERROR failed to create font\n");
      return false;
    }

  // Get the widget control associated with the application window

  NXWidgets::CWidgetControl *control = m_window->getWidgetControl();

  // Create the button array

  m_keypad = new NXWidgets::CButtonArray(control,
                                         m_keypadPos.x, m_keypadPos.y,
                                         NXWM_HEXCALCULATOR_NCOLUMNS,
                                         NXWM_HEXCALCULATOR_NROWS,
                                         m_buttonSize.w, m_buttonSize.h);
  if (!m_keypad)
    {
      gdbg("ERROR: Failed to create CButtonArray\n");
      return false;
    }

  // Disable drawing and events until we are asked to redraw the window

  m_keypad->disableDrawing();
  m_keypad->setRaisesEvents(false);

  // Select the font

  m_keypad->setFont(m_font);

  // Register to receive events from the keypad

  m_keypad->addWidgetEventHandler(this);

  // Create a label to show the accumulator.  A simple label is used
  // because the power of a text box is un-necessary in this application.

  m_text = new NXWidgets::CLabel(control,
                                 m_textPos.x, m_textPos.y,
                                 m_textSize.w, m_textSize.h,
                                 "0");
  if (!m_text)
    {
      gdbg("ERROR: Failed to create CLabel\n");
      return false;
    }

  // Align text on the left

  m_text->setTextAlignmentHoriz(NXWidgets::CLabel::TEXT_ALIGNMENT_HORIZ_RIGHT);

  // Disable drawing and events until we are asked to redraw the window

  m_text->disableDrawing();
  m_text->setRaisesEvents(false);

  // Select the font

  m_text->setFont(m_font);
  return true;
}

/**
 * Applies labels to the keys.
 */

void CHexCalculator::labelKeypad(void)
{
  // Make sure that drawing is disabled from this operation

  bool isEnabled = m_keypad->isDrawingEnabled();
  m_keypad->disableDrawing();

  // Add the labels to each button.

  for (int j = 0; j < NXWM_HEXCALCULATOR_NROWS; j++)
    {
      for (int i = 0; i < NXWM_HEXCALCULATOR_NCOLUMNS; i++)
        {
          int index = j * NXWM_HEXCALCULATOR_NCOLUMNS + i;

          // What mode are we in?  Does the button label appear in this mode?

          FAR const char *label;
          if ((m_hexMode && g_keyDesc[index].hexMode) ||
              (!m_hexMode && g_keyDesc[index].decMode))
            {
              label = g_labels[index];
            }
          else
            {
              label = "";
            }
          
          // Set the text in the label

          m_keypad->setText(i, j, label);
        }
    }

  // Then redraw the display

  if (isEnabled)
    {
      m_keypad->enableDrawing();
      m_keypad->redraw();
    }
}

/**
 * Evaluate a binary operation.  The result is left in m_accum.
 *
 * @param value1. The first value
 * @param value2. The second value
 *
 * @return The result of the operation
 */

int64_t CHexCalculator::evaluateBinaryOperation(uint8_t operation, int64_t value1, int64_t value2)
{
  switch (operation)
    {
      case KEY_NONE:        // Do nothing if there is no pending operation
        return 0;

      case KEY_XOR:         // Exclusive OR
        return value1 ^ value2;

      case KEY_DIVIDE:      // Division
        return value1 / value2;

      case KEY_RSH:         // Right shift
        return value1 >> value2;

      case KEY_LSH:         // Left shift
        return value1 << value2;

      case KEY_MULTIPLY:    // Multiplication
        return value1 * value2;

      case KEY_AND:         // Bit-wise AND
        return value1 & value2;

      case KEY_OR:          // Bit-wise OR
        return value1 | value2;

      case KEY_MINUS:       // Subtraction
        return value1 - value2;

      case KEY_PLUS:        // Additions
        return value1 + value2;

      default:
        gdbg("ERROR: Unexpected pending operation %d\n", operation);
        return 0;
      }
}

/**
 * Show the current value of the accumulator.
 */

void CHexCalculator::updateText(void)
{
  char buffer[24];

  if (m_hexMode)
    {
      std::snprintf(buffer, 24, "%16llX", m_accum);
    }
  else
    {
      std::snprintf(buffer, 24, "%lld", m_accum);
    }

  // setText will perform the redraw as well

  m_text->setText(buffer);
}

/**
 * Called when the window minimize button is pressed.
 */

void CHexCalculator::minimize(void)
{
  m_taskbar->minimizeApplication(static_cast<IApplication*>(this));
}

/**
 * Called when the window close button is pressed.
 */

void CHexCalculator::close(void)
{
  m_taskbar->stopApplication(static_cast<IApplication*>(this));
}

/**
 * Handle a widget action event.  For CButtonArray, this is a button pre-
 * release event.
 *
 * @param e The event data.
 */

void CHexCalculator::handleActionEvent(const NXWidgets::CWidgetEventArgs &e)
{
  // A button should now be clicked

  int column;
  int row;

  if (m_keypad->isButtonClicked(column, row))
    {
      // Handle the key according to its type

      int index = row * NXWM_HEXCALCULATOR_NCOLUMNS + column;

      // First, make sure that the keypress is valid in this mode

      if ((m_hexMode && !g_keyDesc[index].hexMode) ||
          (!m_hexMode && !g_keyDesc[index].decMode))
        {
          // Ignore the key in this mode

          return;
        }

      // Process the keypress

      bool result = false;
      switch (g_keyDesc[index].keyType)
        {
          // Values: {0-9, A-F}

          case KEY_VALUE:       // Key is a value
            {
              // If the accumulator current holds the result of a previous
              // operation, then start fresh

              if (m_result)
                {
                  m_accum = (uint64_t)g_keyDesc[index].value;
                }

              // Otherwise, add the new value to the accumulator.  The way
              // in which it is added depends on the mode

              else if (m_hexMode)
                {
                  m_accum <<= 4;
                  m_accum |= (uint64_t)g_keyDesc[index].value;
                }
              else
                {
                  m_accum *= 10;
                  m_accum += (uint64_t)g_keyDesc[index].value;
                }
              updateText();
            }
            break;

          // Unary operators

          case KEY_NOT:         // 1's complement
            {
              m_accum = ~m_accum;
              updateText();
            }
            break;

          case KEY_NEGATE:      // 2's complement
            {
              m_accum = -m_accum;
              updateText();
            }
            break;

          // Low precedence Binary operators

          case KEY_XOR:         // Exclusive OR
          case KEY_OR:          // Bit-wise OR
          case KEY_MINUS:       // Subtraction
          case KEY_PLUS:        // Additions
            {
              // Is there a high precedence operation?

              if (m_high.operation != (uint8_t)KEY_NONE)
                {
                  m_accum          = evaluateBinaryOperation(m_high.operation, m_high.value, m_accum);
                  m_high.operation = (uint8_t)KEY_NONE;
                  m_high.value     = 0;
                }

              // Is there a pending low precedence operation?

              if (m_low.operation != (uint8_t)KEY_NONE)
                {
                  m_accum  = evaluateBinaryOperation(m_low.operation, m_low.value, m_accum);
                }

              // Save the new low precedence operation

              m_low.operation = (uint8_t) g_keyDesc[index].keyType;
              m_low.value     = m_accum;

              // Update the display with the value in the accumulator, but
              // then clear the accumulator in preparation for the next input

              updateText();
              m_accum = 0;
            }
            break;

          // High precedence Binary operators

          case KEY_DIVIDE:      // Division
          case KEY_RSH:         // Right shift
          case KEY_LSH:         // Left shift
          case KEY_MULTIPLY:    // Multiplication
          case KEY_AND:         // Bit-wise AND
            {
              // Is there a high precedence operation?

              if (m_high.operation != (uint8_t)KEY_NONE)
                {
                  m_accum = evaluateBinaryOperation(m_high.operation, m_high.value, m_accum);
                }

              // Save the new high precedence operation

              m_high.operation = (uint8_t) g_keyDesc[index].keyType;
              m_high.value     = m_accum;

              // Update the display with the value in the accumulator, but
              // then clear the accumulator in preparation for the next input

              updateText();
              m_accum = 0;
            }
            break;

          // Special operations
 
          case KEY_EQUAL:       // Equal/Enter key
            {
              // Is there a high precedence operation?

              if (m_high.operation != (uint8_t)KEY_NONE)
                {
                  m_accum          = evaluateBinaryOperation(m_high.operation, m_high.value, m_accum);
                  m_high.operation = (uint8_t)KEY_NONE;
                  m_high.value     = 0;
                }

              // Is there a pending low precedence operation?

              if (m_low.operation != (uint8_t)KEY_NONE)
                {
                  m_accum         = evaluateBinaryOperation(m_low.operation, m_low.value, m_accum);
                  m_low.operation = (uint8_t)KEY_NONE;
                  m_low.value     = 0;
                }

              // Update the display with the value in the accumulator.  Flag that
              // this is a result meaning that (1) it can be used as an accumulator
              // for the next operation, but new input values cannot be appended to it.

              updateText();
              result = true;
            }
            break;
 
          case KEY_DECIMAL:     // Decimal mode
            {
              if (m_hexMode)
                {
                  m_hexMode = false;
                  labelKeypad();
                  updateText();
                }
            }
            break;

          case KEY_HEXADECIMAL: // Hexadecimal mode
            {
              if (!m_hexMode)
                {
                  m_hexMode = true;
                  labelKeypad();
                  updateText();
                }
            }
            break;

          case KEY_MRECALL:   // Recall from memory
            {
               m_accum = m_memory;
               updateText();
            }
            break;

          case KEY_MPLUS:     // Add to memory
            {
               m_memory += m_accum;
            }
            break;

          case KEY_MMINUS:    // Subtract from memory
            {
               m_memory -= m_accum;
            }
            break;

          case KEY_MCLR:      // Clear memory
            {
               m_memory = 0;
            }
            break;

          case KEY_CLRENTRY:  // Clear entry
            {
               m_accum = 0;
               updateText();
            }
            break;

          case KEY_CLR:       // Clear all
            {
               m_accum          = 0;
               m_high.operation = (uint8_t)KEY_NONE;
               m_high.value     = 0;
               m_low.operation  = (uint8_t)KEY_NONE;
               m_low.value      = 0;
               updateText();
            }
            break;

          case KEY_NONE:
          default:
            gdbg("ERROR: Invalid key type %d\n", g_keyDesc[index].keyType);
            break;
        }

      // Remember if the accumulator contains a special reault

      m_result = result;
    }
}

/**
 * CHexCalculatorFactory Constructor
 *
 * @param taskbar.  The taskbar instance used to terminate the console
 */

CHexCalculatorFactory::CHexCalculatorFactory(CTaskbar *taskbar)
{
  m_taskbar = taskbar;
}

/**
 * Create a new instance of an CHexCalculator (as IApplication).
 */

IApplication *CHexCalculatorFactory::create(void)
{
  // Call CTaskBar::openFullScreenWindow to create a application window for
  // the NxConsole application

  CApplicationWindow *window = m_taskbar->openApplicationWindow();
  if (!window)
    {
      gdbg("ERROR: Failed to create CApplicationWindow\n");
      return (IApplication *)0;
    }

  // Open the window (it is hot in here)

  if (!window->open())
    {
      gdbg("ERROR: Failed to open CApplicationWindow\n");
      delete window;
      return (IApplication *)0;
    }

  // Instantiate the application, providing the window to the application's
  // constructor

  CHexCalculator *hexCalculator = new CHexCalculator(m_taskbar, window);
  if (!hexCalculator)
    {
      gdbg("ERROR: Failed to instantiate CHexCalculator\n");
      delete window;
      return (IApplication *)0;
    }

  return static_cast<IApplication*>(hexCalculator);
}

/**
 * Get the icon associated with the application
 *
 * @return An instance if IBitmap that may be used to rend the
 *   application's icon.  This is an new IBitmap instance that must
 *   be deleted by the caller when it is no long needed.
 */

NXWidgets::IBitmap *CHexCalculatorFactory::getIcon(void)
{
  NXWidgets::CRlePaletteBitmap *bitmap =
    new NXWidgets::CRlePaletteBitmap(&CONFIG_NXWM_HEXCALCULATOR_ICON);

  return bitmap;
}
