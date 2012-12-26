/****************************************************************************
 * drivers/usbhost/usbhost_hidkbd.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>
#include <semaphore.h>
#include <time.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/hid.h>

#ifdef CONFIG_HIDKBD_ENCODED
#  include <nuttx/streams.h>
#  include <nuttx/input/kbd_codec.h>
#endif

/* Don't compile if prerequisites are not met */

#if defined(CONFIG_USBHOST) && !defined(CONFIG_USBHOST_INT_DISABLE) && CONFIG_NFILE_DESCRIPTORS > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* This determines how often the USB keyboard will be polled in units of
 * of microseconds.  The default is 100MS.
 */

#ifndef CONFIG_HIDKBD_POLLUSEC
#  define CONFIG_HIDKBD_POLLUSEC (100*1000)
#endif

/* Worker thread is needed, unfortunately, to handle some cornercase failure
 * conditions.  This is kind of wasteful and begs for a re-design.
 */

#ifndef CONFIG_SCHED_WORKQUEUE
#  warning "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

/* Signals must not be disabled as they are needed by usleep.  Need to have
 * CONFIG_DISABLE_SIGNALS=n
 */

#ifdef CONFIG_DISABLE_SIGNALS
#  warning "Signal support is required (CONFIG_DISABLE_SIGNALS)"
#endif

/* Provide some default values for other configuration settings */

#ifndef CONFIG_HIDKBD_DEFPRIO
#  define CONFIG_HIDKBD_DEFPRIO 50
#endif

#ifndef CONFIG_HIDKBD_STACKSIZE
#  define CONFIG_HIDKBD_STACKSIZE 1024
#endif

#ifndef CONFIG_HIDKBD_BUFSIZE
#  define CONFIG_HIDKBD_BUFSIZE 64
#endif

#ifndef CONFIG_HIDKBD_NPOLLWAITERS
#  define CONFIG_HIDKBD_NPOLLWAITERS 2
#endif

/* The default is to support scancode mapping for the standard 104 key
 * keyboard.  Setting CONFIG_HIDKBD_RAWSCANCODES will disable all scancode
 * mapping; Setting CONFIG_HIDKBD_ALLSCANCODES will enable mapping of all
 * scancodes; 
 */

#ifndef CONFIG_HIDKBD_RAWSCANCODES
#  ifdef CONFIG_HIDKBD_ALLSCANCODES
#    define USBHID_NUMSCANCODES (USBHID_KBDUSE_MAX+1)
#  else
#    define USBHID_NUMSCANCODES 104
#  endif
#endif

/* If we are using raw scancodes, then we cannot support encoding of
 * special characters either.
 */

#ifdef CONFIG_HIDKBD_RAWSCANCODES
#  undef CONFIG_HIDKBD_ENCODED
#endif

/* Driver support ***********************************************************/
/* This format is used to construct the /dev/kbd[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT          "/dev/kbd%c"
#define DEV_NAMELEN         11

/* Used in usbhost_cfgdesc() */

#define USBHOST_IFFOUND     0x01 /* Required I/F descriptor found */
#define USBHOST_EPINFOUND   0x02 /* Required interrupt IN EP descriptor found */
#define USBHOST_EPOUTFOUND  0x04 /* Optional interrupt OUT EP descriptor found */
#define USBHOST_RQDFOUND    (USBHOST_IFFOUND|USBHOST_EPINFOUND)
#define USBHOST_ALLFOUND    (USBHOST_RQDFOUND|USBHOST_EPOUTFOUND)

#define USBHOST_MAX_CREFS   0x7fff

/* Debug ********************************************************************/
/* Both CONFIG_DEBUG_INPUT and CONFIG_DEBUG_USB could apply to this file.
 * We assume here that CONFIG_DEBUG_INPUT might be enabled separately, but
 * CONFIG_DEBUG_USB implies both.
 */

#ifndef CONFIG_DEBUG_INPUT
#  undef  idbg
#  define idbg    udbg
#  undef  illdbg
#  define illdbg  ulldbg
#  undef  ivdbg
#  define ivdbg   uvdbg
#  undef  illvdbg
#  define illvdbg ullvdbg
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure contains the internal, private state of the USB host
 * keyboard storage class.
 */

struct usbhost_state_s
{
  /* This is the externally visible portion of the state */

  struct usbhost_class_s  class;

  /* This is an instance of the USB host driver bound to this class instance */

  struct usbhost_driver_s *drvr;

  /* The remainder of the fields are provide o the keyboard class driver */
  
  char                    devchar;      /* Character identifying the /dev/kbd[n] device */
  volatile bool           disconnected; /* TRUE: Device has been disconnected */
  volatile bool           polling;      /* TRUE: Poll thread is running */
  volatile bool           open;         /* TRUE: The keyboard device is open */
  volatile bool           waiting;      /* TRUE: waiting for keyboard data */
  uint8_t                 ifno;         /* Interface number */
  int16_t                 crefs;        /* Reference count on the driver instance */
  sem_t                   exclsem;      /* Used to maintain mutual exclusive access */
  sem_t                   waitsem;      /* Used to wait for keyboard data */
  FAR uint8_t            *tbuffer;      /* The allocated transfer buffer */
  size_t                  tbuflen;      /* Size of the allocated transfer buffer */
  pid_t                   pollpid;      /* PID of the poll task */
  struct work_s           work;         /* For cornercase error handling by the worker thread */

  /* Endpoints:
   * EP0 (Control):
   * - Receiving and responding to requests for USB control and class data.
   * - IN data when polled by the HID class driver (Get_Report)
   * - OUT data from the host.
   * EP Interrupt IN:
   * - Receiving asynchronous (unrequested) IN data from the device.
   * EP Interrrupt OUT (optional):
   * - Transmitting low latency OUT data to the device.
   * - If not present, EP0 used.
   */

  usbhost_ep_t            epin;         /* Interrupt IN endpoint */
  usbhost_ep_t            epout;        /* Optional interrupt OUT endpoint */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

#ifndef CONFIG_DISABLE_POLL
  struct pollfd *fds[CONFIG_HIDKBD_NPOLLWAITERS];
#endif

  /* Buffer used to collect and buffer incoming keyboard characters */

  volatile uint16_t       headndx;      /* Buffer head index */
  volatile uint16_t       tailndx;      /* Buffer tail index */
  uint8_t                 kbdbuffer[CONFIG_HIDKBD_BUFSIZE];
};

/* This type is used for encoding special characters */

#ifdef CONFIG_HIDKBD_ENCODED
struct usbhost_outstream_s
{
  struct lib_outstream_s stream;
  FAR struct usbhost_state_s *priv;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Semaphores */

static void usbhost_takesem(sem_t *sem);
#define usbhost_givesem(s) sem_post(s);

/* Polling support */

#ifndef CONFIG_DISABLE_POLL
static void usbhost_pollnotify(FAR struct usbhost_state_s *dev);
#else
#  define usbhost_pollnotify(dev)
#endif

/* Memory allocation services */

static inline FAR struct usbhost_state_s *usbhost_allocclass(void);
static inline void usbhost_freeclass(FAR struct usbhost_state_s *class);

/* Device name management */

static int usbhost_allocdevno(FAR struct usbhost_state_s *priv);
static void usbhost_freedevno(FAR struct usbhost_state_s *priv);
static inline void usbhost_mkdevname(FAR struct usbhost_state_s *priv, char *devname);

/* Keyboard polling thread */

static void usbhost_destroy(FAR void *arg);
static void usbhost_putbuffer(FAR struct usbhost_state_s *priv, uint8_t keycode);
#ifdef CONFIG_HIDKBD_ENCODED
static void usbhost_putstream(FAR struct lib_outstream_s *this, int ch);
#endif
static inline uint8_t usbhost_mapscancode(uint8_t scancode, uint8_t modifier);
#ifdef CONFIG_HIDKBD_ENCODED
static inline void usbhost_encodescancode(FAR struct usbhost_state_s *priv,
                                          uint8_t scancode, uint8_t modifier);
#endif
static int usbhost_kbdpoll(int argc, char *argv[]);

/* Helpers for usbhost_connect() */

static inline int usbhost_cfgdesc(FAR struct usbhost_state_s *priv,
                                  FAR const uint8_t *configdesc, int desclen,
                                  uint8_t funcaddr);
static inline int usbhost_devinit(FAR struct usbhost_state_s *priv);

/* (Little Endian) Data helpers */

static inline uint16_t usbhost_getle16(const uint8_t *val);
static inline void usbhost_putle16(uint8_t *dest, uint16_t val);
static inline uint32_t usbhost_getle32(const uint8_t *val);
#if 0 /* Not used */
static void usbhost_putle32(uint8_t *dest, uint32_t val);
#endif

/* Transfer descriptor memory management */

static inline int usbhost_tdalloc(FAR struct usbhost_state_s *priv);
static inline int usbhost_tdfree(FAR struct usbhost_state_s *priv);

/* struct usbhost_registry_s methods */
 
static struct usbhost_class_s *usbhost_create(FAR struct usbhost_driver_s *drvr,
                                              FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int usbhost_connect(FAR struct usbhost_class_s *class,
                           FAR const uint8_t *configdesc, int desclen,
                           uint8_t funcaddr);
static int usbhost_disconnected(FAR struct usbhost_class_s *class);

/* Driver methods.  We export the keyboard as a standard character driver */

static int usbhost_open(FAR struct file *filep);
static int usbhost_close(FAR struct file *filep);
static ssize_t usbhost_read(FAR struct file *filep,
                            FAR char *buffer, size_t len);
static ssize_t usbhost_write(FAR struct file *filep,
                             FAR const char *buffer, size_t len);
#ifndef CONFIG_DISABLE_POLL
static int usbhost_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure provides the registry entry ID informatino that will  be 
 * used to associate the USB host keyboard class driver to a connected USB
 * device.
 */

static const const struct usbhost_id_s g_id =
{
  USB_CLASS_HID,            /* base     */
  USBHID_SUBCLASS_BOOTIF,   /* subclass */
  USBHID_PROTOCOL_KEYBOARD, /* proto    */
  0,                        /* vid      */
  0                         /* pid      */
};

/* This is the USB host storage class's registry entry */

static struct usbhost_registry_s g_skeleton =
{
  NULL,                    /* flink     */
  usbhost_create,          /* create    */
  1,                       /* nids      */
  &g_id                    /* id[]      */
};

static const struct file_operations usbhost_fops =
{
  usbhost_open,            /* open      */
  usbhost_close,           /* close     */
  usbhost_read,            /* read      */
  usbhost_write,           /* write     */
  0,                       /* seek      */
  0                        /* ioctl     */
#ifndef CONFIG_DISABLE_POLL
  , usbhost_poll           /* poll      */
#endif
};

/* This is a bitmap that is used to allocate device names /dev/kbda-z. */

static uint32_t g_devinuse;

/* The following are used to managed the class creation operation */

static sem_t                   g_exclsem; /* For mutually exclusive thread creation */
static sem_t                   g_syncsem; /* Thread data passing interlock */
static struct usbhost_state_s *g_priv;    /* Data passed to thread */

/* The following tables map keyboard scan codes to printable ASIC
 * characters.  There is no support here for function keys or cursor
 * controls.
 */

#ifndef CONFIG_HIDKBD_RAWSCANCODES
#ifdef CONFIG_HIDKBD_ENCODED

/* The first and last scancode values with encode-able values */

#define FIRST_ENCODING      USBHID_KBDUSE_ENTER         /* 0x28 Keyboard Return (ENTER) */
#ifdef CONFIG_HIDKBD_ALLSCANCODES
#  define LAST_ENCODING     USBHID_KBDUSE_POWER         /* 0x66 Keyboard Power */
#else
#define LAST_ENCODING       USBHID_KBDUSE_KPDHEXADECIMAL /* 0xdd Keypad Hexadecimal */
#endif

#define USBHID_NUMENCODINGS (LAST_ENCODING - FIRST_ENCODING + 1)

static const uint8_t encoding[USBHID_NUMENCODINGS] =
{
  /* 0x28-0x2f: Enter,escape,del,back-tab,space,_,+,{ */

  KEYCODE_ENTER,   0,                   KEYCODE_FWDDEL,     KEYCODE_BACKDEL,  0,                   0,             0,                0,

  /* 0x30-0x37: },|,Non-US tilde,:,",grave tidle,<,> */

  0,               0,                   0,                 0,                0,                   0,             0,                0,

  /* 0x38-0x3f: /,CapsLock,F1,F2,F3,F4,F5,F6 */

  0,               KEYCODE_CAPSLOCK,    KEYCODE_F1,        KEYCODE_F2,       KEYCODE_F3,          KEYCODE_F4,    KEYCODE_F5,       KEYCODE_F6,

  /* 0x40-0x47: F7,F8,F9,F10,F11,F12,PrtScn,ScrollLock */

  KEYCODE_F7,      KEYCODE_F8,          KEYCODE_F9,        KEYCODE_F10,      KEYCODE_F11,         KEYCODE_F12,   KEYCODE_PRTSCRN,  KEYCODE_SCROLLLOCK,

  /* 0x48-0x4f: Pause,Insert,Home,PageUp,DeleteForward,End,PageDown,RightArrow */

  KEYCODE_PAUSE,   KEYCODE_INSERT,      KEYCODE_HOME,      KEYCODE_PAGEUP,   KEYCODE_FWDDEL,      KEYCODE_END,   KEYCODE_PAGEDOWN, KEYCODE_RIGHT,

  /* 0x50-0x57: LeftArrow,DownArrow,UpArrow,Num Lock,/,*,-,+ */

  KEYCODE_LEFT,    KEYCODE_DOWN,        KEYCODE_UP,        KEYCODE_NUMLOCK,  0,                   0,             0,                0,

  /* 0x58-0x5f: Enter,1-7 */

  KEYCODE_ENTER,   0,                   0,                 0,                0,                   0,             0,                0,

  /* 0x60-0x66: 8-9,0,.,Non-US \,Application,Power */

  0,               0,                   0,                 0,                0,                   0,             KEYCODE_POWER,

#ifdef CONFIG_HIDKBD_ALLSCANCODES

  0, /* 0x67 = */

  /* 0x68-0x6f: F13,F14,F15,F16,F17,F18,F19,F20 */

  KEYCODE_F13,     KEYCODE_F14,         KEYCODE_F15,       KEYCODE_F16,      KEYCODE_F17,         KEYCODE_F18,   KEYCODE_F19,      KEYCODE_F20,

  /* 0x70-0x77: F21,F22,F23,F24,Execute,Help,Menu,Select */

  KEYCODE_F21,     KEYCODE_F22,         KEYCODE_F23,       KEYCODE_F24,      KEYCODE_EXECUTE,     KEYCODE_HELP,  KEYCODE_MENU,     KEYCODE_SELECT,

  /* 0x78-0x7f: Stop,Again,Undo,Cut,Copy,Paste,Find,Mute */

  KEYCODE_STOP,    KEYCODE_AGAIN,       KEYCODE_UNDO,      KEYCODE_CUT,      KEYCODE_COPY,        KEYCODE_PASTE, KEYCODE_FIND,     KEYCODE_MUTE,

  /* 0x80-0x87: VolUp,VolDown,LCapsLock,lNumLock,LScrollLock,,,=,International1 */

  KEYCODE_VOLUP,   KEYCODE_VOLDOWN,     KEYCODE_LCAPSLOCK, KEYCODE_LNUMLOCK, KEYCODE_LSCROLLLOCK, 0,             0,                0,

  /* 0x88-0x8f: International 2-9 */

  0,               0,                   0,                 0,                0,                   0,             0,                0,

  /* 0x90-0x97: LAN 1-8 */

  KEYCODE_LANG1,   KEYCODE_LANG2,       KEYCODE_LANG3,     KEYCODE_LANG4,    KEYCODE_LANG5,       KEYCODE_LANG6, KEYCODE_LANG7,    KEYCODE_LANG8,

  /* 0x98-0x9f: LAN 9,Erase,SysReq,Cancel,Clear,Prior,Return,Separator */

  0,               0,                   KEYCODE_SYSREQ,    KEYCODE_CANCEL,   KEYCODE_CLEAR,       0,             KEYCODE_ENTER,    0,

  /* 0xa0-0xa7: Out,Oper,Clear,CrSel,Excel,(reserved) */

  0,               0,                   0,                 0,                0,                   0,             0,                 0,

  /* 0xa8-0xaf: (reserved) */

  0,               0,                   0,                 0,                0,                   0,             0,                 0,

  /* 0xb0-0xb7: 00,000,ThouSeparator,DecSeparator,CurrencyUnit,SubUnit,(,) */

  0,               0,                   0,                 0,                0,                   0,             0,                 0,

  /* 0xb8-0xbf: {,},tab,backspace,A-D */

  0,               0,                   0,                 KEYCODE_BACKDEL,  0,                   0,             0,                 0,

  /* 0xc0-0xc7: E-F,XOR,^,%,<,>,& */

  0,               0,                   0,                 0,                0,                   0,             0,                 0,

  /* 0xc8-0xcf: &&,|,||,:,#, ,@,! */

  0,               0,                   0,                 0,                0,                   0,             0,                 0,

  /* 0xd0-0xd7: Memory Store,Recall,Clear,Add,Subtract,Muliply,Divide,+/- */

  KEYCODE_MEMSTORE, KEYCODE_MEMRECALL,  KEYCODE_MEMCLEAR,  KEYCODE_MEMADD,   KEYCODE_MEMSUB,     KEYCODE_MEMMUL, KEYCODE_MEMDIV,    KEYCODE_NEGATE,

  /* 0xd8-0xdd: Clear,ClearEntry,Binary,Octal,Decimal,Hexadecimal */

  KEYCODE_CLEAR,    KEYCODE_CLEARENTRY, KEYCODE_BINARY,    KEYCODE_OCTAL,    KEYCODE_DECIMAL,    KEYCODE_HEXADECIMAL
#endif
};

#endif

static const uint8_t ucmap[USBHID_NUMSCANCODES] =
{
  0,    0,      0,      0,       'A',  'B',  'C',    'D',  /* 0x00-0x07: Reserved, errors, A-D */
  'E',  'F',    'G',    'H',     'I',  'J',  'K',    'L',  /* 0x08-0x0f: E-L */
  'M',  'N',    'O',    'P',     'Q',  'R',  'S',    'T',  /* 0x10-0x17: M-T */
  'U',  'V',    'W',    'X',     'Y',  'Z',  '!',    '@',  /* 0x18-0x1f: U-Z,!,@  */
  '#',  '$',    '%',    '^',     '&',  '*',  '(',    ')',  /* 0x20-0x27: #,$,%,^,&,*,(,) */
  '\n', '\033', '\177', 0,       ' ',  '_',  '+',    '{',  /* 0x28-0x2f: Enter,escape,del,back-tab,space,_,+,{ */
  '}',  '|',    0,      ':',     '"',  0,    '<',    '>',  /* 0x30-0x37: },|,Non-US tilde,:,",grave tidle,<,> */
  '?',  0,       0,      0,      0,    0,    0,      0,    /* 0x38-0x3f: /,CapsLock,F1,F2,F3,F4,F5,F6 */
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0x40-0x47: F7,F8,F9,F10,F11,F12,PrtScn,ScrollLock */
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0x48-0x4f: Pause,Insert,Home,PageUp,DeleteForward,End,PageDown,RightArrow */
  0,    0,       0,      0,      '/',  '*',  '-',    '+',  /* 0x50-0x57: LeftArrow,DownArrow,UpArrow,Num Lock,/,*,-,+ */
  '\n', '1',     '2',    '3',    '4',  '4',  '6',    '7',  /* 0x58-0x5f: Enter,1-7 */
  '8',  '9',     '0',    '.',    0,    0,    0,      '=',  /* 0x60-0x67: 8-9,0,.,Non-US \,Application,Power,= */
#ifdef CONFIG_HIDKBD_ALLSCANCODES
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0x68-0x6f: F13,F14,F15,F16,F17,F18,F19,F20 */
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0x70-0x77: F21,F22,F23,F24,Execute,Help,Menu,Select */
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0x78-0x7f: Stop,Again,Undo,Cut,Copy,Paste,Find,Mute */
  0,    0,       0,      0,      0,    ',',  0,      0,    /* 0x80-0x87: VolUp,VolDown,LCapsLock,lNumLock,LScrollLock,,,=,International1 */
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0x88-0x8f: International 2-9 */
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0x90-0x97: LAN 1-8 */
  0,    0,       0,      0,      0,    0,    '\n',   0,    /* 0x98-0x9f: LAN 9,Erase,SysReq,Cancel,Clear,Prior,Return,Separator */
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0xa0-0xa7: Out,Oper,Clear,CrSel,Excel,(reserved) */
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0xa8-0xaf: (reserved) */
  0,    0,       0,      0,      0,    0,    '(',    ')',  /* 0xb0-0xb7: 00,000,ThouSeparator,DecSeparator,CurrencyUnit,SubUnit,(,) */
  '{',  '}',    '\t',    \177,   'A',  'B',  'C',    'D',  /* 0xb8-0xbf: {,},tab,backspace,A-D */
  'F',  'F',     0,      '^',    '%',  '<', '>',     '&',  /* 0xc0-0xc7: E-F,XOR,^,%,<,>,& */
  0,    '|',     0,      ':',    '%',  ' ', '@',     '!',  /* 0xc8-0xcf: &&,|,||,:,#, ,@,! */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0xd0-0xd7: Memory Store,Recall,Clear,Add,Subtract,Muliply,Divide,+/- */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0xd8-0xdf: Clear,ClearEntry,Binary,Octal,Decimal,Hexadecimal */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0xe0-0xe7: Left Ctrl,Shift,Alt,GUI, Right Ctrl,Shift,Alt,GUI */
#endif
};

static const uint8_t lcmap[USBHID_NUMSCANCODES] =
{
  0,    0,       0,      0,      'a',  'b', 'c',     'd',  /* 0x00-0x07: Reserved, errors, a-d */
  'e',  'f',     'g',    'h',    'i',  'j', 'k',     'l',  /* 0x08-0x0f: e-l */
  'm',  'n',     'o',    'p',    'q',  'r', 's',     't',  /* 0x10-0x17: m-t */
  'u',  'v',     'w',    'x',    'y',  'z', '1',     '2',  /* 0x18-0x1f: u-z,1-2  */
  '3',  '4',     '5',    '6',    '7',  '8', '9',     '0',  /* 0x20-0x27: 3-9,0 */
  '\n', '\033',  '\177', '\t',   ' ',  '-', '=',     '[',  /* 0x28-0x2f: Enter,escape,del,tab,space,-,=,[ */
  ']',  '\\',    '\234', ';',    '\'', 0,   ',',     '.',  /* 0x30-0x37: ],\,Non-US pound,;,',grave accent,,,. */
  '/',  0,       0,      0,      0,    0,   0,       0,    /* 0x38-0x3f: /,CapsLock,F1,F2,F3,F4,F5,F6 */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0x40-0x47: F7,F8,F9,F10,F11,F12,PrtScn,ScrollLock */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0x48-0x4f: Pause,Insert,Home,PageUp,DeleteForward,End,PageDown,RightArrow */
  0,    0,       0,      0,      '/',  '*', '-',     '+',  /* 0x50-0x57: LeftArrow,DownArrow,UpArrow,Num Lock,/,*,-,+ */
  '\n', '1',     '2',    '3',    '4',  '4', '6',     '7',  /* 0x58-0x5f: Enter,1-7 */
  '8',  '9',     '0',    '.',    0,    0,   0,       '=',  /* 0x60-0x67: 8-9,0,.,Non-US \,Application,Power,= */
#ifdef CONFIG_HIDKBD_ALLSCANCODES
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0x68-0x6f: F13,F14,F15,F16,F17,F18,F19,F20 */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0x70-0x77: F21,F22,F23,F24,Execute,Help,Menu,Select */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0x78-0x7f: Stop,Again,Undo,Cut,Copy,Paste,Find,Mute */
  0,    0,       0,      0,      0,    ',', 0,       0,    /* 0x80-0x87: VolUp,VolDown,LCapsLock,lNumLock,LScrollLock,,,=,International1 */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0x88-0x8f: International 2-9 */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0x90-0x97: LAN 1-8 */
  0,    0,       0,      0,      0,    0,   '\n',    0,    /* 0x98-0x9f: LAN 9,Erase,SysReq,Cancel,Clear,Prior,Return,Separator */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0xa0-0xa7: Out,Oper,Clear,CrSel,Excel,(reserved) */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0xa8-0xaf: (reserved) */
  0,    0,       0,      0,      0,    0,   '(',     ')',  /* 0xb0-0xb7: 00,000,ThouSeparator,DecSeparator,CurrencyUnit,SubUnit,(,) */
  '{',  '}',    '\t',    '\177', 'A',  'B', 'C',     'D',  /* 0xb8-0xbf: {,},tab,backspace,A-D */
  'F',  'F',     0,      '^',    '%',  '<', '>',     '&',  /* 0xc0-0xc7: E-F,XOR,^,%,<,>,& */
  0,    '|',     0,      ':',    '%',  ' ', '@',     '!',  /* 0xc8-0xcf: &&,|,||,:,#, ,@,! */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0xd0-0xd7: Memory Store,Recall,Clear,Add,Subtract,Muliply,Divide,+/- */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0xd8-0xdf: Clear,ClearEntry,Binary,Octal,Decimal,Hexadecimal */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0xe0-0xe7: Left Ctrl,Shift,Alt,GUI, Right Ctrl,Shift,Alt,GUI */
#endif
};
#endif /* CONFIG_HIDKBD_RAWSCANCODES */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_takesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 ****************************************************************************/

static void usbhost_takesem(sem_t *sem)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occr here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: usbhost_pollnotify
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static void usbhost_pollnotify(FAR struct usbhost_state_s *priv)
{
  int i;

  for (i = 0; i < CONFIG_HIDKBD_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= (fds->events & POLLIN);
          if (fds->revents != 0)
            {
              uvdbg("Report events: %02x\n", fds->revents);
              sem_post(fds->sem);
            }
        }
    }
}
#endif

/****************************************************************************
 * Name: usbhost_allocclass
 *
 * Description:
 *   This is really part of the logic that implements the create() method
 *   of struct usbhost_registry_s.  This function allocates memory for one
 *   new class instance.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s.  NULL is returned on failure; this function will
 *   will fail only if there are insufficient resources to create another
 *   USB host class instance.
 *
 ****************************************************************************/

static inline FAR struct usbhost_state_s *usbhost_allocclass(void)
{
  FAR struct usbhost_state_s *priv;

  DEBUGASSERT(!up_interrupt_context());
  priv = (FAR struct usbhost_state_s *)kmalloc(sizeof(struct usbhost_state_s));
  uvdbg("Allocated: %p\n", priv);;
  return priv;
}

/****************************************************************************
 * Name: usbhost_freeclass
 *
 * Description:
 *   Free a class instance previously allocated by usbhost_allocclass().
 *
 * Input Parameters:
 *   class - A reference to the class instance to be freed.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline void usbhost_freeclass(FAR struct usbhost_state_s *class)
{
  DEBUGASSERT(class != NULL);

  /* Free the class instance. */

  uvdbg("Freeing: %p\n", class);;
  kfree(class);
}

/****************************************************************************
 * Name: Device name management
 *
 * Description:
 *   Some tiny functions to coordinate management of device names.
 *
 ****************************************************************************/

static int usbhost_allocdevno(FAR struct usbhost_state_s *priv)
{
  irqstate_t flags;
  int devno;

  flags = irqsave();
  for (devno = 0; devno < 26; devno++)
    {
      uint32_t bitno = 1 << devno;
      if ((g_devinuse & bitno) == 0)
        {
          g_devinuse |= bitno;
          priv->devchar = 'a' + devno;
          irqrestore(flags);
          return OK;
        }
    }

  irqrestore(flags);
  return -EMFILE;
}

static void usbhost_freedevno(FAR struct usbhost_state_s *priv)
{
  int devno = 'a' - priv->devchar;

  if (devno >= 0 && devno < 26)
    {
      irqstate_t flags = irqsave();
      g_devinuse &= ~(1 << devno);
      irqrestore(flags);
    }
}

static inline void usbhost_mkdevname(FAR struct usbhost_state_s *priv, char *devname)
{
  (void)snprintf(devname, DEV_NAMELEN, DEV_FORMAT, priv->devchar);
}

/****************************************************************************
 * Name: usbhost_destroy
 *
 * Description:
 *   The USB device has been disconnected and the refernce count on the USB
 *   host class instance has gone to 1.. Time to destroy the USB host class
 *   instance.
 *
 * Input Parameters:
 *   arg - A reference to the class instance to be destroyed.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_destroy(FAR void *arg)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)arg;
  char devname[DEV_NAMELEN];

  DEBUGASSERT(priv != NULL);
  uvdbg("crefs: %d\n", priv->crefs);
 
  /* Unregister the driver */

  uvdbg("Unregister driver\n");
  usbhost_mkdevname(priv, devname);
  (void)unregister_driver(devname);

  /* Release the device name used by this connection */

  usbhost_freedevno(priv);

  /* Free the interrupt endpoints */

  if (priv->epin)
    {
      DRVR_EPFREE(priv->drvr, priv->epin);
    }

  if (priv->epout)
    {
      DRVR_EPFREE(priv->drvr, priv->epout);
    }

  /* Free any transfer buffers */

  usbhost_tdfree(priv);

  /* Destroy the semaphores */

  sem_destroy(&priv->exclsem);
  sem_destroy(&priv->waitsem);

  /* Disconnect the USB host device */

  DRVR_DISCONNECT(priv->drvr);

  /* And free the class instance.  Hmmm.. this may execute on the worker
   * thread and the work structure is part of what is getting freed.  That
   * should be okay because once the work contained is removed from the
   * queue, it should not longer be accessed by the worker thread.
   */

  usbhost_freeclass(priv);
}

/****************************************************************************
 * Name: usbhost_putbuffer
 *
 * Description:
 *   Add one character to the user buffer.
 *
 * Input Parameters:
 *   priv - Driver internal state
 *   keycode - The value to add to the user buffer
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_putbuffer(FAR struct usbhost_state_s *priv,
                              uint8_t keycode)
{
  register unsigned int head;
  register unsigned int tail;

  /* Copy the next keyboard character into the user buffer. */

  head = priv->headndx;
  priv->kbdbuffer[head] = keycode;

  /* Increment the head index */

  if (++head >= CONFIG_HIDKBD_BUFSIZE)
    {
      head = 0;
    }

  /* If the buffer is full, then increment the tail index to make space.  Is
   * it better to lose old keystrokes or new?
   */

  tail = priv->tailndx;
  if (tail == head)
    {
      if (++tail >= CONFIG_HIDKBD_BUFSIZE)
        {
          tail = 0;
        }

      /* Save the updated tail index */

      priv->tailndx = tail;
    }

  /* Save the updated head index */

  priv->headndx = head;
}

/****************************************************************************
 * Name: usbhost_putstream
 *
 * Description:
 *   A wrapper for usbhost_putc that is compatibile with the lib_outstream_s
 *   putc methos.
 *
 * Input Parameters:
 *   stream - The struct lib_outstream_s reference
 *   ch - The character to add to the user buffer
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_HIDKBD_ENCODED
static void usbhost_putstream(FAR struct lib_outstream_s *stream, int ch)
{
  FAR struct usbhost_outstream_s *privstream = (FAR struct lib_outstream_s *)stream;

  DEBUGASSERT(privstream && privstream->priv);
  usbhost_putbuffer(privstream->priv), (uint8_t)ch);
  stream->nput++;
}
#endif

/****************************************************************************
 * Name: usbhost_mapscancode
 *
 * Description:
 *   Map a keyboard scancode to a printable ASCII character.  There is no
 *   support here for function keys or cursor controls in this version of
 *   the driver.
 *
 * Input Parameters:
 *   scancode - Scan code to be mapped.
 *   modifier - Ctrl,Alt,Shift,GUI modifier bits
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline uint8_t usbhost_mapscancode(uint8_t scancode, uint8_t modifier)
{
#ifndef CONFIG_HIDKBD_RAWSCANCODES
  /* Range check */

  if (scancode >= USBHID_NUMSCANCODES)
    {
      return 0;
    }

  /* Is either shift key pressed? */

  if ((modifier & (USBHID_MODIFER_LSHIFT|USBHID_MODIFER_RSHIFT)) != 0)
    {
      return ucmap[scancode];
    }
  else
    {
      return lcmap[scancode];
    }
#else
  return scancode;
#endif
}

/****************************************************************************
 * Name: usbhost_encodescancode
 *
 * Description:
 *  Check if the key has a special function encoding and, if it does, add
 *  the encoded value to the user buffer.
 *
 * Input Parameters:
 *   priv - Driver internal state
 *   scancode - Scan code to be mapped.
 *   modifier - Ctrl,Alt,Shift,GUI modifier bits
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_HIDKBD_ENCODED
static inline void usbhost_encodescancode(FAR struct usbhost_state_s *priv,
                                          uint8_t scancode, uint8_t modifier)
{
  struct usbhost_outstream_s stream;
  uint8_t encoded;

  /* Check if the raw scancode is in a valid range */

  if (scancode >= FIRST_ENCODING && scancode <= LAST_ENCODING)
    {
      /* Yes the value is within range */

      encoded = encoding(scancode - FIRST_ENCODING);
      ivdbg("  scancode: %02x modifier: %02x encoded: %d\n",
            scancode, modifier, encoded);

      if (encoded)
        {
          struct usbhost_outstream_s usbstream;

          /* And it does correspond to a special function key */

          usbstream->stream.put  = usbhost_putstream;
          usbstream->stream.nput = 0;
          usbstream->priv        = priv;

          /* Add the special function value to the user buffer */

          kbd_putspecial((enum kbd_keycode_e)encoded, &usbstream);
        }
    }
}
#endif

/****************************************************************************
 * Name: usbhost_kbdpoll
 *
 * Description:
 *   Periodically check for new keyboard data.
 *
 * Input Parameters:
 *   arg - A reference to the class instance to be destroyed.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static int usbhost_kbdpoll(int argc, char *argv[])
{
  FAR struct usbhost_state_s *priv;
  FAR struct usb_ctrlreq_s   *ctrlreq;
#ifndef CONFIG_HIDKBD_NODEBOUNCE
  uint8_t                     lastkey[6] = {0, 0, 0, 0, 0, 0};
#endif
#if defined(CONFIG_DEBUG_USB) && defined(CONFIG_DEBUG_VERBOSE)
  unsigned int                npolls = 0;
#endif
  unsigned int                nerrors = 0;
  bool                        empty = true;
  bool                        newstate;
  int                         ret;

  uvdbg("Started\n");

  /* Synchronize with the start-up logic.  Get the private instance, re-start
   * the start-up logic, and wait a bit to make sure that all of the class
   * creation logic has a chance to run to completion.
   *
   * NOTE: that the reference count is incremented here.  Therefore, we know
   * that the driver data structure will remain stable while this thread is
   * running.
   */

  priv = g_priv;
  DEBUGASSERT(priv != NULL);
 
  priv->polling = true;
  priv->crefs++;
  usbhost_givesem(&g_syncsem);
  sleep(1);
  
  /* Loop here until the device is disconnected */

  uvdbg("Entering poll loop\n");

  while (!priv->disconnected)
    {
      /* Make sure that we have exclusive access to the private data
       * structure. There may now be other tasks with the character driver
       * open and actively trying to interact with the class driver.
       */

      usbhost_takesem(&priv->exclsem);

      /* Format the HID report request:
       *
       *   bmRequestType 10100001
       *   bRequest      GET_REPORT (0x01)
       *   wValue        Report Type and Report Index
       *   wIndex        Interface Number
       *   wLength       Descriptor Length
       *   Data          Descriptor Data
       */

      ctrlreq       = (struct usb_ctrlreq_s *)priv->tbuffer;
      ctrlreq->type = USB_REQ_DIR_IN|USB_REQ_TYPE_CLASS|USB_REQ_RECIPIENT_INTERFACE;
      ctrlreq->req  = USBHID_REQUEST_GETREPORT;

      usbhost_putle16(ctrlreq->value, (USBHID_REPORTTYPE_INPUT << 8));
      usbhost_putle16(ctrlreq->index, priv->ifno);
      usbhost_putle16(ctrlreq->len,   sizeof(struct usbhid_kbdreport_s));

      /* Send HID report request */

      ret = DRVR_CTRLIN(priv->drvr, ctrlreq, priv->tbuffer);
      usbhost_givesem(&priv->exclsem);

      /* Check for errors -- Bail if an excessive number of errors
       * are encountered.
       */

      if (ret != OK)
        {
          nerrors++;
          udbg("ERROR: GETREPORT/INPUT, DRVR_CTRLIN returned: %d/%d\n",
               ret, nerrors);

          if (nerrors > 200)
            {
              udbg("Too many errors... aborting: %d\n", nerrors);
              break;
            }
        }

      /* The report was received correctly.  But ignore the keystrokes if no
       * task has opened the driver.
       */

      else if (priv->open)
        {
          struct usbhid_kbdreport_s *rpt = (struct usbhid_kbdreport_s *)priv->tbuffer;
          uint8_t                    keycode;
          int                        i;

          /* Add the newly received keystrokes to our internal buffer */

          usbhost_takesem(&priv->exclsem);
          for (i = 0; i < 6; i++)
            {
              /* Is this key pressed?  But not pressed last time?
               * HID spec: "The order of keycodes in array fields has no
               * significance. Order determination is done by the host
               * software comparing the contents of the previous report to
               * the current report. If two or more keys are reported in
               * one report, their order is indeterminate. Keyboards may
               * buffer events that would have otherwise resulted in
               * multiple event in a single report.
               *
               * "'Repeat Rate' and 'Delay Before First Repeat' are
               * implemented by the host and not in the keyboard (this
               * means the BIOS in legacy mode). The host may use the
               * device report rate and the number of reports to determine
               * how long a key is being held down. Alternatively, the host
               * may use its own clock or the idle request for the timing
               * of these features."
               */

              if (rpt->key[i] != USBHID_KBDUSE_NONE
#ifndef CONFIG_HIDKBD_NODEBOUNCE
                 && rpt->key[i] != lastkey[i]
#endif
                 )
                {
                  /* Yes.. Add it to the buffer. */

                  /* Map the keyboard scancode to a printable ASCII
                   * character.  There is no support here for function keys
                   * or cursor controls in this version of the driver.
                   */

                  keycode = usbhost_mapscancode(rpt->key[i], rpt->modifier);
                  ivdbg("Key %d: %02x keycode:%c modifier: %02x\n",
                         i, rpt->key[i], keycode ? keycode : ' ', rpt->modifier);

                  /* Zero at this point means that the key does not map to a
                   * printable character.
                   */

                  if (keycode != 0)
                    {
                      /* Handle control characters.  Zero after this means
                       * a valid, NUL character.
                       */

                      if ((rpt->modifier & (USBHID_MODIFER_LCTRL|USBHID_MODIFER_RCTRL)) != 0)
                        {
                          keycode &= 0x1f;
                        }
 
                      /* Copy the next keyboard character into the user
                       * buffer.
                       */

                      usbhost_putbuffer(priv, keycode);
                    }

                  /* The zero might, however, map to a special keyboard action (such as a
                   * cursor movement or function key).  Attempt to encode the special key.
                   */

#ifdef CONFIG_HIDKBD_ENCODED
                  else
                    {
                      usbhost_encodescancode(priv, rpt->key[i], rpt->modifier));
                    }
#endif
                }

              /* Save the scancode (or lack thereof) for key debouncing on
               * next keyboard report.
               */

#ifndef CONFIG_HIDKBD_NODEBOUNCE
              lastkey[i] = rpt->key[i];
#endif
            }

          /* Is there data available? */

          newstate = (priv->headndx == priv->tailndx);
          if (!newstate)
            {
              /* Yes.. Is there a thread waiting for keyboard data now? */

              if (priv->waiting)
                {
                  /* Yes.. wake it up */

                  usbhost_givesem(&priv->waitsem);
                  priv->waiting = false;
                }

              /* Did we just transition from no data available to data
               * available?  If so, wake up any threads waiting for the
               * POLLIN event.
               */

              if (empty)
                {
                  usbhost_pollnotify(priv);
                }
            }

          empty = newstate;
          usbhost_givesem(&priv->exclsem);
        }

      /* If USB debug is on, then provide some periodic indication that
       * polling is still happening.
       */

#if defined(CONFIG_DEBUG_USB) && defined(CONFIG_DEBUG_VERBOSE)
      npolls++;
      if ((npolls & 31) == 0)
        {
          udbg("Still polling: %d\n", npolls);
        }
#endif
      /* Wait for the required amount (or until a signal is received).  We
       * will wake up when either the delay elapses or we are signalled that
       * the device has been disconnected.
       */

      usleep(CONFIG_HIDKBD_POLLUSEC);
    }

  /* We get here when the driver is removed.. or when too many errors have
   * been encountered.
   *
   * Make sure that we have exclusive access to the private data structure.
   * There may now be other tasks with the character driver open and actively
   * trying to interact with the class driver.
   */

  usbhost_takesem(&priv->exclsem);

  /* Indicate that we are no longer running and decrement the reference
   * count help by this thread.  If there are no other users of the class,
   * we can destroy it now.  Otherwise, we have to wait until the all
   * of the file descriptors are closed.
   */
 
  udbg("Keyboard removed, polling halted\n");
  priv->polling = false;
  if (--priv->crefs < 2)
    {
      /* Destroy the instance (while we hold the semaphore!) */
 
      usbhost_destroy(priv);
    }
  else
    {
      /* No, we will destroy the driver instance when it is finally closed */

      usbhost_givesem(&priv->exclsem);
    }

  return 0;
}

/****************************************************************************
 * Name: usbhost_cfgdesc
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   priv - The USB host class instance.
 *   configdesc - A pointer to a uint8_t buffer container the configuration descripor.
 *   desclen - The length in bytes of the configuration descriptor.
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static inline int usbhost_cfgdesc(FAR struct usbhost_state_s *priv,
                                  FAR const uint8_t *configdesc, int desclen,
                                  uint8_t funcaddr)
{
  FAR struct usb_cfgdesc_s *cfgdesc;
  FAR struct usb_desc_s *desc;
  FAR struct usbhost_epdesc_s epindesc;
  FAR struct usbhost_epdesc_s epoutdesc;
  int remaining;
  uint8_t found = 0;
  bool done = false;
  int ret;

  DEBUGASSERT(priv != NULL && 
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));
  
  /* Verify that we were passed a configuration descriptor */

  cfgdesc = (FAR struct usb_cfgdesc_s *)configdesc;
  if (cfgdesc->type != USB_DESC_TYPE_CONFIG)
    {
      return -EINVAL;
    }

  /* Get the total length of the configuration descriptor (little endian).
   * It might be a good check to get the number of interfaces here too.
  */

  remaining = (int)usbhost_getle16(cfgdesc->totallen);

  /* Skip to the next entry descriptor */

  configdesc += cfgdesc->len;
  remaining  -= cfgdesc->len;

  /* Loop where there are more dscriptors to examine */

  while (remaining >= sizeof(struct usb_desc_s) && !done)
    {
      /* What is the next descriptor? */

      desc = (FAR struct usb_desc_s *)configdesc;
      switch (desc->type)
        {
        /* Interface descriptor. We really should get the number of endpoints
         * from this descriptor too.
         */

        case USB_DESC_TYPE_INTERFACE:
          {
            FAR struct usb_ifdesc_s *ifdesc = (FAR struct usb_ifdesc_s *)configdesc;
 
            uvdbg("Interface descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_IFDESC);

            /* Did we already find what we needed from a preceding interface? */

            if ((found & USBHOST_RQDFOUND) == USBHOST_RQDFOUND)
              {
                /* Yes.. then break out of the loop and use the preceding
                 * interface.
                 */

                done       = true;
              }
            else
              {
                /* Otherwise, save the interface number and discard any
                 * endpoints previously found
                 */

                priv->ifno = ifdesc->ifno;
                found      = USBHOST_IFFOUND;
              }
          }
          break;

        /* HID descriptor */

        case USBHID_DESCTYPE_HID:
            uvdbg("HID descriptor\n");
            break;

        /* Endpoint descriptor.  We expect one or two interrupt endpoints,
         * a required IN endpoint and an optional OUT endpoint.
         */

        case USB_DESC_TYPE_ENDPOINT:
          {
            FAR struct usb_epdesc_s *epdesc = (FAR struct usb_epdesc_s *)configdesc;

            uvdbg("Endpoint descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_EPDESC);

            /* Check for an interrupt endpoint. */

            if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) == USB_EP_ATTR_XFER_INT)
              {
                /* Yes.. it is a interrupt endpoint.  IN or OUT? */

                if (USB_ISEPOUT(epdesc->addr))
                  {
                    /* It is an interrupt OUT endpoint.  There not be more than one
                     * interrupt OUT endpoint.
                     */

                    if ((found & USBHOST_EPOUTFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know what to do with this. */

                        return -EINVAL;
                      }
                    found |= USBHOST_EPOUTFOUND;

                    /* Save the interrupt OUT endpoint information */

                    epoutdesc.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    epoutdesc.in           = false;
                    epoutdesc.funcaddr     = funcaddr;
                    epoutdesc.xfrtype      = USB_EP_ATTR_XFER_INT;
                    epoutdesc.interval     = epdesc->interval;
                    epoutdesc.mxpacketsize = usbhost_getle16(epdesc->mxpacketsize);
                    uvdbg("Interrupt OUT EP addr:%d mxpacketsize:%d\n",
                          epoutdesc.addr, epoutdesc.mxpacketsize);
                  }
                else
                  {
                    /* It is an interrupt IN endpoint.  There should be only
                     * one interrupt IN endpoint.
                     */

                    if ((found & USBHOST_EPINFOUND) != 0)
                      {
                        /* Oops.. more than one endpint.  We don't know what
                         * to do with this.
                         */

                        return -EINVAL;
                      }
                    found |= USBHOST_EPINFOUND;

                    /* Save the interrupt IN endpoint information */
                    
                    epindesc.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    epindesc.in           = 1;
                    epindesc.funcaddr     = funcaddr;
                    epindesc.xfrtype      = USB_EP_ATTR_XFER_INT;
                    epindesc.interval     = epdesc->interval;
                    epindesc.mxpacketsize = usbhost_getle16(epdesc->mxpacketsize);
                    uvdbg("Interrupt IN EP addr:%d mxpacketsize:%d\n",
                          epindesc.addr, epindesc.mxpacketsize);
                  }
              }
          }
          break;

        /* Other descriptors are just ignored for now */

        default:
          uvdbg("Other descriptor: %d\n", desc->type);
          break;
        }

      /* What we found everything that we are going to find? */

      if (found == USBHOST_ALLFOUND)
        {
          /* Yes.. then break out of the loop and use the preceding interface */

          done = true;
        }

      /* Increment the address of the next descriptor */
 
      configdesc += desc->len;
      remaining  -= desc->len;
    }

  /* Sanity checking... did we find all of things that we need? */
    
  if ((found & USBHOST_RQDFOUND) != USBHOST_RQDFOUND)
    {
      ulldbg("ERROR: Found IF:%s EPIN:%s\n",
             (found & USBHOST_IFFOUND) != 0  ? "YES" : "NO",
             (found & USBHOST_EPINFOUND) != 0 ? "YES" : "NO");
      return -EINVAL;
    }

  /* We are good... Allocate the endpoints.  First, the required interrupt
   * IN endpoint.
   */

  ret = DRVR_EPALLOC(priv->drvr, &epindesc, &priv->epin);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate interrupt IN endpoint\n");
      return ret;
    }

  /* Then the optional interrupt OUT endpoint */

  ullvdbg("Found EPOOUT:%s\n",
         (found & USBHOST_EPOUTFOUND) != 0 ? "YES" : "NO");

  if ((found & USBHOST_EPOUTFOUND) != 0)
    {
      ret = DRVR_EPALLOC(priv->drvr, &epoutdesc, &priv->epout);
      if (ret != OK)
        {
          udbg("ERROR: Failed to allocate interrupt OUT endpoint\n");
          (void)DRVR_EPFREE(priv->drvr, priv->epin);
          return ret;
        }
    }

  ullvdbg("Endpoints allocated\n");
  return OK;
}

/****************************************************************************
 * Name: usbhost_devinit
 *
 * Description:
 *   The USB device has been successfully connected.  This completes the
 *   initialization operations.  It is first called after the
 *   configuration descriptor has been received.
 *
 *   This function is called from the connect() method.  This function always
 *   executes on the thread of the caller of connect().
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline int usbhost_devinit(FAR struct usbhost_state_s *priv)
{
  char devname[DEV_NAMELEN];
  int ret;

  /* Set aside a transfer buffer for exclusive use by the keyboard class driver */

  ret = usbhost_tdalloc(priv);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate transfer buffer\n");
      return ret;
    }

  /* Increment the reference count.  This will prevent usbhost_destroy() from
   * being called asynchronously if the device is removed.
   */

  priv->crefs++;
  DEBUGASSERT(priv->crefs == 2);

  /* Start a worker task to poll the USB device.  It would be nice to used the
   * the NuttX worker thread to do this, but this task needs to wait for events
   * and activities on the worker thread should not involve significant waiting.
   * Having a dedicated thread is more efficient in this sense, but requires more
   * memory resources, primarily for the dedicated stack (CONFIG_HIDKBD_STACKSIZE).
   */

  uvdbg("user_start: Start poll task\n");

  /* The inputs to a task started by task_create() are very awkard for this
   * purpose.  They are really designed for command line tasks (argc/argv). So
   * the following is kludge pass binary data when the keyboard poll task
   * is started.
   *
   * First, make sure we have exclusive access to g_priv (what is the likelihood
   * of this being used?  About zero, but we protect it anyway).
   */

  usbhost_takesem(&g_exclsem);
  g_priv = priv;

#ifndef CONFIG_CUSTOM_STACK
  priv->pollpid = task_create("usbhost", CONFIG_HIDKBD_DEFPRIO,
                              CONFIG_HIDKBD_STACKSIZE,
                              (main_t)usbhost_kbdpoll, (const char **)NULL);
#else
  priv->pollpid = task_create("usbhost", CONFIG_HIDKBD_DEFPRIO,
                              (main_t)usbhost_kbdpoll, (const char **)NULL);
#endif
  if (priv->pollpid == ERROR)
    {
      /* Failed to started the poll thread... probably due to memory resources */

      usbhost_givesem(&g_exclsem);
      ret = -ENOMEM;
      goto errout;
    }

  /* Now wait for the poll task to get properly initialized */

  usbhost_takesem(&g_syncsem);
  usbhost_givesem(&g_exclsem);

  /* Register the driver */

  uvdbg("Register driver\n");
  usbhost_mkdevname(priv, devname);
  ret = register_driver(devname, &usbhost_fops, 0666, priv);

  /* We now have to be concerned about asynchronous modification of crefs
   * because the driver has been registerd.
   */

errout:
  usbhost_takesem(&priv->exclsem);
  priv->crefs--;
  usbhost_givesem(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: usbhost_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Values:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static inline uint16_t usbhost_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: usbhost_putle16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 16-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_putle16(uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}

/****************************************************************************
 * Name: usbhost_getle32
 *
 * Description:
 *   Get a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the big endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline uint32_t usbhost_getle32(const uint8_t *val)
{
 /* Little endian means LS halfword first in byte stream */

  return (uint32_t)usbhost_getle16(&val[2]) << 16 | (uint32_t)usbhost_getle16(val);
}

/****************************************************************************
 * Name: usbhost_putle32
 *
 * Description:
 *   Put a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

#if 0 /* Not used */
static void usbhost_putle32(uint8_t *dest, uint32_t val)
{
  /* Little endian means LS halfword first in byte stream */

  usbhost_putle16(dest, (uint16_t)(val & 0xffff));
  usbhost_putle16(dest+2, (uint16_t)(val >> 16));
}
#endif

/****************************************************************************
 * Name: usbhost_tdalloc
 *
 * Description:
 *   Allocate transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On sucess, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_tdalloc(FAR struct usbhost_state_s *priv)
{
  DEBUGASSERT(priv && priv->tbuffer == NULL);
  return DRVR_ALLOC(priv->drvr, &priv->tbuffer, &priv->tbuflen);
}

/****************************************************************************
 * Name: usbhost_tdfree
 *
 * Description:
 *   Free transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On sucess, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_tdfree(FAR struct usbhost_state_s *priv)
{
  int result = OK;
  DEBUGASSERT(priv);

  if (priv->tbuffer)
    {
      DEBUGASSERT(priv->drvr);
      result         = DRVR_FREE(priv->drvr, priv->tbuffer);
      priv->tbuffer = NULL;
      priv->tbuflen = 0;
    }
  return result;
}

/****************************************************************************
 * struct usbhost_registry_s methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_create
 *
 * Description:
 *   This function implements the create() method of struct usbhost_registry_s. 
 *   The create() method is a callback into the class implementation.  It is
 *   used to (1) create a new instance of the USB host class state and to (2)
 *   bind a USB host driver "session" to the class instance.  Use of this
 *   create() method will support environments where there may be multiple
 *   USB ports and multiple USB devices simultaneously connected.
 *
 * Input Parameters:
 *   drvr - An instance of struct usbhost_driver_s that the class
 *     implementation will "bind" to its state structure and will
 *     subsequently use to communicate with the USB host driver.
 *   id - In the case where the device supports multiple base classes,
 *     subclasses, or protocols, this specifies which to configure for.
 *
 * Returned Values:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s that can be used by the USB host driver to communicate
 *   with the USB host class.  NULL is returned on failure; this function
 *   will fail only if the drvr input parameter is NULL or if there are
 *   insufficient resources to create another USB host class instance.
 *
 ****************************************************************************/

static FAR struct usbhost_class_s *usbhost_create(FAR struct usbhost_driver_s *drvr,
                                                  FAR const struct usbhost_id_s *id)
{
  FAR struct usbhost_state_s *priv;

  /* Allocate a USB host class instance */

  priv = usbhost_allocclass();
  if (priv)
    {
      /* Initialize the allocated storage class instance */

      memset(priv, 0, sizeof(struct usbhost_state_s));

      /* Assign a device number to this class instance */

      if (usbhost_allocdevno(priv) == OK)
        {
         /* Initialize class method function pointers */

          priv->class.connect      = usbhost_connect;
          priv->class.disconnected = usbhost_disconnected;

          /* The initial reference count is 1... One reference is held by the driver */

          priv->crefs              = 1;

          /* Initialize semaphores */

          sem_init(&priv->exclsem, 0, 1);
          sem_init(&priv->waitsem, 0, 0);

          /* Bind the driver to the storage class instance */

          priv->drvr               = drvr;

          /* Return the instance of the USB keyboard class driver */
 
          return &priv->class;
        }
    }

  /* An error occurred. Free the allocation and return NULL on all failures */

  if (priv)
    {
      usbhost_freeclass(priv);
    }
  return NULL;
}

/****************************************************************************
 * struct usbhost_class_s methods
 ****************************************************************************/
/****************************************************************************
 * Name: usbhost_connect
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   class - The USB host class entry previously obtained from a call to create().
 *   configdesc - A pointer to a uint8_t buffer container the configuration descripor.
 *   desclen - The length in bytes of the configuration descriptor.
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 *   NOTE that the class instance remains valid upon return with a failure.  It is
 *   the responsibility of the higher level enumeration logic to call
 *   CLASS_DISCONNECTED to free up the class driver resources.
 *
 * Assumptions:
 *   - This function will *not* be called from an interrupt handler.
 *   - If this function returns an error, the USB host controller driver
 *     must call to DISCONNECTED method to recover from the error
 *
 ****************************************************************************/

static int usbhost_connect(FAR struct usbhost_class_s *class,
                           FAR const uint8_t *configdesc, int desclen,
                           uint8_t funcaddr)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)class;
  int ret;

  DEBUGASSERT(priv != NULL && 
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  /* Parse the configuration descriptor to get the endpoints */

  ret = usbhost_cfgdesc(priv, configdesc, desclen, funcaddr);
  if (ret != OK)
    {
      udbg("usbhost_cfgdesc() failed: %d\n", ret);
    }
  else
    {
      /* Now configure the device and register the NuttX driver */

      ret = usbhost_devinit(priv);
      if (ret != OK)
        {
          udbg("usbhost_devinit() failed: %d\n", ret);
        }
    }
 
  /* ERROR handling:  Do nothing. If we return and error during connection,
   * the driver is required to call the DISCONNECT method.  Possibilities:
   *
   * - Failure occurred before the kbdpoll task was started successfully.
   *   In this case, the disconnection will have to be handled on the worker
   *   task.
   * - Failure occured after the kbdpoll task was started succesffuly.  In
   *   this case, the disconnetion can be performed on the kbdpoll thread.
   */

  return ret;
}

/****************************************************************************
 * Name: usbhost_disconnected
 *
 * Description:
 *   This function implements the disconnected() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to inform the class that the USB device has
 *   been disconnected.
 *
 * Input Parameters:
 *   class - The USB host class entry previously obtained from a call to
 *     create().
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

static int usbhost_disconnected(struct usbhost_class_s *class)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)class;

  DEBUGASSERT(priv != NULL);

  /* Set an indication to any users of the keyboard device that the device
   * is no longer available.
   */

  priv->disconnected = true;
  ullvdbg("Disconnected\n");

  /* Is there a thread waiting for keyboard data that will never come? */

  if (priv->waiting)
    {
      /* Yes.. wake it up */

      usbhost_givesem(&priv->waitsem);
      priv->waiting = false;
    }

  /* Possibilities:
   *
   * - Failure occurred before the kbdpoll task was started successfully.
   *   In this case, the disconnection will have to be handled on the worker
   *   task.
   * - Failure occured after the kbdpoll task was started succesffuly.  In
   *   this case, the disconnetion can be performed on the kbdpoll thread.
   */

  if (priv->polling)
    {
      /* The polling task is still alive. Signal the keyboard polling task. 
       * When that task wakes up, it will decrement the reference count and,
       * perhaps, destroy the class instance.  Then it will exit.
       */

      (void)kill(priv->pollpid, SIGALRM);
    }
  else
    {
      /* In the case where the failure occurs before the polling task was
       * started.  Now what?  We are probably executing from an interrupt
       * handler here.  We will use the worker thread.  This is kind of
       * wasteful and begs for a re-design.
       */

      DEBUGASSERT(priv->work.worker == NULL);
      (void)work_queue(HPWORK, &priv->work, usbhost_destroy, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Character driver methods
 ****************************************************************************/
/****************************************************************************
 * Name: usbhost_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int usbhost_open(FAR struct file *filep)
{
  FAR struct inode           *inode;
  FAR struct usbhost_state_s *priv;
  irqstate_t flags;
  int ret;

  uvdbg("Entry\n");
  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv && priv->crefs > 0 && priv->crefs < USBHOST_MAX_CREFS);
  usbhost_takesem(&priv->exclsem);

  /* Check if the keyboard device is still connected.  We need to disable
   * interrupts momentarily to assure that there are no asynchronous disconnect
   * events.
   */

  flags = irqsave();
  if (priv->disconnected)
    {
      /* No... the driver is no longer bound to the class.  That means that
       * the USB storage device is no longer connected.  Refuse any further
       * attempts to open the driver.
       */

      ret = -ENODEV;
    }
  else
    {
      /* Otherwise, just increment the reference count on the driver */

      priv->crefs++;
      priv->open = true;
      ret        = OK;
    }
  irqrestore(flags);

  usbhost_givesem(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: usbhost_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int usbhost_close(FAR struct file *filep)
{
  FAR struct inode           *inode;
  FAR struct usbhost_state_s *priv;

  uvdbg("Entry\n");
  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Decrement the reference count on the driver */

  DEBUGASSERT(priv->crefs > 1);
  usbhost_takesem(&priv->exclsem);
  priv->crefs--;

  /* Is this the last reference (other than the one held by the USB host
   * controller driver)
   */
 
  if (priv->crefs <= 1)
    {
      irqstate_t flags;

      /* Yes.. then the driver is no longer open */

      priv->open    = false;
      priv->headndx = 0;
      priv->tailndx = 0;

      /* We need to disable interrupts momentarily to assure that there are
       * no asynchronous disconnect events.
       */

      flags = irqsave();

      /* Check if the USB keyboard device is still connected.  If the device is
       * no longer connected, then unregister the driver and free the driver
       * class instance.
       */

      if (priv->disconnected)
        {
          /* Destroy the class instance (we can't use priv after this; we can't
           * 'give' the semapore)
           */

          usbhost_destroy(priv);
          irqrestore(flags);
          return OK;
        }
      irqrestore(flags);
    }
 
  usbhost_givesem(&priv->exclsem);
  return OK;
}

/****************************************************************************
 * Name: usbhost_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t usbhost_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode           *inode;
  FAR struct usbhost_state_s *priv;
  size_t                      nbytes;
  unsigned int                tail;
  int                         ret;

  uvdbg("Entry\n");
  DEBUGASSERT(filep && filep->f_inode && buffer);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv && priv->crefs > 0 && priv->crefs < USBHOST_MAX_CREFS);
  usbhost_takesem(&priv->exclsem);

  /* Check if the keyboard is still connected.  We need to disable interrupts
   * momentarily to assure that there are no asynchronous disconnect events.
   */

  if (priv->disconnected)
    {
      /* No... the driver is no longer bound to the class.  That means that
       * the USB keybaord is no longer connected.  Refuse any further attempts
       * to access the driver.
       */

      ret = -ENODEV;
    }
  else
    {
      /* Is there keyboard data now? */

      while (priv->tailndx == priv->headndx)
        {
          /* No.. were we open non-blocking? */

          if (filep->f_oflags & O_NONBLOCK)
            {
              /* Yes.. then return a failure */

              ret = -EAGAIN;
              goto errout;
            }

          /* Wait for data to be available */

          uvdbg("Waiting...\n");

          priv->waiting = true;
          usbhost_givesem(&priv->exclsem);
          usbhost_takesem(&priv->waitsem);
          usbhost_takesem(&priv->exclsem);

          /* Did the keyboard become disconnected while we were waiting */

          if (priv->disconnected)
            {
              ret = -ENODEV;
              goto errout;
            }
        }

      /* Read data from our internal buffer of received characters */
 
      for (tail  = priv->tailndx, nbytes = 0;
           tail != priv->headndx && nbytes < len;
           nbytes++)
        {
           /* Copy the next keyboard character into the user buffer */

           *buffer++ = priv->kbdbuffer[tail];

           /* Handle wrap-around of the tail index */

           if (++tail >= CONFIG_HIDKBD_BUFSIZE)
             {
               tail = 0;
             }
        }
      ret = nbytes;

      /* Update the tail index (pehaps marking the buffer empty) */

      priv->tailndx = tail;
    }

errout:
  usbhost_givesem(&priv->exclsem);
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: usbhost_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t usbhost_write(FAR struct file *filep, FAR const char *buffer, size_t len)
{
  /* We won't try to write to the keyboard */

  return -ENOSYS;
}

/****************************************************************************
 * Name: usbhost_poll
 *
 * Description:
 *   Standard character driver poll method.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int usbhost_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode           *inode;
  FAR struct usbhost_state_s *priv;
  int                         ret = OK;
  int                         i;

  uvdbg("Entry\n");
  DEBUGASSERT(filep && filep->f_inode && fds);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv);
  usbhost_takesem(&priv->exclsem);

  /* Check if the keyboard is still connected.  We need to disable interrupts
   * momentarily to assure that there are no asynchronous disconnect events.
   */

  if (priv->disconnected)
    {
      /* No... the driver is no longer bound to the class.  That means that
       * the USB keybaord is no longer connected.  Refuse any further attempts
       * to access the driver.
       */

      ret = -ENODEV;
    }
  else if (setup)
    {
      /* This is a request to set up the poll.  Find an availableslot for
       * the poll structure reference
       */

      for (i = 0; i < CONFIG_HIDKBD_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_HIDKBD_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? Notify
       * the POLLIN event if there is buffered keyboard data.
       */

      if (priv->headndx != priv->tailndx)
        {
          usbhost_pollnotify(priv);
        }
    }
  else
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot);

      /* Remove all memory of the poll setup */

      *slot                = NULL;
      fds->priv            = NULL;
    }

errout:
  sem_post(&priv->exclsem);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_kbdinit
 *
 * Description:
 *   Initialize the USB storage HID keyboard class driver.  This function
 *   should be called be platform-specific code in order to initialize and
 *   register support for the USB host HID keyboard class device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int usbhost_kbdinit(void)
{
  /* Perform any one-time initialization of the class implementation */

  sem_init(&g_exclsem, 0, 1);
  sem_init(&g_syncsem, 0, 0);

  /* Advertise our availability to support (certain) devices */

  return usbhost_registerclass(&g_skeleton);
}

#endif /* CONFIG_USBHOST)&& !CONFIG_USBHOST_INT_DISABLE && CONFIG_NFILE_DESCRIPTORS */


