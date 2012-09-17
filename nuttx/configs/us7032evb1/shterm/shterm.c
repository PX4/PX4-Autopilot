/****************************************************************************
 * config/us7032evb1/shterm/shterm.c
 *
 *   Copyright(C) 2008-2009 Gregory Nutt. All rights reserved.
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <termios.h>
#include <fcntl.h>
#include <stdarg.h>
#include <errno.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Size of the circular buffer used for interrupt I/O */

#define MAX_FILEPATH 255

#define ENQ          5
#define ACK          6

#define DEFAULT_BAUD 9600

#define dbg(format, arg...)  if (debug > 0) printconsole(format, ##arg)
#define vdbg(format, arg...) if (debug > 1) printconsole(format, ##arg)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void sendfile(int fdtarg, char *filename, int verify);
static void receivefile(int fdtarg, char *filename);
static void getfilename(int fd, char *name);
static int readbyte(int fd, char *ch);
static void writebyte(int fd, char byte);
static void close_tty(void);
static void interrupt(int signo);
static void show_usage(const char *progname, int exitcode);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int debug = 0;
static int g_fd = -1;
static int g_fdnb = -1;
static FILE *g_logstream = NULL;
static const char g_dfttydev[] = "/dev/ttyS0";
static const char *g_ttydev = g_dfttydev;
static const char *g_logfile = 0;
static int g_baud = DEFAULT_BAUD;
static struct termios g_termios;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: putconsole
 ****************************************************************************/

static void putconsole(char ch)
{
  if (g_logstream)
    {
      (void)putc(ch, g_logstream);
    }
  (void)putchar(ch);
}

/****************************************************************************
 * Name: flushconsole
 ****************************************************************************/

static void flushconsole(void)
{
  if (g_logstream)
    {
      (void)fflush(g_logstream);
    }
  (void)fflush(stdout);
}

/****************************************************************************
 * Name: printconsole
 ****************************************************************************/

static void printconsole(const char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  if (g_logstream)
    {
      (void)vfprintf(g_logstream, fmt, ap);
    }
  (void)vprintf(fmt, ap);
  va_end(ap);
}

/****************************************************************************
 * Name: sendfile
 ****************************************************************************/

static void sendfile(int fdtarg, char *filename, int verify)
{
  char chin;
  char chout;
  int  fdin;
  int  nbytes;
  int  ndots;
  int  ret;

  /* Source the source file */

  fdin = open(filename, O_RDONLY);
  if (fdin < 0)
    {
      fprintf(stderr, "ERROR: Failed to open '%s' for reading\n", filename);
      (void)writebyte(fdin, '>');
      return;
    }

  if (verify)
    {
      printconsole("Verifying file '%s':\n", filename);
    }
  else
    {
      printconsole("Loading file '%s':\n", filename);
    }
  flushconsole();

  /* This loop processes each byte from the source file */

  nbytes = 0;
  ndots  = 0;
  while ((ret = readbyte(fdin, &chout)) == 1)
    {
      /* If verbose debug is OFF, then output dots at a low rate */

      if (debug < 2)
        {
          if (++nbytes > 64)
            {
              nbytes = 0;
              putconsole('.');
              if (++ndots > 72)
                {
                   putconsole('\n');
                   ndots = 0;
                }
              flushconsole();
            }
        }

      /* If verbose debug is ON, dump everything */

      else if (chout == 'S')
        {
          printconsole("\n[%c", chout);
        }
      else if (isprint(chout))
        {
          printconsole("[%c", chout);
        }
      else
        {
          printconsole("[.");
        }

      /* Send the byte to the target */

      writebyte(fdtarg, chout);

      /* Get the response from the target.  Loop until the target responds
       * by either echoing the byte sent or by sending '>'
       */

      do
        {
          ret = readbyte(fdtarg, &chin);

          /* If verbose debug is ON, echo the response from the target */

          if (ret == 1 && debug >= 2)
            {
              if (chin != chout)
                {
                  if (isprint(chin))
                    {
                      putconsole(chin);
                    }
                  else
                    {
                      putconsole('.');
                    }
                }
              else
                {
                  putconsole(']');
                }
            }

          /* Check if the target is asking to terminate the transfer */

          if (ret == 1 && chin == '>')
            {
              close(fdin);
              writebyte(fdtarg, ACK);
              return;
            }
        }
      while (ret == 1 && chin != chout);
    }

  writebyte(fdtarg, '>');
  do
   {
     ret = readbyte(fdtarg, &chin);
   }
  while (ret == 1 && chin != ENQ);
  close(fdin);
  writebyte(fdtarg, ACK);
}

/****************************************************************************
 * Name: receivefile
 ****************************************************************************/

static void receivefile(int fdtarg, char *filename)
{
  char ch;
  int  fdout;
  int  nbytes;
  int  ndots;
  int  ret;

  fdout = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fdout < 0)
    {
      fprintf(stderr, "ERROR: Failed to open '%s' for writing\n", filename);
      (void)writebyte(fdtarg, '>');
      return;
    }

  printconsole("Receiving file '%s':\n", filename);
  flushconsole();
  (void)writebyte(fdtarg, '+');

  /* Synchronize */

  do
    {
      ret = readbyte(fdtarg, &ch);
    }
  while (ret == 1 && ch != 'S' && ch != 'Q');

  nbytes = 0;
  ndots = 0;

  /* Receive the file */

  while (ret == 1)
    {
      /* Check for end-of-file */

      if (ch == '>')
        {
          close(fdout);
          return;
        }

      writebyte(fdout, ch);

      if (++nbytes > 256)
        {
          nbytes = 0;
          putconsole('.');
          if (++ndots > 72)
            {
               putconsole('\n');
               ndots = 0;
            }
            flushconsole();
        }

      ret = readbyte(fdtarg, &ch);
      if (ch == '\r')
        {
          writebyte(fdtarg, '+');
        }
    }

  close (fdout);
}

/****************************************************************************
 * Name: getfilename
 ****************************************************************************/

static void getfilename(int fd, char *name)
{
  char ch;
  int ret;

  /* Skip over spaces */

  do
    {
      ret = readbyte(fd, &ch);
    }
  while(ch == ' ' && ret == 1);

  /* Concatenate the filename */

  while(ret == 1 && ch > ' ')
    {
      *name++ = ch;
       ret = readbyte(fd, &ch);
    }
  *name++ = 0;
}

/****************************************************************************
 * Name: readbyte
 ****************************************************************************/

static int readbyte(int fd, char *ch)
{
  int ret;

  /* Read characters from the console, and echo them to the target tty */

  ret = read(fd, ch, 1);
  if(ret < 0)
    {
      if(errno != EAGAIN)
        {
          printconsole("ERROR: Failed to read from fd=%d: %s\n", fd, strerror(errno));
          close_tty();
          exit(12);
        }
      return -EAGAIN;
    }
  else if(ret > 1)
    {
      printconsole("ERROR: Unexpected number of bytes read(%d) from fd=%d\n", ret, fd);
      close_tty();
      exit(13);
    }
  return ret;
}

/****************************************************************************
 * Name: writebyte
 ****************************************************************************/

static void writebyte(int fd, char byte)
{
  int ret = write(fd, &byte, 1);
  if(ret < 0)
    {
      printconsole("ERROR: Failed to write to fd=%d: %s\n", fd, strerror(errno));
      close_tty();
      exit(14);
    }
}

/****************************************************************************
 * Name: close_tty
 ****************************************************************************/

static void close_tty(void)
{
  int ret;

  if (g_fdnb >= 0)
    {
      (void)close(g_fdnb);
    }

  if (g_fd >= 0)
    {
      ret = tcsetattr(g_fd, TCSANOW, &g_termios);
      if (ret < 0)
        {
          printconsole("ERROR: Failed to restore termios for %s: %s\n", g_ttydev, strerror(errno));
        }
      (void)close(g_fd);
    }

  if (g_logstream >= 0)
    {
      (void)fclose(g_logstream);
    }
}

/****************************************************************************
 * Name: interrupt
 ****************************************************************************/

static void interrupt(int signo)
{
  printconsole("Exit-ing...\n");
  close_tty();
  exit(0);
}

/****************************************************************************
 * Name: interrupt
 ****************************************************************************/

static void show_usage(const char *progname, int exitcode)
{
  fprintf(stderr, "\nUSAGE: %s [-h] [-d] [-t <ttyname>] [-b <baud>] [-l <log-file>]\n", progname);
  fprintf(stderr, "\nWhere:\n");
  fprintf(stderr, "\t-h: Prints this message then exit.\n");
  fprintf(stderr, "\t-d: Enable debug output (twice for verbose output).\n");
  fprintf(stderr, "\t-t <ttyname>:  Use <ttyname> device instead of %s.\n", g_dfttydev);
  fprintf(stderr, "\t-b <baud>: Use <baud> instead of %d.\n", DEFAULT_BAUD);
  fprintf(stderr, "\t-l <log-file>: Echo console output in <log-file>.\n");
  exit(exitcode);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  struct termios tty;
  char filename[MAX_FILEPATH];
  char ch;
  int  speed;
  int  opt;
  int  oflags;
  int  ret;

  while((opt = getopt(argc, argv, ":dt:b:hl:")) != -1)
    {
      switch(opt)
        {
        case 'd':
          debug++;
          break;

        case 't':
          g_ttydev = optarg;
          break;

        case 'b':
          g_baud = atoi(optarg);
          break;

        case 'h':
          show_usage(argv[0], 0);
          break;

        case 'l':
          g_logfile = optarg;
          break;

        case ':':
          fprintf(stderr, "ERROR: Missing argument to option '%c'\n", optopt);
          show_usage(argv[0], 1);
          break;

        case '?':
          fprintf(stderr, "ERROR: Unrecognized option '%c'\n", optopt);
          show_usage(argv[0], 2);
          break;
        }
    }

  if(optind < argc)
    {
      fprintf(stderr, "ERROR: Unexpected arguments at end of line\n");
      show_usage(argv[0], 3);
    }

  switch(g_baud)
    {
    case 0:      speed = B0;      break;
    case 50:     speed = B50;     break;
    case 75:     speed = B75;     break;
    case 110:    speed = B110;    break;
    case 134:    speed = B134;    break;
    case 150:    speed = B150;    break;
    case 200:    speed = B200;    break;
    case 300:    speed = B300;    break;
    case 600:    speed = B600;    break;
    case 1200:   speed = B1200;   break;
    case 1800:   speed = B1800;   break;
    case 2400:   speed = B2400;   break;
    case 4800:   speed = B4800;   break;
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;

    default:
      fprintf(stderr, "ERROR: Unsupported BAUD=%d\n", g_baud);
      show_usage(argv[0], 4);
    }

  /* Was a log file specified? */

  if (g_logfile)
    {
      g_logstream = fopen(g_logfile, "w");
      if (!g_logstream)
        {
          fprintf(stderr, "ERROR: Failed to open '%s' for writing\n", g_logfile);
          return 5;
        }
    }

  /* Set the host stdin to O_NONBLOCK */

  oflags = fcntl(0, F_GETFL, 0);
  if(oflags == -1)
    {
      fprintf(stderr, "ERROR: fnctl(F_GETFL) failed: %s\n", strerror(errno));
      return 6;
    }

  ret = fcntl(0, F_SETFL, oflags | O_NONBLOCK);
  if(ret < 0)
    {
      fprintf(stderr, "ERROR: fnctl(F_SETFL) failed: %s\n", strerror(errno));
      return 7;
    }

  /* Open the selected serial port (blocking)*/

  g_fd = open(g_ttydev, O_RDWR);
  if(g_fd < 0)
    {
      printconsole("ERROR: Failed to open %s: %s\n", g_ttydev, strerror(errno));
      return 8;
    }

  /* Configure the serial port in at the selected baud in 8-bit, no-parity, raw mode
   * and turn off echo, etc.
   */

  ret = tcgetattr(g_fd, &g_termios);
  if(ret < 0)
    {
      printconsole("ERROR: Failed to get termios for %s: %s\n", g_ttydev, strerror(errno));
      close(g_fd);
      return 9;
    }

  memcpy(&tty, &g_termios, sizeof(struct termios));
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
  tty.c_oflag &= ~OPOST;
  tty.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
  tty.c_cflag &= ~(CSIZE|PARENB);
  tty.c_cflag |= CS8;

 (void)cfsetispeed(&tty, speed);
 (void)cfsetospeed(&tty, speed);

  ret = tcsetattr(g_fd, TCSANOW, &tty);
  if(ret < 0)
    {
      printconsole("ERROR: Failed to set termios for %s: %s\n", g_ttydev, strerror(errno));
      close(g_fd);
      return 10;
    }

#if 1
  /* Open the selected serial port (non-blocking)*/

  g_fdnb = open(g_ttydev, O_RDONLY | O_NONBLOCK);
  if(g_fdnb < 0)
    {
      printconsole("ERROR: Failed to open %s: %s\n", g_ttydev, strerror(errno));
      return 11;
    }
#else
  /* Create a non-blocking copy of the configure tty descriptor */

  g_fdnb = dup(g_fd);
  if (g_fdnb < 0)
    {
      printconsole("ERROR: Failed to dup %s fd=%d: %s\n", g_ttydev, g_fd, strerror(errno));
      close_tty();
      return 12;
    }

  oflags = fcntl(g_fdnb, F_GETFL, 0);
  if(oflags == -1)
    {
      fprintf(stderr, "ERROR: fnctl(F_GETFL) failed: %s\n", strerror(errno));
      close_tty();
      return 13;
    }

  ret = fcntl(g_fdnb, F_SETFL, oflags | O_NONBLOCK);
  if(ret < 0)
    {
      fprintf(stderr, "ERROR: fnctl(F_SETFL) failed: %s\n", strerror(errno));
      close_tty();
      return 14;
    }
#endif

  /* Catch attempts to control-C out of the program so that we can restore
   * the TTY settings.
   */

  signal(SIGINT, interrupt);

  /* Loopo until control-C */

  for(;;)
    {
      /* Read characters from the console, and echo them to the target tty */

      ret = readbyte(0, &ch);
      if (ret == 0)
        {
          printconsole("End-of-file: exitting\n");
          close_tty();
          return 0;
        }
      else if (ret == 1)
        {
          writebyte(g_fd, ch);
        }

      /* Read characters from target TTY and echo them on the console */

      ret = readbyte(g_fdnb, &ch);
      if (ret == 0)
        {
          printconsole("ERROR: Unexpected number of bytes read(%d) from %s\n", ret, g_ttydev);
          close_tty();
          return 15;
        }
      else if (ret == 1)
        {
          if (ch == ENQ)
            {
              char ch1;
              char ch2;

              writebyte(g_fd, '*');
              ret = readbyte(g_fd, &ch1);
              if (ret != 1)
                {
                  printconsole("ERROR: Unexpected number of bytes read(%d) from %s\n", ret, g_ttydev);
                  close_tty();
                  return 15;
                }
              ret = readbyte(g_fd, &ch2);
              if (ret != 1)
                {
                  printconsole("ERROR: Unexpected number of bytes read(%d) from %s\n", ret, g_ttydev);
                  close_tty();
                  return 16;
                }

              getfilename(g_fd, filename);

              if (ch1 == 'l' || ch1 == 'L')
                {
                  sendfile(g_fd, filename, 0);
                }
              else if (ch1 == 'v' || ch1 == 'v')
                {
                  sendfile(g_fd, filename, 1);
                }
              else if (ch1 == 's' || ch1 == 'S')
                {
                  receivefile(g_fd, filename);
                }
            }
          else
            {
              putconsole(ch);
              flushconsole();
            }
        }
    }
  return 0;
}
