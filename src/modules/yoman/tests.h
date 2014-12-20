#ifndef __APPS_PX4_TESTS_H
#define __APPS_PX4_TESTS_H

/**
 * @file tests.h
 * Tests declaration file.
 *
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#    define msgflush()
#  else
#    define message(...) printf(__VA_ARGS__)
#    define msgflush() fflush(stdout)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#    define msgflush()
#  else
#    define message printf
#    define msgflush() fflush(stdout)
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

__BEGIN_DECLS

extern int	test_sensors(int argc, char *argv[]);
extern int	test_gpio(int argc, char *argv[]);
extern int	test_hrt(int argc, char *argv[]);
extern int	test_tone(int argc, char *argv[]);
extern int	test_led(int argc, char *argv[]);
extern int	test_adc(int argc, char *argv[]);
extern int	test_int(int argc, char *argv[]);
extern int	test_float(int argc, char *argv[]);
extern int	test_ppm(int argc, char *argv[]);
extern int	test_servo(int argc, char *argv[]);
extern int	test_ppm_loopback(int argc, char *argv[]);
extern int	test_uart_loopback(int argc, char *argv[]);
extern int	test_uart_baudchange(int argc, char *argv[]);
extern int	test_cpuload(int argc, char *argv[]);
extern int	test_uart_send(int argc, char *argv[]);
extern int	test_sleep(int argc, char *argv[]);
extern int	test_time(int argc, char *argv[]);
extern int	test_uart_console(int argc, char *argv[]);
extern int	test_hott_telemetry(int argc, char *argv[]);
extern int	test_jig_voltages(int argc, char *argv[]);
extern int	test_param(int argc, char *argv[]);
extern int	test_bson(int argc, char *argv[]);
extern int	test_file(int argc, char *argv[]);
extern int	test_file2(int argc, char *argv[]);
extern int	test_mixer(int argc, char *argv[]);
extern int	test_rc(int argc, char *argv[]);
extern int	test_conv(int argc, char *argv[]);
extern int	test_mount(int argc, char *argv[]);
extern int	test_mtd(int argc, char *argv[]);
extern int	test_mathlib(int argc, char *argv[]);

__END_DECLS

#endif /* __APPS_PX4_TESTS_H */
