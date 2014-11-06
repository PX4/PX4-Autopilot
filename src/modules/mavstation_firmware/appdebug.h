
#ifndef __MAVSTATION_FIRMWARE_APPDEBUG_H__
#define __MAVSTATION_FIRMWARE_APPDEBUG_H__

# define debug(fmt, args...)	lowsyslog(LOG_DEBUG,fmt "\n", ##args)
/*
 * add a debug message to be printed on the console
 */
void isr_debug(uint8_t level, const char *fmt, ...);
void isr_debug_tick(void);
void show_debug_messages(void);

#endif // __MAVSTATION_FIRMWARE_APPDEBUG_H__

