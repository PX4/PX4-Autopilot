#include <stdlib.h>
#include <px4_log.h>
#ifdef __PX4_POSIX
#include <execinfo.h>
#include <platforms/posix/px4_daemon/server_io.h>
#endif
#include <uORB/uORB.h>
#include <uORB/topics/log_message.h>
#include <drivers/drv_hrt.h>

static orb_advert_t orb_log_message_pub = NULL;

__EXPORT const char *__px4_log_level_str[_PX4_LOG_LEVEL_PANIC + 1] = { "DEBUG", "INFO", "WARN", "ERROR", "PANIC" };
__EXPORT const char *__px4_log_level_color[_PX4_LOG_LEVEL_PANIC + 1] =
{ PX4_ANSI_COLOR_GREEN, PX4_ANSI_COLOR_RESET, PX4_ANSI_COLOR_YELLOW, PX4_ANSI_COLOR_RED, PX4_ANSI_COLOR_RED };


void px4_log_initialize(void)
{
	ASSERT(orb_log_message_pub == NULL);

	/* we need to advertise with a valid message */
	struct log_message_s log_message;
	log_message.timestamp = hrt_absolute_time();
	log_message.severity = 6; //info
	strcpy((char *)log_message.text, "initialized uORB logging");

	orb_log_message_pub = orb_advertise_queue(ORB_ID(log_message), &log_message, 2);

	if (!orb_log_message_pub) {
		PX4_ERR("failed to advertise log_message");
	}
}

void px4_backtrace()
{
#ifdef __PX4_POSIX
	void *buffer[10];
	char **callstack;
	int bt_size;
	int idx;

	bt_size = backtrace(buffer, 10);
	callstack = backtrace_symbols(buffer, bt_size);

	PX4_INFO("Backtrace: %d", bt_size);

	for (idx = 0; idx < bt_size; idx++) {
		PX4_INFO("%s", callstack[idx]);
	}

	free(callstack);
#endif
}


__EXPORT void px4_log_modulename(int level, const char *moduleName, const char *fmt, ...)
{

#ifdef __PX4_POSIX
	char *buffer;
	unsigned max_length;
	bool is_atty = false;

	if (get_stdout_pipe_buffer(&buffer, &max_length, &is_atty) == 0) {
		if (level >= _PX4_LOG_LEVEL_INFO) {

			unsigned pos = 0;

			if (is_atty) { pos += snprintf(buffer + pos, max_length - pos, "%s", __px4_log_level_color[level]); }

			pos += snprintf(buffer + pos, max_length - pos, __px4__log_level_fmt __px4__log_level_arg(level));

			if (is_atty) { pos += snprintf(buffer + pos, max_length - pos, "%s", PX4_ANSI_COLOR_GRAY); }

			pos += snprintf(buffer + pos, max_length - pos, __px4__log_modulename_pfmt, moduleName);
			va_list argptr;

			if (is_atty) { pos += snprintf(buffer + pos, max_length - pos, "%s", __px4_log_level_color[level]); }

			va_start(argptr, fmt);
			pos += vsnprintf(buffer + pos, max_length - pos, fmt, argptr);
			va_end(argptr);
			pos += snprintf(buffer + pos, max_length - pos, "\n");

			if (is_atty) { pos += snprintf(buffer + pos, max_length - pos, "%s", PX4_ANSI_COLOR_RESET); }

			// +1 for the terminating 0 char.
			send_stdout_pipe_buffer(pos + 1);
		}

	} else {
#endif

		if (level >= _PX4_LOG_LEVEL_INFO) {
			PX4_LOG_COLOR_START
			printf(__px4__log_level_fmt __px4__log_level_arg(level));
			PX4_LOG_COLOR_MODULE
			printf(__px4__log_modulename_pfmt, moduleName);
			PX4_LOG_COLOR_MESSAGE
			va_list argptr;
			va_start(argptr, fmt);
			vprintf(fmt, argptr);
			va_end(argptr);
			PX4_LOG_COLOR_END
			printf("\n");
		}

#ifdef __PX4_POSIX
	}

#endif

	/* publish an orb log message */
	if (level >= _PX4_LOG_LEVEL_WARN && orb_log_message_pub) { //only publish important messages

		struct log_message_s log_message;
		const unsigned max_length_pub = sizeof(log_message.text);
		log_message.timestamp = hrt_absolute_time();

		const uint8_t log_level_table[] = {
			7, /* _PX4_LOG_LEVEL_DEBUG */
			6, /* _PX4_LOG_LEVEL_INFO */
			4, /* _PX4_LOG_LEVEL_WARN */
			3, /* _PX4_LOG_LEVEL_ERROR */
			0  /* _PX4_LOG_LEVEL_PANIC */
		};
		log_message.severity = log_level_table[level];

		unsigned pos = 0;

		va_list argptr;

		pos += snprintf((char *)log_message.text + pos, max_length_pub - pos, __px4__log_modulename_pfmt, moduleName);
		va_start(argptr, fmt);
		pos += vsnprintf((char *)log_message.text + pos, max_length_pub - pos, fmt, argptr);
		va_end(argptr);
		log_message.text[max_length_pub - 1] = 0; //ensure 0-termination

		orb_publish(ORB_ID(log_message), orb_log_message_pub, &log_message);
	}
}
