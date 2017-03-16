#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/voliro_ao.h>

static volatile bool thread_should_exit = false;	/**< daemon exit flag */
static volatile bool thread_running = false;		/**< daemon status flag */
static int daemon_task;					/**< Handle of daemon task / thread */

int test_thread_main(int argc, char *argv[]);

__EXPORT int test_daemon_main(int argc, char *argv[]);

/*Might remove this*/
/******************************/
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}
/*******************************/

int test_daemon_main(int argc, char *argv[])
{

  if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}

		daemon_task = px4_task_spawn_cmd("test_daemon",
                 SCHED_DEFAULT,
                 SCHED_PRIORITY_DEFAULT + 10,
                 2600,
                 test_thread_main,
                 (char * const *)&argv[0]);

            return 0;
       }

       if (!strcmp(argv[1], "stop")) {
     		thread_should_exit = true;
     		return 0;
     	}

     	if (!strcmp(argv[1], "status")) {
     		if (thread_running) {
     			warnx("\trunning\n");

     		} else {
     			warnx("\tnot started\n");
     		}
        return 0;
      }

		      PX4_INFO("exiting");
          usage("unrecognized command");
        	return 1;
}

int test_thread_main(int argc, char *argv[])
{

	warnx("[daemon] starting\n");
	thread_running = true;

  /* advertise voliro_ao topic */
  struct voliro_ao_s volao;
  memset(&volao, 0, sizeof(volao));
  orb_advert_t volao_pub = orb_advertise(ORB_ID(voliro_ao), &volao);

	while (!thread_should_exit) {
    /*Publishing message*/

		volao.alpha[0] = 1.87f;
		volao.alpha[1] = 1.09f;
		volao.alpha[2] = 1.09f;
		volao.alpha[3] = 1.09f;
		volao.alpha[4] = 1.09f;
		volao.alpha[5] = 1.09f;

		volao.omega[0] = 10.3f;
		volao.omega[1] = 10.9f;
		volao.omega[2] = 11.1f;
		volao.omega[3] = 20.1f;
		volao.omega[4] = 2.09f;
		volao.omega[5] = 3.85f;


		orb_publish(ORB_ID(voliro_ao), volao_pub, &volao);
	}

	warnx("[daemon] exiting.\n");

	thread_running = false;

	return 0;
}
