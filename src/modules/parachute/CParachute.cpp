/**
 * @file parachute.cpp
 *
 * @date 01.04.2015
 *
 * @author boehmt@rgoesspace.net
 */


#include "CParachute.hpp"

/**
 * parachute app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int parachute_main(int argc, char *argv[]);


namespace Parachute
{
	CParachute	*g_parachute = nullptr;
}


CParachute::CParachute() :
	m_task_should_exit(true),
	m_parachute_task(-1),
	m_hil_enabled(false),
	m_cStateMachine(nullptr)
{
	/* performance counters */
	m_loop_perf = perf_alloc(PC_ELAPSED, "parachute task update");
}


CParachute::~CParachute()
{
	if(nullptr != m_cStateMachine)
	{
		delete m_cStateMachine;
		m_cStateMachine = nullptr;
	}

	if (m_parachute_task != -1)
	{
		/* task wakes up every 100ms or so at the longest */
		m_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do
		{
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50)
			{
				task_delete((short)m_parachute_task);
				break;
			}
		}
		while (m_parachute_task != -1);
	}

	Parachute::g_parachute = nullptr;
}


void
CParachute::task_main_trampoline(int argc, char *argv[])
{
	Parachute::g_parachute->task_main();
}


void
CParachute::task_main()
{
	hrt_abstime l_tCurrentTime = 0,
				l_tElapsedTime = 0;

	warnx("[daemon] starting.");

	m_cStateMachine = new CStateMachine();

	m_cStateMachine->init();


	m_task_should_exit = false;

	while (!m_task_should_exit)
	{
		l_tCurrentTime = hrt_absolute_time();

		perf_begin(m_loop_perf);

		m_cStateMachine->run();

		perf_end(m_loop_perf);

		l_tElapsedTime = hrt_absolute_time() - l_tCurrentTime;
		// limit loop rate
		if( 0 < (l_tElapsedTime - 1000 * PARACHUTE_TASK_DELAY_MS) )
		{
			usleep(l_tElapsedTime - 1000 * PARACHUTE_TASK_DELAY_MS);
		}
	}

	warnx("[daemon] exiting.");
}


int
CParachute::start()
{
	ASSERT(m_parachute_task == -1);


	/* start the task */
	m_parachute_task = task_spawn_cmd("parachute_task",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_DEFAULT - 20,
				       2048,
				       (main_t)&CParachute::task_main_trampoline,
				       nullptr);

	/* wait until the task is up and running or has failed */
	while (m_parachute_task > 0 && m_task_should_exit)
	{
		usleep(100);
	}

	if (m_parachute_task < 0)
	{
		return -ERROR;
	}

	return OK;
}


int parachute_main(int argc, char *argv[])
{
	if (argc < 1)
	{
		errx(1, "usage: parachute {start|stop|status}");
	}

	if (!strcmp(argv[1], "start"))
	{
		if (Parachute::g_parachute != nullptr)
		{
			errx(0, "already running");
		}

		Parachute::g_parachute = new CParachute();

		if (Parachute::g_parachute == nullptr)
		{
			errx(1, "alloc failed");
		}

		if (OK != Parachute::g_parachute->start())
		{
			delete Parachute::g_parachute;
			Parachute::g_parachute = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop"))
	{
		if (Parachute::g_parachute == nullptr)
		{
			errx(1, "not running");
		}

		delete Parachute::g_parachute;
		Parachute::g_parachute = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status"))
	{
		if (Parachute::g_parachute)
		{
			errx(0, "is running in state: %d", Parachute::g_parachute->getState());
		}
		else
		{
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
