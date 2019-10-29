/*
 * Copyright (C) 2015, 2018 Pavel Kirienko <pavel.kirienko@gmail.com>
 * Kinetis Port Author David Sidrane <david_s5@nscdg.com>
 */

#pragma once

/**
 * OS detection
 */
#ifndef UAVCAN_KINETIS_CHIBIOS
# define UAVCAN_KINETIS_CHIBIOS 0
#endif

#ifndef UAVCAN_KINETIS_NUTTX
# define UAVCAN_KINETIS_NUTTX 0
#endif

#ifndef UAVCAN_KINETIS_BAREMETAL
# define UAVCAN_KINETIS_BAREMETAL 0
#endif

#ifndef UAVCAN_KINETIS_FREERTOS
# define UAVCAN_KINETIS_FREERTOS 0
#endif

/**
 * Number of interfaces must be enabled explicitly
 */
#if !defined(UAVCAN_KINETIS_NUM_IFACES) || (UAVCAN_KINETIS_NUM_IFACES != 1 && UAVCAN_KINETIS_NUM_IFACES != 2)
# error "UAVCAN_KINETIS_NUM_IFACES must be set to either 1 or 2"
#endif

/**
 * Any PIT timer channel (PIT0-PIT3)
 * e.g. -DUAVCAN_KINETIS_TIMER_NUMBER=2
 */
#ifndef UAVCAN_KINETIS_TIMER_NUMBER
// In this case the clock driver should be implemented by the application
# define UAVCAN_KINETIS_TIMER_NUMBER 0
#endif
