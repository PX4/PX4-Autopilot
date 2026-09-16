/*
 * Copyright (C) 2015, 2018 Pavel Kirienko <pavel.kirienko@gmail.com>
 * Kinetis Port Author David Sidrane <david_s5@nscdg.com>
 */

#pragma once

/**
 * OS detection
 */
#ifndef UAVCAN_KINETIS_NUTTX
# error "Only NuttX is supported"
#endif

/**
 * Number of interfaces must be enabled explicitly
 */
#if !defined(UAVCAN_KINETIS_NUM_IFACES) || (UAVCAN_KINETIS_NUM_IFACES != 1 && UAVCAN_KINETIS_NUM_IFACES != 2)
# error "UAVCAN_KINETIS_NUM_IFACES must be set to either 1 or 2"
#endif
