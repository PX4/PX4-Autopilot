# Copyright (c) 2026 PX4 Development Team. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

px4_add_module(
	MODULE drivers__distance_sensor__ptgl100
	MAIN ptgl100
	SRCS
		PTGL100.cpp
		ptgl100_parser.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		drivers_rangefinder
		px4_work_queue
)

px4_add_unit_gtest(SRC PTGL100ParserTest.cpp EXTRA_SRCS ptgl100_parser.cpp)
