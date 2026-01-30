px4_add_module(
	MODULE modules__fw_mpc_avoidance
	MAIN fw_mpc_avoidance
	SRCS
		FwMpcAvoidance.cpp
		FwMpcAvoidance.hpp
		FwMpcDynamics.hpp
		FixedWingMpcModel.hpp
		FwMpcAero.hpp
		FwMpcController.cpp
		FwMpcController.hpp
	DEPENDS
		px4_work_queue
		osqp
	)
