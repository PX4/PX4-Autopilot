#include <commander/Commander.hpp>
#include <commander/IoT-code/MovementCommander/MovementCommander.hpp>
#include <stdio.h>

static bool send_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN,
				 const float param3 = NAN,  const float param4 = NAN, const double param5 = static_cast<double>(NAN),
				 const double param6 = static_cast<double>(NAN), const float param7 = NAN)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	vcmd.param6 = param6;
	vcmd.param7 = param7;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = vehicle_status_sub.get().system_id;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = vehicle_status_sub.get().component_id;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}



// our includes:

MovementCommander::MovementCommander(){
}
MovementCommander::~MovementCommander(){
}

int MovementCommander::Movingtest(){
    printf("Arming\n");
    // possibly not used
    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);

    printf("Taking off to 2 meters\n");

    // Advertise the vehicle_command topic for publication
    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF,10.0f);

    printf("sleeping 5\n");
    sleep(5);
    // forward backwards
    printf("Forwards backwards");
    // Advertise the vehicle_command topic for publication
    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED,vehicle_command_s::SPEED_TYPE_GROUNDSPEED,2,-1);
    printf("sleeping for 5 seconds");


    printf("sleeping 5\n");
    sleep(5);
    // forward backwards
    printf("Forwards backwards");
    // Advertise the vehicle_command topic for publication
    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED,vehicle_command_s::SPEED_TYPE_CLIMB_SPEED,2,-1);
    printf("sleeping for 5 seconds");
    sleep(5);
    printf("moving back");
    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED,vehicle_command_s::SPEED_TYPE_DESCEND_SPEED,2,-1);

    // Clean up

    printf("DONE");

    return 1;

}


