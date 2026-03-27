/****************************************************************************
 * DroneCAN Airspeed subscriber: IndicatedAirspeed (1021) -> uORB airspeed_s
 ****************************************************************************/

#pragma once

#include "UavcanSubscriberBase.hpp"

#include <uavcan/equipment/air_data/IndicatedAirspeed.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/airspeed.h>

namespace uavcannode
{

class Airspeed;

typedef uavcan::MethodBinder<Airspeed *,
        void (Airspeed::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::IndicatedAirspeed>&)>
        AirspeedBinder;

class Airspeed :
        public UavcanSubscriberBase,
        private uavcan::Subscriber<uavcan::equipment::air_data::IndicatedAirspeed, AirspeedBinder>
{
public:
        Airspeed(uavcan::INode &node) :
                UavcanSubscriberBase(uavcan::equipment::air_data::IndicatedAirspeed::DefaultDataTypeID),
                uavcan::Subscriber<uavcan::equipment::air_data::IndicatedAirspeed, AirspeedBinder>(node)
        {}

        bool init()
        {
                if (start(AirspeedBinder(this, &Airspeed::callback)) < 0) {
                        PX4_ERR("uavcan::equipment::air_data::IndicatedAirspeed subscription failed");
                        return false;
                }

                return true;
        }

        void PrintInfo() const override
        {
                printf("\t%s:%d -> %s\n",
                       uavcan::equipment::air_data::IndicatedAirspeed::getDataTypeFullName(),
                       uavcan::equipment::air_data::IndicatedAirspeed::DefaultDataTypeID,
                       _airspeed_pub.get_topic()->o_name);
        }

private:
        void callback(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::IndicatedAirspeed> &msg)
        {
                airspeed_s as{};
                as.timestamp = hrt_absolute_time();

                // libuavcan gives float m/s here
                as.indicated_airspeed_m_s = msg.indicated_airspeed;
                as.true_airspeed_m_s      = NAN;
                as.confidence             = 1.0f;

                _airspeed_pub.publish(as);
        }

        uORB::Publication<airspeed_s> _airspeed_pub{ORB_ID(airspeed)};
};

} // namespace uavcannode
