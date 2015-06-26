/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>,
 *                    Ilia  Sheremet <illia.sheremet@gmail.com>
 */

#include <gtest/gtest.h>
#include <cassert>

#include <uavcan/transport/can_acceptance_filter_configurator.hpp>
#include "../node/test_node.hpp"
#include "uavcan/node/subscriber.hpp"
#include <uavcan/equipment/camera_gimbal/AngularCommand.hpp>
#include <uavcan/equipment/air_data/Sideslip.hpp>
#include <uavcan/equipment/air_data/TrueAirspeed.hpp>
#include <uavcan/equipment/air_data/AngleOfAttack.hpp>
#include <uavcan/equipment/ahrs/AHRS.hpp>
#include <uavcan/equipment/air_data/StaticPressure.hpp>
#include <uavcan/protocol/file/BeginFirmwareUpdate.hpp>
#include <uavcan/node/service_client.hpp>
#include <uavcan/node/service_server.hpp>
#include <iostream>
#include <bitset>

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11

template <typename DataType>
struct SubscriptionListener
{
    typedef uavcan::ReceivedDataStructure<DataType> ReceivedDataStructure;

    struct ReceivedDataStructureCopy
    {
        uavcan::MonotonicTime ts_monotonic;
        uavcan::UtcTime ts_utc;
        uavcan::TransferType transfer_type;
        uavcan::TransferID transfer_id;
        uavcan::NodeID src_node_id;
        uavcan::uint8_t iface_index;
        DataType msg;

        ReceivedDataStructureCopy(const ReceivedDataStructure& s) :
            ts_monotonic(s.getMonotonicTimestamp()),
            ts_utc(s.getUtcTimestamp()),
            transfer_type(s.getTransferType()),
            transfer_id(s.getTransferID()),
            src_node_id(s.getSrcNodeID()),
            iface_index(s.getIfaceIndex()),
            msg(s)
        { }
    };

    std::vector<DataType> simple;
    std::vector<ReceivedDataStructureCopy> extended;

    void receiveExtended(ReceivedDataStructure& msg)
    {
        extended.push_back(msg);
    }

    void receiveSimple(DataType& msg)
    {
        simple.push_back(msg);
    }

    typedef SubscriptionListener<DataType> SelfType;
    typedef uavcan::MethodBinder<SelfType*, void(SelfType::*) (ReceivedDataStructure&)> ExtendedBinder;
    typedef uavcan::MethodBinder<SelfType*, void(SelfType::*) (DataType&)> SimpleBinder;

    ExtendedBinder bindExtended() { return ExtendedBinder(this, &SelfType::receiveExtended); }
    SimpleBinder bindSimple() { return SimpleBinder(this, &SelfType::receiveSimple); }
};

static void writeServiceServerCallback(
    const uavcan::ReceivedDataStructure<uavcan::protocol::file::BeginFirmwareUpdate::Request>& req,
    uavcan::protocol::file::BeginFirmwareUpdate::Response& rsp)
{
    std::cout << req << std::endl;
    rsp.error = rsp.ERROR_UNKNOWN;
}

TEST(CanAcceptanceFilter, Basic_test)
{
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::equipment::camera_gimbal::AngularCommand> _reg1;
    uavcan::DefaultDataTypeRegistrator<uavcan::equipment::air_data::Sideslip> _reg2;
    uavcan::DefaultDataTypeRegistrator<uavcan::equipment::air_data::TrueAirspeed> _reg3;
    uavcan::DefaultDataTypeRegistrator<uavcan::equipment::air_data::AngleOfAttack> _reg4;
    uavcan::DefaultDataTypeRegistrator<uavcan::equipment::ahrs::AHRS> _reg5;
    uavcan::DefaultDataTypeRegistrator<uavcan::equipment::air_data::StaticPressure> _reg6;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::file::BeginFirmwareUpdate> _reg7;

    SystemClockDriver clock_driver;
    CanDriverMock can_driver(1, clock_driver);
    TestNode node(can_driver, clock_driver, 24);

    uavcan::Subscriber<uavcan::equipment::camera_gimbal::AngularCommand,
                    SubscriptionListener<uavcan::equipment::camera_gimbal::AngularCommand>::ExtendedBinder> sub_1(node);
    uavcan::Subscriber<uavcan::equipment::air_data::Sideslip,
                       SubscriptionListener<uavcan::equipment::air_data::Sideslip>::ExtendedBinder> sub_2(node);
    uavcan::Subscriber<uavcan::equipment::air_data::TrueAirspeed,
                       SubscriptionListener<uavcan::equipment::air_data::TrueAirspeed>::ExtendedBinder> sub_3(node);
    uavcan::Subscriber<uavcan::equipment::air_data::AngleOfAttack,
                       SubscriptionListener<uavcan::equipment::air_data::AngleOfAttack>::ExtendedBinder> sub_4(node);
    uavcan::Subscriber<uavcan::equipment::ahrs::AHRS,
                       SubscriptionListener<uavcan::equipment::ahrs::AHRS>::ExtendedBinder> sub_5(node);
    uavcan::Subscriber<uavcan::equipment::air_data::StaticPressure,
                       SubscriptionListener<uavcan::equipment::air_data::StaticPressure>::ExtendedBinder> sub_6(node);
    uavcan::Subscriber<uavcan::equipment::air_data::StaticPressure,
                       SubscriptionListener<uavcan::equipment::air_data::StaticPressure>::ExtendedBinder> sub_6_1(node);
    uavcan::ServiceServer<uavcan::protocol::file::BeginFirmwareUpdate> server(node);

    SubscriptionListener<uavcan::equipment::camera_gimbal::AngularCommand> listener_1;
    SubscriptionListener<uavcan::equipment::air_data::Sideslip> listener_2;
    SubscriptionListener<uavcan::equipment::air_data::TrueAirspeed> listener_3;
    SubscriptionListener<uavcan::equipment::air_data::AngleOfAttack> listener_4;
    SubscriptionListener<uavcan::equipment::ahrs::AHRS> listener_5;
    SubscriptionListener<uavcan::equipment::air_data::StaticPressure> listener_6;

    sub_1.start(listener_1.bindExtended());
    sub_2.start(listener_2.bindExtended());
    sub_3.start(listener_3.bindExtended());
    sub_4.start(listener_4.bindExtended());
    sub_5.start(listener_5.bindExtended());
    sub_6.start(listener_6.bindExtended());
    sub_6_1.start(listener_6.bindExtended());
    server.start(writeServiceServerCallback);
    std::cout << "Subscribers are initialized ..." << std::endl;

    uavcan::CanAcceptanceFilterConfigurator test_configurator(node);
    int configure_filters_assert = test_configurator.configureFilters();
    if (configure_filters_assert == 0)
    {
        std::cout << "Filters are configured ..." << std::endl;
    }

    const auto& configure_array = test_configurator.getConfiguration();
    uint32_t configure_array_size = configure_array.getSize();

    ASSERT_EQ(configure_filters_assert, 0);
    ASSERT_EQ(configure_array_size, 4);

    for (uint16_t i = 0; i<configure_array_size; i++)
    {
        std::cout << "config.ID [" << i << "]= " << configure_array.getByIndex(i)->id << std::endl;
        std::cout << "config.MK [" << i << "]= " << configure_array.getByIndex(i)->mask << std::endl;
    }

    ASSERT_EQ(configure_array.getByIndex(0)->id, 268435456);
    ASSERT_EQ(configure_array.getByIndex(0)->mask, 469762048);
    ASSERT_EQ(configure_array.getByIndex(1)->id, 363069440);
    ASSERT_EQ(configure_array.getByIndex(1)->mask, 536739840);
    ASSERT_EQ(configure_array.getByIndex(2)->id, 16777216);
    ASSERT_EQ(configure_array.getByIndex(2)->mask, 124452864);
    ASSERT_EQ(configure_array.getByIndex(3)->id, 18874368);
    ASSERT_EQ(configure_array.getByIndex(3)->mask, 133169152);
}
#endif
