/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/protocol/firmware_update_trigger.hpp>
#include <uavcan/protocol/node_status_provider.hpp>
#include "helpers.hpp"

using namespace uavcan::protocol::file;

struct FirmwareVersionChecker : public uavcan::IFirmwareVersionChecker
{
    unsigned should_request_cnt;
    unsigned should_retry_cnt;
    unsigned confirmation_cnt;

    std::string firmware_path;

    bool should_retry;
    std::string expected_node_name_to_update;

    BeginFirmwareUpdate::Response last_error_response;

    FirmwareVersionChecker()
        : should_request_cnt(0)
        , should_retry_cnt(0)
        , confirmation_cnt(0)
        , should_retry(false)
    { }

    virtual bool shouldRequestFirmwareUpdate(uavcan::NodeID node_id,
                                             const uavcan::protocol::GetNodeInfo::Response& node_info,
                                             FirmwareFilePath& out_firmware_file_path)
    {
        should_request_cnt++;
        std::cout << "REQUEST? " << int(node_id.get()) << "\n" << node_info << std::endl;
        out_firmware_file_path = firmware_path.c_str();
        return node_info.name == expected_node_name_to_update;
    }

    virtual bool shouldRetryFirmwareUpdate(uavcan::NodeID node_id,
                                           const BeginFirmwareUpdate::Response& error_response,
                                           FirmwareFilePath& out_firmware_file_path)
    {
        last_error_response = error_response;
        std::cout << "RETRY? " << int(node_id.get()) << "\n" << error_response << std::endl;
        should_retry_cnt++;
        out_firmware_file_path = firmware_path.c_str();
        return should_retry;
    }

    virtual void handleFirmwareUpdateConfirmation(uavcan::NodeID node_id,
                                                  const BeginFirmwareUpdate::Response& response)
    {
        confirmation_cnt++;
        std::cout << "CONFIRMED " << int(node_id.get()) << "\n" << response << std::endl;
    }
};

static uint8_t response_error_code = 0;

static void beginFirmwareUpdateRequestCallback(
    const uavcan::ReceivedDataStructure<typename BeginFirmwareUpdate::Request>& req,
    uavcan::ServiceResponseDataStructure<typename BeginFirmwareUpdate::Response>& res)
{
    std::cout << "REQUEST\n" << req << std::endl;

    res.error = response_error_code;
    res.optional_error_message = "foobar";
}


TEST(FirmwareUpdateTrigger, Basic)
{
    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<BeginFirmwareUpdate> _reg1;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GetNodeInfo> _reg2;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::NodeStatus> _reg3;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GlobalDiscoveryRequest> _reg4;

    InterlinkedTestNodesWithSysClock nodes;

    FirmwareVersionChecker checker;

    uavcan::NodeInfoRetriever node_info_retriever(nodes.a);            // On the same node

    uavcan::FirmwareUpdateTrigger trigger(nodes.a, checker);
    std::cout << "sizeof(uavcan::FirmwareUpdateTrigger): " << sizeof(uavcan::FirmwareUpdateTrigger) << std::endl;

    std::auto_ptr<uavcan::NodeStatusProvider> provider(new uavcan::NodeStatusProvider(nodes.b));    // Other node

    /*
     * Initializing
     */
    ASSERT_LE(0, trigger.start(node_info_retriever, "/path_prefix/"));

    ASSERT_LE(0, node_info_retriever.start());
    ASSERT_EQ(1, node_info_retriever.getNumListeners());

    uavcan::protocol::HardwareVersion hwver;
    hwver.unique_id[0] = 123;
    hwver.unique_id[4] = 213;
    hwver.unique_id[8] = 45;

    provider->setName("Ivan");
    provider->setHardwareVersion(hwver);

    ASSERT_LE(0, provider->startAndPublish());

    ASSERT_FALSE(trigger.isTimerRunning());
    ASSERT_EQ(0, trigger.getNumPendingNodes());

    /*
     * Updating one node
     * The server that can confirm the request is not running yet
     */
    checker.firmware_path = "firmware_path";
    checker.expected_node_name_to_update = "Ivan";
    checker.should_retry = true;

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(2000));

    ASSERT_TRUE(trigger.isTimerRunning());
    ASSERT_EQ(1, trigger.getNumPendingNodes());

    ASSERT_EQ(1, checker.should_request_cnt);
    ASSERT_EQ(0, checker.should_retry_cnt);
    ASSERT_EQ(0, checker.confirmation_cnt);

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(2000));

    // Still running!
    ASSERT_TRUE(trigger.isTimerRunning());
    ASSERT_EQ(1, trigger.getNumPendingNodes());

    /*
     * Starting the firmware update server that returns an error
     * The checker will instruct the trigger to repeat
     */
    uavcan::ServiceServer<BeginFirmwareUpdate> srv(nodes.b);

    ASSERT_LE(0, srv.start(beginFirmwareUpdateRequestCallback));

    response_error_code = BeginFirmwareUpdate::Response::ERROR_UNKNOWN;
    checker.should_retry = true;

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(1100));

    ASSERT_EQ(1, checker.should_request_cnt);
    ASSERT_EQ(1, checker.should_retry_cnt);
    ASSERT_EQ(0, checker.confirmation_cnt);

    // Still running!
    ASSERT_TRUE(trigger.isTimerRunning());
    ASSERT_EQ(1, trigger.getNumPendingNodes());

    /*
     * Trying again, this time with ERROR_IN_PROGRESS
     */
    response_error_code = BeginFirmwareUpdate::Response::ERROR_IN_PROGRESS;
    checker.should_retry = false;

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(2100));

    ASSERT_EQ(1, checker.should_request_cnt);
    ASSERT_EQ(1, checker.should_retry_cnt);
    ASSERT_EQ(1, checker.confirmation_cnt);

    // Stopped!
    ASSERT_FALSE(trigger.isTimerRunning());
    ASSERT_EQ(0, trigger.getNumPendingNodes());

    /*
     * Restarting the node info provider
     * Now it doesn't need an update
     */
    provider.reset(new uavcan::NodeStatusProvider(nodes.b));

    provider->setName("Dmitry");
    provider->setHardwareVersion(hwver);

    ASSERT_LE(0, provider->startAndPublish());

    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(2100));

    ASSERT_EQ(2, checker.should_request_cnt);
    ASSERT_EQ(1, checker.should_retry_cnt);
    ASSERT_EQ(1, checker.confirmation_cnt);

    // Stopped!
    ASSERT_FALSE(trigger.isTimerRunning());
    ASSERT_EQ(0, trigger.getNumPendingNodes());
}
