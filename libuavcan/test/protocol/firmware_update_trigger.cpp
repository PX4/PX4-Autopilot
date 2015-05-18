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
    virtual bool shouldSendFirmwareUpdateRequest(uavcan::NodeID node_id,
                                                 const uavcan::protocol::GetNodeInfo::Response& node_info,
                                                 FirmwareFilePath& out_firmware_file_path)
    {
        (void)node_id;
        (void)node_info;
        (void)out_firmware_file_path;
        return false;
    }

    virtual bool shouldRetryFirmwareUpdateRequest(uavcan::NodeID node_id,
                                                  const BeginFirmwareUpdate::Response& error_response,
                                                  FirmwareFilePath& out_firmware_file_path)
    {
        (void)node_id;
        (void)error_response;
        (void)out_firmware_file_path;
        return false;
    }

    virtual void handleFirmwareUpdateConfirmation(uavcan::NodeID node_id,
                                                  const BeginFirmwareUpdate::Response& response)
    {
        (void)node_id;
        (void)response;
    }
};

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

    /*
     * Discovering one node
     */
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(2000));
}
