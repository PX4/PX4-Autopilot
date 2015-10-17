/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>,
 *                    Ilia  Sheremet <illia.sheremet@gmail.com>
 */

#ifndef UAVCAN_ACCEPTANCE_FILTER_CONFIGURATOR_HPP_INCLUDED
#define UAVCAN_ACCEPTANCE_FILTER_CONFIGURATOR_HPP_INCLUDED

#include <cassert>
#include <uavcan/data_type.hpp>
#include <uavcan/error.hpp>
#include <uavcan/transport/dispatcher.hpp>
#include <uavcan/node/abstract_node.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/util/multiset.hpp>

namespace uavcan
{
/**
 * This class configures hardware acceptance filters (if this feature is present on the particular CAN driver) to
 * preclude reception of irrelevant CAN frames on the hardware level.
 *
 * Configuration starts by creating an object of class @ref CanAcceptanceFilterConfigurator on the stack.
 * Through the method configureFilters() it determines the number of available HW filters and the number
 * of listeners. In case the number of listeners is higher than the number of available HW filters, the function
 * automatically merges configs in the most efficient way until their number is reduced to the number of
 * available HW filters. Subsequently obtained configurations are then loaded into the CAN driver.
 *
 * The maximum number of CAN acceptance filters is predefined in uavcan/build_config.hpp through a constant
 * @ref MaxCanAcceptanceFilters. The algorithm doesn't allow to have higher number of HW filters configurations than
 * defined by MaxCanAcceptanceFilters. You can change this value according to the number specified in your CAN driver
 * datasheet.
 */
class CanAcceptanceFilterConfigurator
{
public:
    /**
     * These arguments defines whether acceptance filter configuration has anonymous messages or not
     */
    enum AnonymousMessages
    {
        AcceptAnonymousMessages,
        IgnoreAnonymousMessages
    };

private:
    /**
     * Below constants based on UAVCAN transport layer specification. Masks and ID's depends on message
     * TypeID, TransferID (RequestNotResponse - for service types, ServiceNotMessage - for all types of messages).
     * For more details refer to uavcan.org/CAN_bus_transport_layer_specification.
     * For clarity let's represent "i" as Data Type ID and "d" as Destination Node Id
     * DefaultFilterMsgMask = 00000 11111111 11111111 10000000
     * DefaultFilterMsgID   = 00000 iiiiiiii iiiiiiii 00000000, no need to explicitly define, since MsgID initialized
     * as 0.
     * DefaultFilterServiceMask = 00000 00000000 01111111 10000000
     * DefaultFilterServiceID   = 00000 00000000 0ddddddd 10000000, all Service Response Frames are accepted by
     * HW filters.
     * DefaultAnonMsgMask = 00000 00000000 00000000 11111111
     * DefaultAnonMsgID   = 00000 00000000 00000000 00000000, by default the config is added to accept all anonymous
     * frames. In case there are no anonymous messages, invoke configureFilters(IgnoreAnonymousMessages).
     */
    static const unsigned DefaultFilterMsgMask = 0xFFFF80;
    static const unsigned DefaultFilterServiceMask = 0x7F80;
    static const unsigned DefaultFilterServiceID = 0x80;
    static const unsigned DefaultAnonMsgMask = 0xFF;
    static const unsigned DefaultAnonMsgID = 0x0;

    typedef uavcan::Multiset<CanFilterConfig> MultisetConfigContainer;

    static CanFilterConfig mergeFilters(CanFilterConfig &a_, CanFilterConfig &b_);
    static uint8_t countBits(uint32_t n_);
    uint16_t getNumFilters() const;

    /**
     * Fills the multiset_configs_ to proceed it with computeConfiguration()
     */
    int16_t loadInputConfiguration(AnonymousMessages load_mode);

    /**
     * This method merges several listeners's filter configurations by predetermined algorithm
     * if number of available hardware acceptance filters less than number of listeners
     */
    int16_t computeConfiguration();

    /**
     * This method loads the configuration computed with computeConfiguration() to the CAN driver.
     */
    int16_t applyConfiguration();

    INode& node_;               //< Node reference is needed for access to ICanDriver and Dispatcher
    MultisetConfigContainer multiset_configs_;

public:
    explicit CanAcceptanceFilterConfigurator(INode& node)
        : node_(node)
        , multiset_configs_(node.getAllocator())
    { }

    /**
     * This method invokes loadInputConfiguration(), computeConfiguration() and applyConfiguration() consequently, so
     * that optimal acceptance filter configuration will be computed and loaded through CanDriver::configureFilters()
     *
     * @param mode Either: AcceptAnonymousMessages - the filters will accept all anonymous messages (this is default)
     *                     IgnoreAnonymousMessages - anonymous messages will be ignored
     * @return 0 = success, negative for error.
     */
    int configureFilters(AnonymousMessages mode = AcceptAnonymousMessages);

    /**
     * Returns the configuration computed with computeConfiguration().
     * If computeConfiguration() has not been called yet, an empty configuration will be returned.
     */
    const MultisetConfigContainer& getConfiguration() const
    {
        return multiset_configs_;
    }
};

}

#endif // UAVCAN_BUILD_CONFIG_HPP_INCLUDED
