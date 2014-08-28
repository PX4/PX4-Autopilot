/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <map>
#include <gtest/gtest.h>
#include <uavcan/protocol/param_server.hpp>
#include "helpers.hpp"

struct ParamServerTestManager : public uavcan::IParamManager
{
    typedef std::map<std::string, double> KeyValue;
    KeyValue kv;

    virtual void getParamNameByIndex(ParamIndex index, ParamName& out_name) const
    {
        ParamIndex current_idx = 0;
        for (KeyValue::const_iterator it = kv.begin(); it != kv.end(); ++it, ++current_idx)
        {
            if (current_idx == index)
            {
                out_name = it->first.c_str();
                break;
            }
        }
    }

    virtual void assignParamValue(const ParamName& name, const ParamValue& value)
    {
        std::cout << "ASSIGN [" << name.c_str() << "]\n" << value << "\n---" << std::endl;
        KeyValue::iterator it = kv.find(name.c_str());
        if (it != kv.end())
        {
            if (!value.value_bool.empty())
            {
                it->second = double(value.value_bool[0]);
            }
            else if (!value.value_int.empty())
            {
                it->second = double(value.value_int[0]);
            }
            else if (!value.value_float.empty())
            {
                it->second = double(value.value_float[0]);
            }
            else
            {
                assert(0);
            }
        }
    }

    virtual void readParamValue(const ParamName& name, ParamValue& out_value) const
    {
        KeyValue::const_iterator it = kv.find(name.c_str());
        if (it != kv.end())
        {
            out_value.value_float.push_back(float(it->second));
        }
        std::cout << "READ [" << name.c_str() << "]\n" << out_value << "\n---" << std::endl;
    }

    virtual int saveAllParams()
    {
        std::cout << "SAVE" << std::endl;
        return 0;
    }

    virtual int eraseAllParams()
    {
        std::cout << "ERASE" << std::endl;
        return 0;
    }
};


template <typename Client, typename Message>
static void doCall(Client& client, const Message& request, InterlinkedTestNodesWithSysClock& nodes)
{
    ASSERT_LE(0, client.call(1, request));
    ASSERT_LE(0, nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10)));
    ASSERT_TRUE(client.collector.result->isSuccessful());
}


TEST(ParamServer, Basic)
{
    InterlinkedTestNodesWithSysClock nodes;

    uavcan::ParamServer server(nodes.a);

    ParamServerTestManager mgr;

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::param::GetSet> _reg1;
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::param::SaveErase> _reg2;

    ASSERT_LE(0, server.start(&mgr));

    ServiceClientWithCollector<uavcan::protocol::param::GetSet> get_set_cln(nodes.b);
    ServiceClientWithCollector<uavcan::protocol::param::SaveErase> save_erase_cln(nodes.b);

    /*
     * Save/erase
     */
    uavcan::protocol::param::SaveErase::Request save_erase_rq;
    save_erase_rq.opcode = uavcan::protocol::param::SaveErase::Request::OPCODE_SAVE;
    doCall(save_erase_cln, save_erase_rq, nodes);
    ASSERT_TRUE(save_erase_cln.collector.result.get());
    ASSERT_TRUE(save_erase_cln.collector.result->response.ok);

    save_erase_rq.opcode = uavcan::protocol::param::SaveErase::Request::OPCODE_ERASE;
    doCall(save_erase_cln, save_erase_rq, nodes);
    ASSERT_TRUE(save_erase_cln.collector.result->response.ok);

    // Invalid opcode
    save_erase_rq.opcode = 0xFF;
    doCall(save_erase_cln, save_erase_rq, nodes);
    ASSERT_FALSE(save_erase_cln.collector.result->response.ok);

    /*
     * Get/set
     */
    uavcan::protocol::param::GetSet::Request get_set_rq;
    get_set_rq.name = "nonexistent_parameter";
    doCall(get_set_cln, get_set_rq, nodes);
    ASSERT_TRUE(get_set_cln.collector.result.get());
    ASSERT_TRUE(get_set_cln.collector.result->response.name.empty());

    // No such variable, shall return empty name/value
    get_set_rq.index = 0;
    get_set_rq.name.clear();
    get_set_rq.value.value_int.push_back(0xDEADBEEF);
    doCall(get_set_cln, get_set_rq, nodes);
    ASSERT_TRUE(get_set_cln.collector.result->response.name.empty());
    ASSERT_TRUE(get_set_cln.collector.result->response.value.value_bool.empty());
    ASSERT_TRUE(get_set_cln.collector.result->response.value.value_int.empty());
    ASSERT_TRUE(get_set_cln.collector.result->response.value.value_float.empty());

    mgr.kv["foobar"] = 123.456;    // New param

    // Get by name
    get_set_rq = uavcan::protocol::param::GetSet::Request();
    get_set_rq.name = "foobar";
    doCall(get_set_cln, get_set_rq, nodes);
    ASSERT_STREQ("foobar", get_set_cln.collector.result->response.name.c_str());
    ASSERT_TRUE(get_set_cln.collector.result->response.value.value_bool.empty());
    ASSERT_TRUE(get_set_cln.collector.result->response.value.value_int.empty());
    ASSERT_FLOAT_EQ(123.456F, get_set_cln.collector.result->response.value.value_float[0]);

    // Set by index
    get_set_rq = uavcan::protocol::param::GetSet::Request();
    get_set_rq.index = 0;
    get_set_rq.value.value_int.push_back(424242);
    doCall(get_set_cln, get_set_rq, nodes);
    ASSERT_STREQ("foobar", get_set_cln.collector.result->response.name.c_str());
    ASSERT_FLOAT_EQ(424242, get_set_cln.collector.result->response.value.value_float[0]);

    // Get by index
    get_set_rq = uavcan::protocol::param::GetSet::Request();
    get_set_rq.index = 0;
    doCall(get_set_cln, get_set_rq, nodes);
    ASSERT_STREQ("foobar", get_set_cln.collector.result->response.name.c_str());
    ASSERT_FLOAT_EQ(424242, get_set_cln.collector.result->response.value.value_float[0]);
}
