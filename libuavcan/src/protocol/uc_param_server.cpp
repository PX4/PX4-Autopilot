/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/protocol/param_server.hpp>
#include <uavcan/debug.hpp>
#include <cassert>
#include <cstdlib>

namespace uavcan
{

bool ParamServer::isValueNonEmpty(const protocol::param::Value& value)
{
    return !value.value_bool.empty() || !value.value_int.empty() || !value.value_float.empty();
}

void ParamServer::handleGetSet(const protocol::param::GetSet::Request& in, protocol::param::GetSet::Response& out)
{
    UAVCAN_ASSERT(manager_ != NULL);

    // Recover the name from index
    if (in.name.empty())
    {
        manager_->getParamNameByIndex(in.index, out.name);
        UAVCAN_TRACE("ParamServer", "GetSet: Index %i --> '%s'", int(in.index), out.name.c_str());
    }
    else
    {
        out.name = in.name;
    }

    // Assign if needed, read back
    if (isValueNonEmpty(in.value))
    {
        manager_->assignParamValue(out.name, in.value);
    }
    manager_->readParamValue(out.name, out.value);

    // Check if the value is OK, otherwise reset the name to indicate that we have no idea what is it all about
    if (isValueNonEmpty(out.value))
    {
        manager_->readParamDefaultMaxMin(out.name, out.default_value, out.max_value, out.min_value);
    }
    else
    {
        UAVCAN_TRACE("ParamServer", "GetSet: Unknown param: index=%i name='%s'", int(in.index), out.name.c_str());
        out.name.clear();
    }
}

void ParamServer::handleSaveErase(const protocol::param::SaveErase::Request& in,
                                  protocol::param::SaveErase::Response& out)
{
    UAVCAN_ASSERT(manager_ != NULL);

    if (in.opcode == protocol::param::SaveErase::Request::OPCODE_SAVE)
    {
        out.ok = manager_->saveAllParams() >= 0;
    }
    else if (in.opcode == protocol::param::SaveErase::Request::OPCODE_ERASE)
    {
        out.ok = manager_->eraseAllParams() >= 0;
    }
    else
    {
        UAVCAN_TRACE("ParamServer", "SaveErase: invalid opcode %i", int(in.opcode));
        out.ok = false;
    }
}

int ParamServer::start(IParamManager* manager)
{
    if (manager == NULL)
    {
        return -ErrInvalidParam;
    }
    manager_ = manager;

    int res = get_set_srv_.start(GetSetCallback(this, &ParamServer::handleGetSet));
    if (res < 0)
    {
        return res;
    }

    res = save_erase_srv_.start(SaveEraseCallback(this, &ParamServer::handleSaveErase));
    if (res < 0)
    {
        get_set_srv_.stop();
    }
    return res;
}

}
