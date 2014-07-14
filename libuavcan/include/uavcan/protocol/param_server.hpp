/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/protocol/param/GetSet.hpp>
#include <uavcan/protocol/param/SaveErase.hpp>
#include <uavcan/node/service_server.hpp>
#include <uavcan/util/method_binder.hpp>

namespace uavcan
{
/**
 * Implement this interface in the application to support the standard remote reconfiguration services.
 * Refer to @ref ParamServer.
 */
class UAVCAN_EXPORT IParamManager
{
public:
    typedef typename StorageType<typename protocol::param::GetSet::Response::FieldTypes::name>::Type ParamName;
    typedef typename StorageType<typename protocol::param::GetSet::Request::FieldTypes::index>::Type ParamIndex;
    typedef protocol::param::Value ParamValue;

    virtual ~IParamManager() { }

    /**
     * Copy the parameter name to @ref out_name if it exists, otherwise do nothing.
     */
    virtual void getParamNameByIndex(ParamIndex index, ParamName& out_name) const = 0;

    /**
     * Assign by name if exists.
     */
    virtual void assignParamValue(const ParamName& name, const ParamValue& value) = 0;

    /**
     * Read by name if exists, otherwise do nothing.
     */
    virtual void readParamValue(const ParamName& name, ParamValue& out_value) const = 0;

    /**
     * Read param's default/max/min if available.
     * Implementation is optional.
     */
    virtual void readParamDefaultMaxMin(const ParamName& name, ParamValue& out_default,
                                        ParamValue& out_max, ParamValue& out_min) const
    {
        (void)name;
        (void)out_default;
        (void)out_max;
        (void)out_min;
    }

    /**
     * Save all params to non-volatile storage.
     * @return Negative if failed.
     */
    virtual int saveAllParams() = 0;

    /**
     * Clear the non-volatile storage.
     * @return Negative if failed.
     */
    virtual int eraseAllParams() = 0;
};

/**
 * Convenience class for supporting the standard configuration services.
 * Highly recommended to use.
 */
class UAVCAN_EXPORT ParamServer
{
    typedef MethodBinder<ParamServer*, void (ParamServer::*)(const protocol::param::GetSet::Request&,
                                                             protocol::param::GetSet::Response&)> GetSetCallback;

    typedef MethodBinder<ParamServer*, void (ParamServer::*)(const protocol::param::SaveErase::Request&,
                                                             protocol::param::SaveErase::Response&)> SaveEraseCallback;

    ServiceServer<protocol::param::GetSet, GetSetCallback> get_set_srv_;
    ServiceServer<protocol::param::SaveErase, SaveEraseCallback> save_erase_srv_;
    IParamManager* manager_;

    static bool isValueNonEmpty(const protocol::param::Value& value);

    void handleGetSet(const protocol::param::GetSet::Request& request, protocol::param::GetSet::Response& response);

    void handleSaveErase(const protocol::param::SaveErase::Request& request,
                         protocol::param::SaveErase::Response& response);

public:
    explicit ParamServer(INode& node)
        : get_set_srv_(node)
        , save_erase_srv_(node)
        , manager_(NULL)
    { }

    /**
     * Starts the parameter server with given param manager instance.
     * Returns negative error code.
     */
    int start(IParamManager* manager);

    IParamManager* getParamManager() const { return manager_; }
};

}
