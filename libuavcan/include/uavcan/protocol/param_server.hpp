/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_PARAM_SERVER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_PARAM_SERVER_HPP_INCLUDED

#include <uavcan/protocol/param/GetSet.hpp>
#include <uavcan/protocol/param/ExecuteOpcode.hpp>
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

    typedef MethodBinder<ParamServer*,
                         void (ParamServer::*)(const protocol::param::ExecuteOpcode::Request&,
                                               protocol::param::ExecuteOpcode::Response&)> ExecuteOpcodeCallback;

public:
    /**
     * This class can automatically enforce that parameter names are not case-sensitive, by means of
     * automatic conversion to either upper or lower case whenever a parameter name is received.
     * For instance, if the lower-case conversion is enabled, then a UAVCAN request for a parameter
     * named "Foo" will be passed to the application as "foo". Similarly, when the application reports
     * that it has a parameter named "FOO", the class will convert the name to "foo".
     */
    enum ParamNameCaseConversion
    {
        ParamNameCaseConversionDisabled,        ///< Do not convert parameter names. The case will remain intact.
        ParamNameCaseConversionToLower,         ///< Convert parameter names to lower case.
        ParamNameCaseConversionToUpper          ///< Convert parameter names to upper case.
    };

private:
    ServiceServer<protocol::param::GetSet, GetSetCallback> get_set_srv_;
    ServiceServer<protocol::param::ExecuteOpcode, ExecuteOpcodeCallback> save_erase_srv_;
    IParamManager* manager_;
    const ParamNameCaseConversion param_name_case_conversion_mode_;

    static bool isValueNonEmpty(const protocol::param::Value& value);

    void convertParamNameCase(IParamManager::ParamName& name) const;

    void handleGetSet(const protocol::param::GetSet::Request& request, protocol::param::GetSet::Response& response);

    void handleExecuteOpcode(const protocol::param::ExecuteOpcode::Request& request,
                             protocol::param::ExecuteOpcode::Response& response);

public:
    /**
     * @param param_name_case_conversion    Specifies the parameter name conversion mode. Lower case is default,
     *                                      which means that an external request for a parameter named "Foo" will
     *                                      be relayed to the application as "foo".
     */
    explicit ParamServer(INode& node,
                         ParamNameCaseConversion param_name_case_conversion = ParamNameCaseConversionToLower)
        : get_set_srv_(node)
        , save_erase_srv_(node)
        , manager_(NULL)
        , param_name_case_conversion_mode_(param_name_case_conversion)
    { }

    /**
     * Starts the parameter server with given param manager instance.
     * Returns negative error code.
     */
    int start(IParamManager* manager);

    IParamManager* getParamManager() const { return manager_; }
};

}

#endif // UAVCAN_PROTOCOL_PARAM_SERVER_HPP_INCLUDED
