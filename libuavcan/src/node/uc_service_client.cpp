/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/node/service_client.hpp>

namespace uavcan
{
/*
 * ServiceClientBase::CallState
 */
void ServiceClientBase::CallState::handleDeadline(MonotonicTime)
{
    UAVCAN_ASSERT(id_.isValid());
    UAVCAN_TRACE("ServiceClient", "Timeout from nid=%d, tid=%d, dtname=%s",
                 int(id_.server_node_id.get()), int(id_.transfer_id.get()),
                 (owner_.data_type_descriptor_ == NULL) ? "???" : owner_.data_type_descriptor_->getFullName());
    owner_.handleTimeout(id_);
}

/*
 * ServiceClientBase
 */
int ServiceClientBase::prepareToCall(INode& node,
                                     const char* dtname,
                                     NodeID server_node_id,
                                     ServiceCallID& out_call_id)
{
    /*
     * Making sure we're not going to get transport error because of invalid input data
     */
    if (!server_node_id.isUnicast() || (server_node_id == node.getNodeID()))
    {
        UAVCAN_TRACE("ServiceClient", "Invalid Server Node ID");
        return -ErrInvalidParam;
    }
    out_call_id.server_node_id = server_node_id;

    /*
     * Determining the Data Type ID
     */
    if (data_type_descriptor_ == NULL)
    {
        GlobalDataTypeRegistry::instance().freeze();
        data_type_descriptor_ = GlobalDataTypeRegistry::instance().find(DataTypeKindService, dtname);
        if (data_type_descriptor_ == NULL)
        {
            UAVCAN_TRACE("ServiceClient", "Type [%s] is not registered", dtname);
            return -ErrUnknownDataType;
        }
        UAVCAN_TRACE("ServiceClient", "Data type descriptor inited: %s", data_type_descriptor_->toString().c_str());
    }
    UAVCAN_ASSERT(data_type_descriptor_ != NULL);

    /*
     * Determining the Transfer ID
     */
    const OutgoingTransferRegistryKey otr_key(data_type_descriptor_->getID(),
                                              TransferTypeServiceRequest, server_node_id);
    const MonotonicTime otr_deadline = node.getMonotonicTime() + TransferSender::getDefaultMaxTransferInterval();
    TransferID* const otr_tid =
        node.getDispatcher().getOutgoingTransferRegistry().accessOrCreate(otr_key, otr_deadline);
    if (!otr_tid)
    {
        UAVCAN_TRACE("ServiceClient", "OTR access failure, dtd=%s", data_type_descriptor_->toString().c_str());
        return -ErrMemory;
    }
    out_call_id.transfer_id = *otr_tid;
    otr_tid->increment();

    return 0;
}

}
