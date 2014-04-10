/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/transport/outgoing_transfer_registry.hpp>

namespace uavcan
{
/*
 * OutgoingTransferRegistryKey
 */
#if UAVCAN_TOSTRING
std::string OutgoingTransferRegistryKey::toString() const
{
    using namespace std;
    char buf[40];
    (void)snprintf(buf, sizeof(buf), "dtid=%u tt=%u dnid=%u",
                   int(data_type_id_.get()), int(transfer_type_), int(destination_node_id_.get()));
    return std::string(buf);
}
#endif

}
