/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/debug.hpp>
#include <cassert>
#include <cstdlib>

namespace uavcan
{

GlobalDataTypeRegistry GlobalDataTypeRegistry::singleton;

GlobalDataTypeRegistry::List* GlobalDataTypeRegistry::selectList(DataTypeKind kind) const
{
    if (kind == DataTypeKindMessage)
    {
        return &msgs_;
    }
    else if (kind == DataTypeKindService)
    {
        return &srvs_;
    }
    else
    {
        assert(0);
        return NULL;
    }
}

GlobalDataTypeRegistry::RegistResult GlobalDataTypeRegistry::remove(Entry* dtd)
{
    if (!dtd)
    {
        assert(0);
        return RegistResultInvalidParams;
    }
    if (isFrozen())
    {
        return RegistResultFrozen;
    }

    List* list = selectList(dtd->descriptor.getKind());
    if (!list)
    {
        return RegistResultInvalidParams;
    }

    list->remove(dtd);       // If this call came from regist<>(), that would be enough
    Entry* p = list->get();  // But anyway
    while (p)
    {
        Entry* const next = p->getNextListNode();
        if (p->descriptor.match(dtd->descriptor.getKind(), dtd->descriptor.getFullName()))
        {
            list->remove(p);
        }
        p = next;
    }
    return RegistResultOk;
}

GlobalDataTypeRegistry::RegistResult GlobalDataTypeRegistry::registImpl(Entry* dtd)
{
    if (!dtd || (dtd->descriptor.getID() > DataTypeID::Max))
    {
        assert(0);
        return RegistResultInvalidParams;
    }
    if (isFrozen())
    {
        return RegistResultFrozen;
    }

    List* list = selectList(dtd->descriptor.getKind());
    if (!list)
    {
        return RegistResultInvalidParams;
    }

    {   // Collision check
        Entry* p = list->get();
        while (p)
        {
            if (p->descriptor.getID() == dtd->descriptor.getID()) // ID collision
            {
                return RegistResultCollision;
            }
            if (!std::strncmp(p->descriptor.getFullName(), dtd->descriptor.getFullName(),
                              DataTypeDescriptor::MaxFullNameLen))                        // Name collision
            {
                return RegistResultCollision;
            }
            p = p->getNextListNode();
        }
    }
    list->insertBefore(dtd, EntryInsertionComparator(dtd));

#if UAVCAN_DEBUG
    {   // Order check
        Entry* p = list->get();
        int id = -1;
        while (p)
        {
            if (id >= p->descriptor.getID().get())
            {
                assert(0);
                std::abort();
            }
            id = p->descriptor.getID().get();
            p = p->getNextListNode();
        }
    }
#endif
    return RegistResultOk;
}

GlobalDataTypeRegistry& GlobalDataTypeRegistry::instance()
{
    return singleton;
}

void GlobalDataTypeRegistry::freeze()
{
    if (!frozen_)
    {
        frozen_ = true;
        UAVCAN_TRACE("GlobalDataTypeRegistry", "Frozen; num msgs: %u, num srvs: %u",
                     getNumMessageTypes(), getNumServiceTypes());
    }
}

const DataTypeDescriptor* GlobalDataTypeRegistry::find(DataTypeKind kind, const char* name) const
{
    if (!name)
    {
        assert(0);
        return NULL;
    }
    const List* list = selectList(kind);
    if (!list)
    {
        assert(0);
        return NULL;
    }
    Entry* p = list->get();
    while (p)
    {
        if (p->descriptor.match(kind, name))
        {
            return &p->descriptor;
        }
        p = p->getNextListNode();
    }
    return NULL;
}

const DataTypeDescriptor* GlobalDataTypeRegistry::find(DataTypeKind kind, DataTypeID dtid) const
{
    const List* list = selectList(kind);
    if (!list)
    {
        assert(0);
        return NULL;
    }
    Entry* p = list->get();
    while (p)
    {
        if (p->descriptor.match(kind, dtid))
        {
            return &p->descriptor;
        }
        p = p->getNextListNode();
    }
    return NULL;
}

DataTypeSignature GlobalDataTypeRegistry::computeAggregateSignature(DataTypeKind kind,
                                                                    DataTypeIDMask& inout_id_mask) const
{
    assert(isFrozen());  // Computing the signature if the registry is not frozen is pointless

    const List* list = selectList(kind);
    if (!list)
    {
        assert(0);
        return DataTypeSignature();
    }

    int prev_dtid = -1;
    DataTypeSignature signature;
    bool signature_initialized = false;
    Entry* p = list->get();
    while (p)
    {
        const DataTypeDescriptor& desc = p->descriptor;
        const int dtid = desc.getID().get();

        if (inout_id_mask[dtid])
        {
            if (signature_initialized)
            {
                signature.extend(desc.getSignature());
            }
            else
            {
                signature = DataTypeSignature(desc.getSignature());
            }
            signature_initialized = true;
        }

        assert(prev_dtid < dtid);  // Making sure that list is ordered properly
        prev_dtid++;
        while (prev_dtid < dtid)
        {
            inout_id_mask[prev_dtid++] = false; // Erasing bits for missing types
        }
        assert(prev_dtid == dtid);

        p = p->getNextListNode();
    }
    prev_dtid++;
    while (prev_dtid <= DataTypeID::Max)
    {
        inout_id_mask[prev_dtid++] = false;
    }

    return signature;
}

void GlobalDataTypeRegistry::getDataTypeIDMask(DataTypeKind kind, DataTypeIDMask& mask) const
{
    (void)mask.reset();
    const List* list = selectList(kind);
    if (!list)
    {
        assert(0);
        return;
    }
    Entry* p = list->get();
    while (p)
    {
        assert(p->descriptor.getKind() == kind);
        mask[p->descriptor.getID().get()] = true;
        p = p->getNextListNode();
    }
}

}
