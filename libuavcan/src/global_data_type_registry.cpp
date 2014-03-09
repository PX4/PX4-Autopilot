/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <cstdlib>
#include <uavcan/global_data_type_registry.hpp>
#include <uavcan/internal/debug.hpp>

namespace uavcan
{

const GlobalDataTypeRegistry::List* GlobalDataTypeRegistry::selectList(DataTypeKind kind) const
{
    switch (kind)
    {
    case DataTypeKindMessage: return &msgs_;
    case DataTypeKindService: return &srvs_;
    default:
        assert(0);
        return NULL;
    }
}

GlobalDataTypeRegistry::List* GlobalDataTypeRegistry::selectList(DataTypeKind kind)
{
    switch (kind)
    {
    case DataTypeKindMessage: return &msgs_;
    case DataTypeKindService: return &srvs_;
    default:
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
        return RegistResultFrozen;

    List* list = selectList(dtd->descriptor.getKind());
    if (!list)
        return RegistResultInvalidParams;

    list->remove(dtd);       // If this call came from regist<>(), that would be enough
    Entry* p = list->get();  // But anyway
    while (p)
    {
        Entry* const next = p->getNextListNode();
        if (p->descriptor.match(dtd->descriptor.getKind(), dtd->descriptor.getFullName()))
            list->remove(p);
        p = next;
    }
    return RegistResultOk;
}

GlobalDataTypeRegistry::RegistResult GlobalDataTypeRegistry::registImpl(Entry* dtd)
{
    if (!dtd || (dtd->descriptor.getID() > DataTypeDescriptor::MaxDataTypeID))
    {
        assert(0);
        return RegistResultInvalidParams;
    }
    if (isFrozen())
        return RegistResultFrozen;

    List* list = selectList(dtd->descriptor.getKind());
    if (!list)
        return RegistResultInvalidParams;

    {   // Collision check
        Entry* p = list->get();
        while (p)
        {
            if (p->descriptor.getID() == dtd->descriptor.getID()) // ID collision
                return RegistResultCollision;
            if (!std::strcmp(p->descriptor.getFullName(), dtd->descriptor.getFullName())) // Name collision
                return RegistResultCollision;
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
            if (id >= p->descriptor.getID())
            {
                assert(0);
                std::abort();
            }
            id = p->descriptor.getID();
            p = p->getNextListNode();
        }
    }
#endif
    return RegistResultOk;
}

GlobalDataTypeRegistry& GlobalDataTypeRegistry::instance()
{
    static GlobalDataTypeRegistry inst;
    return inst;
}

void GlobalDataTypeRegistry::freeze()
{
    if (!frozen_)
    {
        frozen_ = true;
        UAVCAN_TRACE("GlobalDataTypeRegistry", "Frozen");
    }
}

const DataTypeDescriptor* GlobalDataTypeRegistry::find(DataTypeKind kind, const char* name) const
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
        if (p->descriptor.match(kind, name))
            return &p->descriptor;
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

        if (inout_id_mask[desc.getID()])
        {
            if (signature_initialized)
                signature.extend(desc.getSignature());
            else
                signature = DataTypeSignature(desc.getSignature());
            signature_initialized = true;
        }

        assert(prev_dtid < desc.getID());  // Making sure that list is ordered properly
        prev_dtid++;
        while (prev_dtid < desc.getID())
            inout_id_mask[prev_dtid++] = false; // Erasing bits for missing types
        assert(prev_dtid == desc.getID());

        p = p->getNextListNode();
    }
    prev_dtid++;
    while (prev_dtid <= DataTypeDescriptor::MaxDataTypeID)
        inout_id_mask[prev_dtid++] = false;

    return signature;
}

void GlobalDataTypeRegistry::getDataTypeIDMask(DataTypeKind kind, DataTypeIDMask& mask) const
{
    mask.reset();
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
        mask[p->descriptor.getID()] = true;
        p = p->getNextListNode();
    }
}

}
