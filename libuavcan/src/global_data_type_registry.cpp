/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <cstdlib>
#include <uavcan/global_data_type_registry.hpp>

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

void GlobalDataTypeRegistry::remove(Entry* dtd)
{
    assert(dtd);
    if (isFrozen())
        return;

    List* list = selectList(dtd->decriptor.getKind());
    if (!list)
        return;

    Entry* p = list->get();
    while (p)
    {
        Entry* const next = p->getNextListNode();
        if (p->decriptor.match(dtd->decriptor.getKind(), dtd->decriptor.getName()))
            list->remove(dtd);
        p = next;
    }
}

bool GlobalDataTypeRegistry::add(Entry* dtd)
{
    assert(dtd);
    if (isFrozen())
        return false;

    List* list = selectList(dtd->decriptor.getKind());
    if (!list)
        return false;

    {   // Collision check
        Entry* p = list->get();
        while (p)
        {
            if (p->decriptor.getID() == dtd->decriptor.getID()) // ID collision
                return false;
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
            if (id >= p->decriptor.getID())
            {
                assert(0);
                std::abort();
            }
            id = p->decriptor.getID();
            p = p->getNextListNode();
        }
    }
#endif
    return true;
}

GlobalDataTypeRegistry& GlobalDataTypeRegistry::instance()
{
    static GlobalDataTypeRegistry inst;
    return inst;
}

const DataTypeDescriptor* GlobalDataTypeRegistry::find(DataTypeKind kind, const char* name) const
{
    const List* list = selectList(kind);
    if (list == NULL)
    {
        assert(0);
        return NULL;
    }
    Entry* p = list->get();
    while (p)
    {
        if (p->decriptor.match(kind, name))
            return &p->decriptor;
        p = p->getNextListNode();
    }
    return NULL;
}

DataTypeSignature GlobalDataTypeRegistry::computeAggregateSignature(DataTypeKind kind,
                                                                    DataTypeIDMask& inout_id_mask) const
{
    assert(isFrozen());  // Computing the signature if the registry is not frozen is pointless

    const List* list = selectList(kind);
    if (list == NULL)
    {
        assert(0);
        return DataTypeSignature();
    }

    DataTypeSignature signature;
    bool signature_initialized = false;

    for (int id = 0; id <= DataTypeDescriptor::MaxDataTypeID; id++)
    {
        if (!inout_id_mask[id])
            continue;

        // TODO: do it faster - no need to traverse the list on each iteration
        const DataTypeDescriptor* desc = NULL;
        {
            Entry* p = list->get();
            while (p)
            {
                if (p->decriptor.match(kind, id))
                {
                    desc = &p->decriptor;
                    break;
                }
                p = p->getNextListNode();
            }
        }

        if (desc)
        {
            if (signature_initialized)
            {
                signature.extend(desc->getSignature());
            }
            else
            {
                signature_initialized = true;
                signature = DataTypeSignature(desc->getSignature());
            }
        }
        else
            inout_id_mask[id] = false;
    }
    return signature;
}

void GlobalDataTypeRegistry::getDataTypeIDMask(DataTypeKind kind, DataTypeIDMask& mask) const
{
    mask.reset();
    const List* list = selectList(kind);
    if (list == NULL)
    {
        assert(0);
        return;
    }
    Entry* p = list->get();
    while (p)
    {
        assert(p->decriptor.getKind() == kind);
        mask[p->decriptor.getID()] = true;
        p = p->getNextListNode();
    }
}

}
