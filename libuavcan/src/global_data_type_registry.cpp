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

    list->insert(dtd);
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

DataTypeSignature GlobalDataTypeRegistry::computeAggregateSignature(const DataTypeIDMask& msgs,
                                                                    const DataTypeIDMask& srvs) const
{
    (void)msgs;
    (void)srvs;
    return DataTypeSignature::zero(); // TODO: implementation
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
