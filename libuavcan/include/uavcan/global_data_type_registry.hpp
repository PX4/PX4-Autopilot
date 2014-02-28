/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <bitset>
#include <stdexcept>
#include <stdint.h>
#include <algorithm>
#include <uavcan/data_type.hpp>
#include <uavcan/internal/linked_list.hpp>

namespace uavcan
{

typedef std::bitset<DataTypeDescriptor::MaxDataTypeID + 1> DataTypeIDMask;

class GlobalDataTypeRegistry : Noncopyable
{
    struct Entry : public LinkedListNode<Entry>
    {
        DataTypeDescriptor descriptor;

        Entry() { }

        Entry(DataTypeKind kind, uint16_t id, const DataTypeSignature& signature, const char* name)
        : descriptor(kind, id, signature, name)
        { }
    };

    struct EntryInsertionComparator
    {
        const uint16_t id;
        EntryInsertionComparator(Entry* dtd) : id(dtd->descriptor.getID()) { }
        bool operator()(const Entry* entry) const
        {
            assert(entry);
            return entry->descriptor.getID() > id;
        }
    };

public:
    enum RegistResult
    {
        RegistResultOk,
        RegistResultCollision,
        RegistResultInvalidParams,
        RegistResultFrozen
    };

private:
    typedef LinkedListRoot<Entry> List;
    List msgs_;
    List srvs_;
    bool frozen_;

    GlobalDataTypeRegistry() : frozen_(false) { }

    const List* selectList(DataTypeKind kind) const;
    List* selectList(DataTypeKind kind);

    RegistResult remove(Entry* dtd);
    RegistResult registImpl(Entry* dtd);

public:
    static GlobalDataTypeRegistry& instance();

    /// Last call wins
    template <typename Type>
    RegistResult regist(uint16_t id)
    {
        if (isFrozen())
            return RegistResultFrozen;

        static Entry entry;
        const RegistResult remove_res = remove(&entry);
        if (remove_res != RegistResultOk)
            return remove_res;

        entry = Entry(DataTypeKind(Type::DataTypeKind), id, Type::getDataTypeSignature(), Type::getDataTypeName());
        return registImpl(&entry);
    }

    /// Further calls to regist<>() will fail
    void freeze() { frozen_ = true; }
    bool isFrozen() const { return frozen_; }

    const DataTypeDescriptor* find(DataTypeKind kind, const char* name) const;

    DataTypeSignature computeAggregateSignature(DataTypeKind kind, DataTypeIDMask& inout_id_mask) const;

    void getDataTypeIDMask(DataTypeKind kind, DataTypeIDMask& mask) const;

    unsigned int getNumMessageTypes() const { return msgs_.getLength(); }
    unsigned int getNumServiceTypes() const { return srvs_.getLength(); }

#if UAVCAN_DEBUG
    /// Required for unit testing
    void reset()
    {
        frozen_ = false;
        while (msgs_.get())
            msgs_.remove(msgs_.get());
        while (srvs_.get())
            srvs_.remove(srvs_.get());
    }
#endif
};


template <typename Type>
struct DefaultDataTypeRegistrator
{
    DefaultDataTypeRegistrator()
    {
        const GlobalDataTypeRegistry::RegistResult res =
            GlobalDataTypeRegistry::instance().regist<Type>(Type::DefaultDataTypeID);

        if (res != GlobalDataTypeRegistry::RegistResultOk)
        {
#if UAVCAN_EXCEPTIONS
            throw std::logic_error("Type registration failed");
#else
            assert(0);
            std::abort();
#endif
        }
    }
};

}
