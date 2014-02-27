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
        DataTypeDescriptor decriptor;

        Entry() { }

        Entry(DataTypeKind kind, uint16_t id, const DataTypeSignature& signature, const char* name)
        : decriptor(kind, id, signature, name)
        { }
    };

    typedef LinkedListRoot<Entry> List;
    List msgs_;
    List srvs_;
    bool frozen_;

    GlobalDataTypeRegistry() : frozen_(false) { }

    const List* selectList(DataTypeKind kind) const;
    List* selectList(DataTypeKind kind);

    void remove(Entry* dtd);
    bool add(Entry* dtd);

public:
    static GlobalDataTypeRegistry& instance();

    /// Last call wins
    template <typename Type>
    bool assign(uint16_t id)
    {
        if (isFrozen())
            return false;
        static Entry entry;
        remove(&entry);
        entry = Entry(DataTypeKind(Type::DataTypeKind), id, Type::getDataTypeSignature(), Type::getDataTypeName());
        return add(&entry);
    }

    /// Further calls to add() will fail
    void freeze() { frozen_ = true; }
    bool isFrozen() const { return frozen_; }

    const DataTypeDescriptor* find(DataTypeKind kind, const char* name) const;

    DataTypeSignature computeAggregateSignature(const DataTypeIDMask& msgs, const DataTypeIDMask& srvs) const;

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
        if (!GlobalDataTypeRegistry::instance().assign<Type>(Type::DefaultDataTypeID))
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
