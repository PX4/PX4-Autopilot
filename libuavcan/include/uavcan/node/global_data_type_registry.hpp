/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <bitset>
#include <algorithm>
#include <uavcan/error.hpp>
#include <uavcan/stdint.hpp>
#include <uavcan/data_type.hpp>
#include <uavcan/util/compile_time.hpp>
#include <uavcan/linked_list.hpp>
#if UAVCAN_DEBUG
# include <uavcan/debug.hpp>
#endif

namespace uavcan
{

typedef std::bitset<DataTypeID::Max + 1> DataTypeIDMask;

class UAVCAN_EXPORT GlobalDataTypeRegistry : Noncopyable
{
    struct Entry : public LinkedListNode<Entry>
    {
        DataTypeDescriptor descriptor;

        Entry() { }

        Entry(DataTypeKind kind, DataTypeID id, const DataTypeSignature& signature, const char* name)
            : descriptor(kind, id, signature, name)
        { }
    };

    struct EntryInsertionComparator
    {
        const DataTypeID id;
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
    mutable List msgs_;
    mutable List srvs_;
    bool frozen_;

    GlobalDataTypeRegistry() : frozen_(false) { }

    List* selectList(DataTypeKind kind) const;

    RegistResult remove(Entry* dtd);
    RegistResult registImpl(Entry* dtd);

public:
    static GlobalDataTypeRegistry& instance();

    /// Last call wins
    template <typename Type>
    RegistResult regist(DataTypeID id)
    {
        if (isFrozen())
        {
            return RegistResultFrozen;
        }

        static Entry entry;
        {
            const RegistResult remove_res = remove(&entry);
            if (remove_res != RegistResultOk)
            {
                return remove_res;
            }
        }
        entry = Entry(DataTypeKind(Type::DataTypeKind), id, Type::getDataTypeSignature(), Type::getDataTypeFullName());
        {
            const RegistResult remove_res = remove(&entry);
            if (remove_res != RegistResultOk)
            {
                return remove_res;
            }
        }
        return registImpl(&entry);
    }

    /// Further calls to regist<>() will fail
    void freeze();
    bool isFrozen() const { return frozen_; }

    const DataTypeDescriptor* find(DataTypeKind kind, const char* name) const;
    const DataTypeDescriptor* find(DataTypeKind kind, DataTypeID dtid) const;

    DataTypeSignature computeAggregateSignature(DataTypeKind kind, DataTypeIDMask& inout_id_mask) const;

    void getDataTypeIDMask(DataTypeKind kind, DataTypeIDMask& mask) const;

    unsigned getNumMessageTypes() const { return msgs_.getLength(); }
    unsigned getNumServiceTypes() const { return srvs_.getLength(); }

#if UAVCAN_DEBUG
    /// Required for unit testing
    void reset()
    {
        UAVCAN_TRACE("GlobalDataTypeRegistry", "Reset; was frozen: %i, num msgs: %u, num srvs: %u",
                     int(frozen_), getNumMessageTypes(), getNumServiceTypes());
        frozen_ = false;
        while (msgs_.get())
        {
            msgs_.remove(msgs_.get());
        }
        while (srvs_.get())
        {
            srvs_.remove(srvs_.get());
        }
    }
#endif
};


template <typename Type>
struct UAVCAN_EXPORT DefaultDataTypeRegistrator
{
    DefaultDataTypeRegistrator()
    {
        const GlobalDataTypeRegistry::RegistResult res =
            GlobalDataTypeRegistry::instance().regist<Type>(Type::DefaultDataTypeID);

        if (res != GlobalDataTypeRegistry::RegistResultOk)
        {
            handleFatalError("Type registration failed");
        }
    }
};

}
