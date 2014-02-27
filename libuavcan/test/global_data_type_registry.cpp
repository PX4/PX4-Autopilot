/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/global_data_type_registry.hpp>

namespace
{
    struct DataTypeAMessage
    {
        enum { DefaultDataTypeID = 0 };
        enum { DataTypeKind = uavcan::DataTypeKindMessage };
        static uavcan::DataTypeSignature getDataTypeSignature() { return uavcan::DataTypeSignature(123); }
        static const char* getDataTypeName() { return "my_namespace.DataTypeA"; }
    };

    struct DataTypeAService
    {
        enum { DefaultDataTypeID = 0 };
        enum { DataTypeKind = uavcan::DataTypeKindService };
        static uavcan::DataTypeSignature getDataTypeSignature() { return uavcan::DataTypeSignature(789); }
        static const char* getDataTypeName() { return "my_namespace.DataTypeA"; }
    };

    struct DataTypeB
    {
        enum { DefaultDataTypeID = 42 };
        enum { DataTypeKind = uavcan::DataTypeKindMessage };
        static uavcan::DataTypeSignature getDataTypeSignature() { return uavcan::DataTypeSignature(456); }
        static const char* getDataTypeName() { return "my_namespace.DataTypeB"; }
    };


    template <typename Type>
    uavcan::DataTypeDescriptor extractDescriptor(uint16_t dtid = Type::DefaultDataTypeID)
    {
        return uavcan::DataTypeDescriptor(uavcan::DataTypeKind(Type::DataTypeKind), dtid,
                                          Type::getDataTypeSignature(), Type::getDataTypeName());
    }
}


TEST(GlobalDataTypeRegistry, Basic)
{
    using uavcan::GlobalDataTypeRegistry;

    GlobalDataTypeRegistry::instance().reset();
    ASSERT_FALSE(GlobalDataTypeRegistry::instance().isFrozen());
    ASSERT_EQ(0, GlobalDataTypeRegistry::instance().getNumMessageTypes());
    ASSERT_EQ(0, GlobalDataTypeRegistry::instance().getNumServiceTypes());

    uavcan::DataTypeIDMask dtmask;
    dtmask.set();
    GlobalDataTypeRegistry::instance().getDataTypeIDMask(uavcan::DataTypeKindMessage, dtmask);
    ASSERT_FALSE(dtmask.any());
    dtmask.set();
    GlobalDataTypeRegistry::instance().getDataTypeIDMask(uavcan::DataTypeKindService, dtmask);
    ASSERT_FALSE(dtmask.any());

    /*
     * Static registrations
     */
    uavcan::DefaultDataTypeRegistrator<DataTypeAMessage> reg_DataTypeAMessage;
    uavcan::DefaultDataTypeRegistrator<DataTypeB> reg_DataTypeB;

    ASSERT_EQ(2, GlobalDataTypeRegistry::instance().getNumMessageTypes());
    ASSERT_EQ(0, GlobalDataTypeRegistry::instance().getNumServiceTypes());

    GlobalDataTypeRegistry::instance().getDataTypeIDMask(uavcan::DataTypeKindMessage, dtmask);
    ASSERT_TRUE(dtmask[0]);
    ASSERT_TRUE(dtmask[42]);
    dtmask[0] = dtmask[42] = false;
    ASSERT_FALSE(dtmask.any());

    /*
     * Runtime registrations
     */
    ASSERT_TRUE(GlobalDataTypeRegistry::instance().assign<DataTypeAService>(DataTypeAService::DefaultDataTypeID));

    ASSERT_EQ(2, GlobalDataTypeRegistry::instance().getNumMessageTypes());
    ASSERT_EQ(1, GlobalDataTypeRegistry::instance().getNumServiceTypes());

    GlobalDataTypeRegistry::instance().getDataTypeIDMask(uavcan::DataTypeKindService, dtmask);
    ASSERT_TRUE(dtmask[0]);
    dtmask[0] = false;
    ASSERT_FALSE(dtmask.any());

    /*
     * Runtime re-registration
     */
    ASSERT_TRUE(GlobalDataTypeRegistry::instance().assign<DataTypeAService>(147));
    ASSERT_TRUE(GlobalDataTypeRegistry::instance().assign<DataTypeB>(741));

    ASSERT_EQ(2, GlobalDataTypeRegistry::instance().getNumMessageTypes());
    ASSERT_EQ(1, GlobalDataTypeRegistry::instance().getNumServiceTypes());

    GlobalDataTypeRegistry::instance().getDataTypeIDMask(uavcan::DataTypeKindMessage, dtmask);
    ASSERT_TRUE(dtmask[0]);
    ASSERT_TRUE(dtmask[741]);
    dtmask[0] = dtmask[741] = false;
    ASSERT_FALSE(dtmask.any());

    GlobalDataTypeRegistry::instance().getDataTypeIDMask(uavcan::DataTypeKindService, dtmask);
    ASSERT_TRUE(dtmask[147]);
    dtmask[147] = false;
    ASSERT_FALSE(dtmask.any());

    /*
     * Frozen state
     */
    GlobalDataTypeRegistry::instance().freeze();

    ASSERT_FALSE(GlobalDataTypeRegistry::instance().assign<DataTypeAService>(555)); // Rejected
    ASSERT_FALSE(GlobalDataTypeRegistry::instance().assign<DataTypeAMessage>(999)); // Rejected
    ASSERT_FALSE(GlobalDataTypeRegistry::instance().assign<DataTypeB>(888));        // Rejected

    /*
     * Searching
     */
    const uavcan::DataTypeDescriptor* pdtd = NULL;
    ASSERT_FALSE(GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindMessage, "Nonexistent"));
    // Asking for service, but this is a message:
    ASSERT_FALSE(GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindService, "my_namespace.DataTypeB"));

    ASSERT_TRUE((pdtd = GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindMessage, "my_namespace.DataTypeB")));
    ASSERT_EQ(extractDescriptor<DataTypeB>(741), *pdtd);

    ASSERT_TRUE((pdtd = GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindMessage, "my_namespace.DataTypeA")));
    ASSERT_EQ(extractDescriptor<DataTypeAMessage>(), *pdtd);

    ASSERT_TRUE((pdtd = GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindService, "my_namespace.DataTypeA")));
    ASSERT_EQ(extractDescriptor<DataTypeAService>(147), *pdtd);

    /*
     * Aggregate signature computation
     */
    // TODO: test

    /*
     * Since we're dealing with singleton, we need to reset it for other tests to use
     */
    GlobalDataTypeRegistry::instance().reset();
}
