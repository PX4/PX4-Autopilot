/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/node/global_data_type_registry.hpp>

namespace
{

struct DataTypeAMessage
{
    enum { DefaultDataTypeID = 0 };
    enum { DataTypeKind = uavcan::DataTypeKindMessage };
    static uavcan::DataTypeSignature getDataTypeSignature() { return uavcan::DataTypeSignature(123); }
    static const char* getDataTypeFullName() { return "my_namespace.DataTypeA"; }
};

struct DataTypeAService
{
    enum { DefaultDataTypeID = 0 };
    enum { DataTypeKind = uavcan::DataTypeKindService };
    static uavcan::DataTypeSignature getDataTypeSignature() { return uavcan::DataTypeSignature(789); }
    static const char* getDataTypeFullName() { return "my_namespace.DataTypeA"; }
};

struct DataTypeB
{
    enum { DefaultDataTypeID = 42 };
    enum { DataTypeKind = uavcan::DataTypeKindMessage };
    static uavcan::DataTypeSignature getDataTypeSignature() { return uavcan::DataTypeSignature(456); }
    static const char* getDataTypeFullName() { return "my_namespace.DataTypeB"; }
};

struct DataTypeC
{
    enum { DefaultDataTypeID = 1023 };
    enum { DataTypeKind = uavcan::DataTypeKindMessage };
    static uavcan::DataTypeSignature getDataTypeSignature() { return uavcan::DataTypeSignature(654); }
    static const char* getDataTypeFullName() { return "foo.DataTypeC"; }
};

struct DataTypeD
{
    enum { DefaultDataTypeID = 43 };
    enum { DataTypeKind = uavcan::DataTypeKindService };
    static uavcan::DataTypeSignature getDataTypeSignature() { return uavcan::DataTypeSignature(987); }
    static const char* getDataTypeFullName() { return "foo.DataTypeD"; }
};

template <typename Type>
uavcan::DataTypeDescriptor extractDescriptor(uint16_t dtid = Type::DefaultDataTypeID)
{
    return uavcan::DataTypeDescriptor(uavcan::DataTypeKind(Type::DataTypeKind), dtid,
                                      Type::getDataTypeSignature(), Type::getDataTypeFullName());
}

}


TEST(GlobalDataTypeRegistry, Basic)
{
    using uavcan::GlobalDataTypeRegistry;
    using uavcan::DataTypeIDMask;
    using uavcan::DataTypeSignature;

    GlobalDataTypeRegistry::instance().reset();
    ASSERT_FALSE(GlobalDataTypeRegistry::instance().isFrozen());
    ASSERT_EQ(0, GlobalDataTypeRegistry::instance().getNumMessageTypes());
    ASSERT_EQ(0, GlobalDataTypeRegistry::instance().getNumServiceTypes());

    DataTypeIDMask dtmask;
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
    ASSERT_EQ(GlobalDataTypeRegistry::RegistResultOk,
              GlobalDataTypeRegistry::instance().regist<DataTypeAService>(DataTypeAService::DefaultDataTypeID));

    ASSERT_EQ(2, GlobalDataTypeRegistry::instance().getNumMessageTypes());
    ASSERT_EQ(1, GlobalDataTypeRegistry::instance().getNumServiceTypes());

    GlobalDataTypeRegistry::instance().getDataTypeIDMask(uavcan::DataTypeKindService, dtmask);
    ASSERT_TRUE(dtmask[0]);
    dtmask[0] = false;
    ASSERT_FALSE(dtmask.any());

    /*
     * Runtime re-registration
     */
    ASSERT_EQ(GlobalDataTypeRegistry::RegistResultOk, GlobalDataTypeRegistry::instance().regist<DataTypeAService>(147));
    ASSERT_EQ(GlobalDataTypeRegistry::RegistResultOk, GlobalDataTypeRegistry::instance().regist<DataTypeB>(741));

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
     * These types will be necessary for the aggregate signature test
     */
    ASSERT_EQ(GlobalDataTypeRegistry::RegistResultCollision,
              GlobalDataTypeRegistry::instance().regist<DataTypeC>(741));                        // ID COLLISION

    ASSERT_EQ(GlobalDataTypeRegistry::RegistResultOk,
              GlobalDataTypeRegistry::instance().regist<DataTypeC>(DataTypeC::DefaultDataTypeID));
    uavcan::DefaultDataTypeRegistrator<DataTypeD> reg_DataTypeD;

    GlobalDataTypeRegistry::instance().getDataTypeIDMask(uavcan::DataTypeKindMessage, dtmask);
    ASSERT_TRUE(dtmask[0]);
    ASSERT_TRUE(dtmask[741]);
    ASSERT_TRUE(dtmask[1023]);
    dtmask[0] = dtmask[1023] = dtmask[741] = false;
    ASSERT_FALSE(dtmask.any());

    GlobalDataTypeRegistry::instance().getDataTypeIDMask(uavcan::DataTypeKindService, dtmask);
    ASSERT_TRUE(dtmask[147]);
    ASSERT_TRUE(dtmask[43]);
    dtmask[43] = dtmask[147] = false;
    ASSERT_FALSE(dtmask.any());

    /*
     * Frozen state
     */
    GlobalDataTypeRegistry::instance().freeze();

    ASSERT_EQ(GlobalDataTypeRegistry::RegistResultFrozen,
              GlobalDataTypeRegistry::instance().regist<DataTypeAService>(555)); // Rejected

    ASSERT_EQ(GlobalDataTypeRegistry::RegistResultFrozen,
              GlobalDataTypeRegistry::instance().regist<DataTypeAMessage>(999)); // Rejected

    ASSERT_EQ(GlobalDataTypeRegistry::RegistResultFrozen,
              GlobalDataTypeRegistry::instance().regist<DataTypeB>(888));        // Rejected

    /*
     * Searching
     */
    const uavcan::DataTypeDescriptor* pdtd = NULL;
    ASSERT_FALSE(GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindMessage, "Nonexistent"));
    ASSERT_FALSE(GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindMessage, 987));
    // Asking for service, but this is a message:
    ASSERT_FALSE(GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindService, "my_namespace.DataTypeB"));
    ASSERT_FALSE(GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindService, 42));

    ASSERT_TRUE((pdtd = GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindMessage,
                                                                "my_namespace.DataTypeB")));
    ASSERT_EQ(extractDescriptor<DataTypeB>(741), *pdtd);
    ASSERT_TRUE((pdtd = GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindMessage, 741)));
    ASSERT_EQ(extractDescriptor<DataTypeB>(741), *pdtd);

    ASSERT_TRUE((pdtd = GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindMessage,
                                                                "my_namespace.DataTypeA")));
    ASSERT_EQ(extractDescriptor<DataTypeAMessage>(), *pdtd);
    ASSERT_TRUE((pdtd = GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindMessage, uavcan::DataTypeID(0))));
    ASSERT_EQ(extractDescriptor<DataTypeAMessage>(), *pdtd);

    ASSERT_TRUE((pdtd = GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindService,
                                                                "my_namespace.DataTypeA")));
    ASSERT_EQ(extractDescriptor<DataTypeAService>(147), *pdtd);
    ASSERT_TRUE((pdtd = GlobalDataTypeRegistry::instance().find(uavcan::DataTypeKindService, 147)));
    ASSERT_EQ(extractDescriptor<DataTypeAService>(147), *pdtd);
}


TEST(GlobalDataTypeRegistry, AggregateSignature)
{
    using uavcan::GlobalDataTypeRegistry;
    using uavcan::DataTypeIDMask;
    using uavcan::DataTypeSignature;

    ASSERT_TRUE(GlobalDataTypeRegistry::instance().isFrozen());

    DataTypeIDMask mask;
    DataTypeSignature sign;

    // Zero - empty mask
    sign = GlobalDataTypeRegistry::instance().computeAggregateSignature(uavcan::DataTypeKindMessage, mask);

    ASSERT_EQ(DataTypeSignature(), sign);
    ASSERT_FALSE(mask.any());

    // All set
    mask.set();
    sign = GlobalDataTypeRegistry::instance().computeAggregateSignature(uavcan::DataTypeKindMessage, mask);
    ASSERT_TRUE(mask[0]);     // DataTypeAMessage
    ASSERT_TRUE(mask[741]);   // DataTypeB
    ASSERT_TRUE(mask[1023]);  // DataTypeC
    mask[0] = mask[741] = mask[1023] = false;
    ASSERT_FALSE(mask.any());
    {
        DataTypeSignature check_signature(DataTypeAMessage::getDataTypeSignature()); // Order matters - low --> high
        check_signature.extend(DataTypeB::getDataTypeSignature());
        check_signature.extend(DataTypeC::getDataTypeSignature());
        ASSERT_EQ(check_signature, sign);
    }

    mask.set();
    sign = GlobalDataTypeRegistry::instance().computeAggregateSignature(uavcan::DataTypeKindService, mask);
    ASSERT_TRUE(mask[43]);    // DataTypeD
    ASSERT_TRUE(mask[147]);   // DataTypeAService
    mask[43] = mask[147] = false;
    ASSERT_FALSE(mask.any());
    {
        DataTypeSignature check_signature(DataTypeD::getDataTypeSignature());
        check_signature.extend(DataTypeAService::getDataTypeSignature());
        ASSERT_EQ(check_signature, sign);
    }

    // Random
    mask[0] = mask[99] = mask[147] = mask[741] = mask[999] = mask[1022] = true;
    sign = GlobalDataTypeRegistry::instance().computeAggregateSignature(uavcan::DataTypeKindMessage, mask);
    ASSERT_TRUE(mask[0]);     // DataTypeAMessage
    ASSERT_TRUE(mask[741]);   // DataTypeB
    mask[0] = mask[741] = false;
    ASSERT_FALSE(mask.any());
    {
        DataTypeSignature check_signature(DataTypeAMessage::getDataTypeSignature()); // Order matters - low --> high
        check_signature.extend(DataTypeB::getDataTypeSignature());
        ASSERT_EQ(check_signature, sign);
    }

    // One
    mask[1] = mask[43] = true;
    sign = GlobalDataTypeRegistry::instance().computeAggregateSignature(uavcan::DataTypeKindService, mask);
    ASSERT_TRUE(mask[43]);   // DataTypeD
    mask[43] = false;
    ASSERT_FALSE(mask.any());
    ASSERT_EQ(DataTypeD::getDataTypeSignature(), sign);
}


TEST(GlobalDataTypeRegistry, Reset)
{
    using uavcan::GlobalDataTypeRegistry;

    /*
     * Since we're dealing with singleton, we need to reset it for other tests to use
     */
    ASSERT_TRUE(GlobalDataTypeRegistry::instance().isFrozen());
    GlobalDataTypeRegistry::instance().reset();
    ASSERT_FALSE(GlobalDataTypeRegistry::instance().isFrozen());
}
