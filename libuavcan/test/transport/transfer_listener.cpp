/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <algorithm>
#include <queue>
#include <gtest/gtest.h>
#include <uavcan/internal/transport/transfer_listener.hpp>


struct Transfer
{
    uint64_t ts_monotonic;
    uint64_t ts_utc;
    uavcan::TransferType transfer_type;
    uavcan::TransferID transfer_id;
    uint8_t source_node_id;
    std::string payload;

    Transfer(const uavcan::IncomingTransfer& tr)
    : ts_monotonic(tr.getMonotonicTimestamp())
    , ts_utc(tr.getUtcTimestamp())
    , transfer_type(tr.getTransferType())
    , transfer_id(tr.getTransferID())
    , source_node_id(tr.getSourceNodeID())
    {
        unsigned int offset = 0;
        while (true)
        {
            uint8_t buf[256];
            int res = tr.read(offset, buf, sizeof(buf));
            if (res < 0)
            {
                std::cout << "IncomingTransferContainer: read failure " << res << std::endl;
                exit(1);
            }
            if (res == 0)
                break;
            payload += std::string(reinterpret_cast<const char*>(buf), res);
            offset += res;
        }
    }

    Transfer(uint64_t ts_monotonic, uint64_t ts_utc, uavcan::TransferType transfer_type,
             uavcan::TransferID transfer_id, uint8_t source_node_id, const std::string& payload)
    : ts_monotonic(ts_monotonic)
    , ts_utc(ts_utc)
    , transfer_type(transfer_type)
    , transfer_id(transfer_id)
    , source_node_id(source_node_id)
    , payload(payload)
    { }

    bool operator==(const Transfer& rhs) const
    {
        return
            (ts_monotonic   == rhs.ts_monotonic) &&
            (ts_utc         == rhs.ts_utc) &&
            (transfer_type  == rhs.transfer_type) &&
            (transfer_id    == rhs.transfer_id) &&
            (source_node_id == rhs.source_node_id) &&
            (payload        == rhs.payload);
    }

    std::string toString() const
    {
        std::ostringstream os;
        os << "ts_m="     << ts_monotonic
            << " ts_utc=" << ts_utc
            << " tt="     << transfer_type
            << " tid="    << int(transfer_id.get())
            << " snid="   << int(source_node_id)
            << "\n\t'" << payload << "'";
        return os.str();
    }
};


TEST(TransferListener, TestingEnvironmentTransfer)
{
    uavcan::PoolAllocator<uavcan::MEM_POOL_BLOCK_SIZE * 8, uavcan::MEM_POOL_BLOCK_SIZE> pool;
    uavcan::PoolManager<1> poolmgr;
    poolmgr.addPool(&pool);

    uavcan::TransferBufferManager<128, 1> mgr(&poolmgr);
    uavcan::TransferBufferAccessor tba(&mgr, uavcan::TransferBufferManagerKey(0, uavcan::TRANSFER_TYPE_MESSAGE_UNICAST));

    uavcan::RxFrame frame;
    frame.last_frame = true;
    uavcan::MultiFrameIncomingTransfer mfit(10, 1000, frame, tba);

    // Filling the buffer with data
    static const std::string TEST_DATA = "Kaneda! What do you see? Kaneda! What do you see? Kaneda! Kaneda!!!";
    ASSERT_TRUE(tba.create());
    ASSERT_EQ(TEST_DATA.length(),
              tba.access()->write(0, reinterpret_cast<const uint8_t*>(TEST_DATA.c_str()), TEST_DATA.length()));

    // Reading back
    const Transfer transfer(mfit);
    ASSERT_EQ(TEST_DATA, transfer.payload);
}


static std::vector<uavcan::RxFrame> serializeTransfer(const Transfer& transfer, uint8_t target_node_id,
                                                      const uavcan::DataTypeDescriptor& type)
{
    uavcan::Crc16 payload_crc(type.hash.value, uavcan::DataTypeHash::NUM_BYTES);
    payload_crc.add(reinterpret_cast<const uint8_t*>(transfer.payload.c_str()), transfer.payload.length());

    std::vector<uint8_t> raw_payload;
    bool need_crc = false;

    switch (transfer.transfer_type)
    {
    case uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST:
        need_crc = transfer.payload.length() > uavcan::Frame::PAYLOAD_LEN_MAX;
        break;
    case uavcan::TRANSFER_TYPE_SERVICE_RESPONSE:
    case uavcan::TRANSFER_TYPE_SERVICE_REQUEST:
    case uavcan::TRANSFER_TYPE_MESSAGE_UNICAST:
        need_crc = transfer.payload.length() > (uavcan::Frame::PAYLOAD_LEN_MAX - 1);
        raw_payload.push_back(target_node_id);
        break;
    default:
        std::cerr << "X_X" << std::endl;
        std::exit(1);
    }

    if (need_crc)
    {
        // Little endian
        raw_payload.push_back(payload_crc.get() & 0xFF);
        raw_payload.push_back((payload_crc.get() >> 8) & 0xFF);
    }

    raw_payload.insert(raw_payload.end(), transfer.payload.begin(), transfer.payload.end());

    std::vector<uavcan::RxFrame> output;
    unsigned int frame_index = 0;
    unsigned int offset = 0;
    uint64_t ts_monotonic = transfer.ts_monotonic;
    uint64_t ts_utc = transfer.ts_utc;

    while (true)
    {
        int bytes_left = raw_payload.size() - offset;
        EXPECT_TRUE(bytes_left >= 0);
        if (bytes_left <= 0 && !raw_payload.empty())
            break;

        uavcan::RxFrame frm;

        frm.data_type_id = type.id;
        frm.frame_index = frame_index++;
        EXPECT_TRUE(uavcan::Frame::FRAME_INDEX_MAX >= frame_index);

        frm.iface_index = 0;
        frm.last_frame = bytes_left <= uavcan::Frame::PAYLOAD_LEN_MAX;
        frm.payload_len = std::min(int(uavcan::Frame::PAYLOAD_LEN_MAX), bytes_left);
        std::copy(raw_payload.begin() + offset, raw_payload.begin() + offset + frm.payload_len, frm.payload);
        offset += frm.payload_len;

        frm.source_node_id = transfer.source_node_id;
        frm.transfer_id = transfer.transfer_id;
        frm.transfer_type = transfer.transfer_type;

        frm.ts_monotonic = ts_monotonic;
        frm.ts_utc = ts_utc;
        ts_monotonic += 1;
        ts_utc += 1;

        output.push_back(frm);
        if (raw_payload.empty())
            break;
    }
    return output;
}


TEST(TransferListener, TestingEnvironmentMFTSerialization)
{
    uavcan::DataTypeDescriptor type(uavcan::DATA_TYPE_KIND_MESSAGE, 123, uavcan::DataTypeHash());
    for (int i = 0; i < uavcan::DataTypeHash::NUM_BYTES; i++)
        type.hash.value[i] = i;

    static const std::string DATA = "To go wrong in one's own way is better than to go right in someone else's.";
    const Transfer transfer(1, 100000, uavcan::TRANSFER_TYPE_MESSAGE_UNICAST, 12, 42, DATA);

    const std::vector<uavcan::RxFrame> ser = serializeTransfer(transfer, 127, type);

    std::cout << "Serialized transfer:\n";
    for (std::vector<uavcan::RxFrame>::const_iterator it = ser.begin(); it != ser.end(); ++it)
        std::cout << "\t" << it->toString() << "\n";

    for (std::vector<uavcan::RxFrame>::const_iterator it = ser.begin(); it != ser.end(); ++it)
    {
        std::cout << "\t'";
        for (int i = 0; i < it->payload_len; i++)
        {
            uint8_t ch = it->payload[i];
            if (ch < 0x20 || ch > 0x7E)
                ch = '.';
            std::cout << static_cast<char>(ch);
        }
        std::cout << "'\n";
    }
    std::cout << std::flush;
}


TEST(TransferListener, TestingEnvironmentSFTSerialization)
{
    uavcan::DataTypeDescriptor type(uavcan::DATA_TYPE_KIND_MESSAGE, 123, uavcan::DataTypeHash());
    for (int i = 0; i < uavcan::DataTypeHash::NUM_BYTES; i++)
        type.hash.value[i] = i;

    {
        const Transfer transfer(1, 100000, uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST, 12, 42, "Nvrfrget");
        const std::vector<uavcan::RxFrame> ser = serializeTransfer(transfer, 127, type);
        ASSERT_EQ(1, ser.size());
        std::cout << "Serialized transfer:\n\t" << ser[0].toString() << "\n";
    }
    {
        const Transfer transfer(1, 100000, uavcan::TRANSFER_TYPE_SERVICE_REQUEST, 12, 42, "7-chars");
        const std::vector<uavcan::RxFrame> ser = serializeTransfer(transfer, 127, type);
        ASSERT_EQ(1, ser.size());
        std::cout << "Serialized transfer:\n\t" << ser[0].toString() << "\n";
    }
    {
        const Transfer transfer(1, 100000, uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST, 12, 42, "");
        const std::vector<uavcan::RxFrame> ser = serializeTransfer(transfer, 127, type);
        ASSERT_EQ(1, ser.size());
        std::cout << "Serialized transfer:\n\t" << ser[0].toString() << "\n";
    }
    {
        const Transfer transfer(1, 100000, uavcan::TRANSFER_TYPE_SERVICE_RESPONSE, 12, 42, "");
        const std::vector<uavcan::RxFrame> ser = serializeTransfer(transfer, 127, type);
        ASSERT_EQ(1, ser.size());
        std::cout << "Serialized transfer:\n\t" << ser[0].toString() << "\n";
    }
}


template <unsigned int MAX_BUF_SIZE, unsigned int NUM_STATIC_BUFS, unsigned int NUM_STATIC_RECEIVERS>
class TestSubscriber : public uavcan::TransferListener<MAX_BUF_SIZE, NUM_STATIC_BUFS, NUM_STATIC_RECEIVERS>
{
    typedef uavcan::TransferListener<MAX_BUF_SIZE, NUM_STATIC_BUFS, NUM_STATIC_RECEIVERS> Base;

    std::queue<Transfer> transfers_;

public:
    TestSubscriber(const uavcan::DataTypeDescriptor* data_type, uavcan::IAllocator* allocator)
    : Base(data_type, allocator)
    { }

    void handleIncomingTransfer(uavcan::IncomingTransfer& transfer)
    {
        const Transfer rx(transfer);
        transfers_.push(rx);
        std::cout << "Received transfer: " << rx.toString() << std::endl;
    }

    bool matchAndPop(const Transfer& reference)
    {
        if (transfers_.empty())
        {
            std::cout << "No received transfers" << std::endl;
            return false;
        }

        const Transfer tr = transfers_.front();
        transfers_.pop();

        const bool res = (tr == reference);
        if (!res)
        {
            std::cout << "TestSubscriber: Transfer mismatch:\n"
                << "Expected: " << reference.toString() << "\n"
                << "Received: " << tr.toString() << std::endl;
        }
        return res;
    }

    int getNumReceivedTransfers() const { return transfers_.size(); }
    bool isEmpty() const { return transfers_.empty(); }
};


class Emulator
{
    uavcan::TransferListenerBase& target_;
    const uavcan::DataTypeDescriptor type_;
    uint64_t ts_;
    uavcan::TransferID tid_;
    uint8_t target_node_id_;

public:
    Emulator(uavcan::TransferListenerBase& target, const uavcan::DataTypeDescriptor& type, uint8_t target_node_id = 127)
    : target_(target)
    , type_(type)
    , ts_(0)
    , target_node_id_(target_node_id)
    { }

    Transfer makeTransfer(uavcan::TransferType transfer_type, uint8_t source_node_id, const std::string& payload)
    {
        ts_ += 100;
        const Transfer tr(ts_, ts_ + 1000000000ul, transfer_type, tid_, source_node_id, payload);
        tid_.increment();
        return tr;
    }

    void send(const std::vector<std::vector<uavcan::RxFrame> >& sers)
    {
        unsigned int index = 0;
        while (true)
        {
            // Sending all transfers concurrently
            bool all_empty = true;
            for (std::vector<std::vector<uavcan::RxFrame> >::const_iterator it = sers.begin(); it != sers.end(); ++it)
            {
                if (it->size() <= index)
                    continue;
                all_empty = false;
                std::cout << "Emulator: Sending: " << it->at(index).toString() << std::endl;
                target_.handleFrame(it->at(index));
            }
            index++;
            if (all_empty)
                break;
        }
    }

    void send(const Transfer* transfers, unsigned int num_transfers)
    {
        std::vector<std::vector<uavcan::RxFrame> > sers;
        while (num_transfers--)
            sers.push_back(serializeTransfer(*transfers++, target_node_id_, type_));
        send(sers);
    }

    template <int SIZE>
    void send(const Transfer (&transfers)[SIZE])
    {
        send(transfers, SIZE);
    }
};


TEST(TransferListener, BasicMFT)
{
    uavcan::DataTypeDescriptor type(uavcan::DATA_TYPE_KIND_MESSAGE, 123, uavcan::DataTypeHash());
    for (int i = 0; i < uavcan::DataTypeHash::NUM_BYTES; i++)
        type.hash.value[i] = i | (i << 4);

    static const int NUM_POOL_BLOCKS = 12;    // This number is just enough to pass the test
    uavcan::PoolAllocator<uavcan::MEM_POOL_BLOCK_SIZE * NUM_POOL_BLOCKS, uavcan::MEM_POOL_BLOCK_SIZE> pool;
    uavcan::PoolManager<1> poolmgr;
    poolmgr.addPool(&pool);

    TestSubscriber<256, 1, 1> subscriber(&type, &poolmgr);

    /*
     * Test data
     */
    static const std::string DATA[] =
    {
        "123456789",

        "Build a man a fire, and he'll be warm for a day. "
        "Set a man on fire, and he'll be warm for the rest of his life.",

        "The USSR, which they'd begun to renovate and improve at about the time when Tatarsky decided to "
        "change his profession, improved so much that it ceased to exist",

        "In the beginning there was nothing, which exploded.",

        "BEWARE JET BLAST"
    };

    Emulator emulator(subscriber, type);
    const Transfer transfers[] =
    {
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST, 1, DATA[0]),
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   1, DATA[1]),   // Same NID
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   2, DATA[2]),
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_SERVICE_REQUEST,   3, DATA[3]),
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_SERVICE_RESPONSE,  4, DATA[4]),
    };

    /*
     * Sending concurrently
     * Expected reception order: 0, 4, 3, 1, 2
     */
    emulator.send(transfers);

    ASSERT_TRUE(subscriber.matchAndPop(transfers[0]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[4]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[3]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[1]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[2]));

    ASSERT_TRUE(subscriber.isEmpty());
}


TEST(TransferListener, CrcFailure)
{
    uavcan::DataTypeDescriptor type(uavcan::DATA_TYPE_KIND_MESSAGE, 123, uavcan::DataTypeHash());
    for (int i = 0; i < uavcan::DataTypeHash::NUM_BYTES; i++)
        type.hash.value[i] = i | (i << 4);

    uavcan::PoolManager<1> poolmgr;                         // No dynamic memory
    TestSubscriber<256, 2, 2> subscriber(&type, &poolmgr);  // Static buffer only, 2 entries

    /*
     * Generating transfers with damaged payload (CRC is not valid)
     */
    Emulator emulator(subscriber, type);
    const Transfer tr_mft = emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST, 42, "123456789abcdefghik");
    const Transfer tr_sft = emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_UNICAST, 11, "abcd");

    std::vector<uavcan::RxFrame> ser_mft = serializeTransfer(tr_mft, 0, type);
    std::vector<uavcan::RxFrame> ser_sft = serializeTransfer(tr_sft, 9, type);

    ASSERT_TRUE(ser_mft.size() > 1);
    ASSERT_TRUE(ser_sft.size() == 1);

    ser_mft[1].payload[0] = ~ser_mft[1].payload[0];     // CRC is no longer valid
    ser_sft[0].payload[2] = ~ser_sft[0].payload[2];     // SFT has no CRC, so the corruption will be undetected

    /*
     * Sending and making sure that MFT was not received, but SFT was.
     */
    std::vector<std::vector<uavcan::RxFrame> > sers;
    sers.push_back(ser_mft);
    sers.push_back(ser_sft);
    sers.push_back(ser_mft);  // Ignored
    sers.push_back(ser_sft);  // Ignored

    emulator.send(sers);

    Transfer tr_sft_damaged = tr_sft;
    tr_sft_damaged.payload[1] = ~tr_sft.payload[1];     // Damaging the data similarly, so that it can be matched

    ASSERT_TRUE(subscriber.matchAndPop(tr_sft_damaged));
    ASSERT_TRUE(subscriber.isEmpty());
}


TEST(TransferListener, BasicSFT)
{
    uavcan::DataTypeDescriptor type(uavcan::DATA_TYPE_KIND_MESSAGE, 123, uavcan::DataTypeHash());
    for (int i = 0; i < uavcan::DataTypeHash::NUM_BYTES; i++)
        type.hash.value[i] = i | (i << 4);

    uavcan::PoolManager<1> poolmgr;                         // No dynamic memory. At all.
    TestSubscriber<0, 0, 5> subscriber(&type, &poolmgr);    // Max buf size is 0, i.e. SFT-only

    Emulator emulator(subscriber, type);
    const Transfer transfers[] =
    {
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST, 1, "123"),
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   1, "456"),   // Same NID
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   2, ""),
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_SERVICE_REQUEST,   3, "abc"),
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_SERVICE_RESPONSE,  4, ""),
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_SERVICE_RESPONSE,  5, ""),      // New NID, ignored due to OOM
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_SERVICE_RESPONSE,  2, ""),      // New TT, ignored due to OOM
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   2, "foo"),   // Same as 2, not ignored
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   2, "123456789abc"),// Same as 2, not SFT - ignore
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   2, "bar"),   // Same as 2, not ignored
    };

    emulator.send(transfers);

    ASSERT_TRUE(subscriber.matchAndPop(transfers[0]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[1]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[2]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[3]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[4]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[7]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[9]));

    ASSERT_TRUE(subscriber.isEmpty());
}
