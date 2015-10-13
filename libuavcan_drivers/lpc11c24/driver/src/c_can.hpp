/*
 * Bosch C_CAN controller API.
 *
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace uavcan_lpc11c24
{
namespace c_can
{

struct MsgIfaceType
{
    std::uint32_t CMDREQ;

    union
    {
        std::uint32_t W;
        std::uint32_t R;
    } CMDMSK;

    std::uint32_t MSK1;
    std::uint32_t MSK2;

    std::uint32_t ARB1;
    std::uint32_t ARB2;

    std::uint32_t MCTRL;

    std::uint32_t DA1;
    std::uint32_t DA2;
    std::uint32_t DB1;
    std::uint32_t DB2;

    const std::uint32_t _skip[13];
};

static_assert(offsetof(MsgIfaceType, CMDMSK) == 0x04, "C_CAN offset");
static_assert(offsetof(MsgIfaceType, MSK1)   == 0x08, "C_CAN offset");
static_assert(offsetof(MsgIfaceType, ARB1)   == 0x10, "C_CAN offset");
static_assert(offsetof(MsgIfaceType, MCTRL)  == 0x18, "C_CAN offset");
static_assert(offsetof(MsgIfaceType, DA1)    == 0x1c, "C_CAN offset");
static_assert(offsetof(MsgIfaceType, DB2)    == 0x28, "C_CAN offset");

static_assert(sizeof(MsgIfaceType) == 96, "C_CAN size");


struct Type
{
    std::uint32_t CNTL;
    std::uint32_t STAT;
    const std::uint32_t EC;
    std::uint32_t BT;
    const std::uint32_t INT;
    std::uint32_t TEST;
    std::uint32_t BRPE;

    const std::uint32_t _skip_a[1];

    MsgIfaceType IF[2];                 // [0] @ 0x020, [1] @ 0x080

    const std::uint32_t _skip_b[8];

    const std::uint32_t TXREQ[2];       // 0x100

    const std::uint32_t _skip_c[6];

    const std::uint32_t ND[2];          // 0x120

    const std::uint32_t _skip_d[6];

    const std::uint32_t IR[2];          // 0x140

    const std::uint32_t _skip_e[6];

    const std::uint32_t MSGV[2];        // 0x160

    const std::uint32_t _skip_f[6];

    std::uint32_t CLKDIV;               // 0x180
};

static_assert(offsetof(Type, CNTL)   == 0x000, "C_CAN offset");
static_assert(offsetof(Type, STAT)   == 0x004, "C_CAN offset");
static_assert(offsetof(Type, TEST)   == 0x014, "C_CAN offset");
static_assert(offsetof(Type, BRPE)   == 0x018, "C_CAN offset");
static_assert(offsetof(Type, IF[0])  == 0x020, "C_CAN offset");
static_assert(offsetof(Type, IF[1])  == 0x080, "C_CAN offset");
static_assert(offsetof(Type, TXREQ)  == 0x100, "C_CAN offset");
static_assert(offsetof(Type, ND)     == 0x120, "C_CAN offset");
static_assert(offsetof(Type, IR)     == 0x140, "C_CAN offset");
static_assert(offsetof(Type, MSGV)   == 0x160, "C_CAN offset");
static_assert(offsetof(Type, CLKDIV) == 0x180, "C_CAN offset");

static_assert(offsetof(Type, IF[0].DB2) == 0x048, "C_CAN offset");
static_assert(offsetof(Type, IF[1].DB2) == 0x0A8, "C_CAN offset");


volatile Type& CAN = *reinterpret_cast<volatile Type*>(0x40050000);


/*
 * CNTL
 */
static constexpr std::uint32_t CNTL_TEST = 1 << 7;
static constexpr std::uint32_t CNTL_CCE  = 1 << 6;
static constexpr std::uint32_t CNTL_DAR  = 1 << 5;
static constexpr std::uint32_t CNTL_EIE  = 1 << 3;
static constexpr std::uint32_t CNTL_SIE  = 1 << 2;
static constexpr std::uint32_t CNTL_IE   = 1 << 1;
static constexpr std::uint32_t CNTL_INIT = 1 << 0;

static constexpr std::uint32_t CNTL_IRQ_MASK = CNTL_EIE | CNTL_IE | CNTL_SIE;

/*
 * TEST
 */
static constexpr std::uint32_t TEST_RX       = 1 << 7;
static constexpr std::uint32_t TEST_LBACK    = 1 << 4;
static constexpr std::uint32_t TEST_SILENT   = 1 << 3;
static constexpr std::uint32_t TEST_BASIC    = 1 << 2;
static constexpr std::uint32_t TEST_TX_SHIFT = 5;

enum class TestTx : std::uint32_t
{
    Controller      = 0,
    SamplePoint     = 1,
    LowDominant     = 2,
    HighRecessive   = 3
};

/*
 * STAT
 */
static constexpr std::uint32_t STAT_BOFF      = 1 << 7;
static constexpr std::uint32_t STAT_EWARN     = 1 << 6;
static constexpr std::uint32_t STAT_EPASS     = 1 << 5;
static constexpr std::uint32_t STAT_RXOK      = 1 << 4;
static constexpr std::uint32_t STAT_TXOK      = 1 << 3;
static constexpr std::uint32_t STAT_LEC_MASK  = 7;
static constexpr std::uint32_t STAT_LEC_SHIFT = 0;

enum class StatLec : std::uint32_t
{
    NoError     = 0,
    StuffError  = 1,
    FormError   = 2,
    AckError    = 3,
    Bit1Error   = 4,
    Bit0Error   = 5,
    CRCError    = 6,
    Unused      = 7
};

/*
 * IF.CMDREQ
 */
static constexpr std::uint32_t IF_CMDREQ_BUSY = 1 << 15;

/*
 * IF.CMDMSK
 */
static constexpr std::uint32_t IF_CMDMSK_W_DATA_A = 1 << 0;
static constexpr std::uint32_t IF_CMDMSK_W_DATA_B = 1 << 1;
static constexpr std::uint32_t IF_CMDMSK_W_TXRQST = 1 << 2;
static constexpr std::uint32_t IF_CMDMSK_W_CTRL   = 1 << 4;
static constexpr std::uint32_t IF_CMDMSK_W_ARB    = 1 << 5;
static constexpr std::uint32_t IF_CMDMSK_W_MASK   = 1 << 6;
static constexpr std::uint32_t IF_CMDMSK_W_WR_RD  = 1 << 7;

/*
 * IF.MCTRL
 */
static constexpr std::uint32_t IF_MCTRL_NEWDAT   = 1 << 15;
static constexpr std::uint32_t IF_MCTRL_MSGLST   = 1 << 14;
static constexpr std::uint32_t IF_MCTRL_INTPND   = 1 << 13;
static constexpr std::uint32_t IF_MCTRL_UMASK    = 1 << 12;
static constexpr std::uint32_t IF_MCTRL_TXIE     = 1 << 11;
static constexpr std::uint32_t IF_MCTRL_RXIE     = 1 << 10;
static constexpr std::uint32_t IF_MCTRL_RMTEN    = 1 << 9;
static constexpr std::uint32_t IF_MCTRL_TXRQST   = 1 << 8;
static constexpr std::uint32_t IF_MCTRL_EOB      = 1 << 7;
static constexpr std::uint32_t IF_MCTRL_DLC_MASK = 15;

}
}
