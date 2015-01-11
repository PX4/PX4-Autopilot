/*
 * Fast bit array copy algorithm.
 * Source: http://stackoverflow.com/questions/3534535/whats-a-time-efficient-algorithm-to-copy-unaligned-bit-arrays
 * Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/marshal/bit_stream.hpp>
#include <climits>
#include <cstring>
#include <cstddef>
namespace uavcan
{

static const unsigned char reverse_mask[]     = { 0x55U, 0x80U, 0xC0U, 0xE0U, 0xF0U, 0xF8U, 0xFCU, 0xFEU, 0xFFU };
static const unsigned char reverse_mask_xor[] = { 0xFFU, 0x7FU, 0x3FU, 0x1FU, 0x0FU, 0x07U, 0x03U, 0x01U, 0x00U };

#if UAVCAN_TINY

#define PREPARE_FIRST_COPY()                                       \
    do {                                                           \
    if (src_len >= (CHAR_BIT - dst_offset_modulo)) {               \
        *dst &= reverse_mask[dst_offset_modulo];                   \
        src_len -= CHAR_BIT - dst_offset_modulo;                   \
    } else {                                                       \
        *dst &= reverse_mask[dst_offset_modulo] |                  \
                reverse_mask_xor[dst_offset_modulo + src_len + 1]; \
        c &= reverse_mask[dst_offset_modulo + src_len];            \
        src_len = 0;                                               \
    } } while (0)


void bitarrayCopy(const unsigned char* src_org, unsigned src_offset, unsigned src_len,
                  unsigned char* dst_org, unsigned dst_offset)
{
    if (src_len > 0U)
    {
        const unsigned char *src = src_org + (src_offset / CHAR_BIT);
        unsigned char *dst = dst_org + (dst_offset / CHAR_BIT);

        const unsigned src_offset_modulo = src_offset % CHAR_BIT;
        const unsigned dst_offset_modulo = dst_offset % CHAR_BIT;

        if (src_offset_modulo == dst_offset_modulo)
        {
            if (src_offset_modulo > 0U)
            {
                unsigned char c = reverse_mask_xor[dst_offset_modulo] & *src++;
                PREPARE_FIRST_COPY();
                *dst++ |= c;
            }

            const unsigned byte_len = src_len / CHAR_BIT;
            const unsigned src_len_modulo = src_len % CHAR_BIT;

            if (byte_len > 0U)
            {
                (void)std::memcpy(dst, src, byte_len);
                src += byte_len;
                dst += byte_len;
            }
            if (src_len_modulo > 0U)
            {
                *dst &= reverse_mask_xor[src_len_modulo];
                *dst |= reverse_mask[src_len_modulo] & *src;
            }
        }
        else
        {
            unsigned bit_diff_ls = 0U;
            unsigned bit_diff_rs = 0U;
            unsigned char c = 0U;
            /*
             * Begin: Line things up on destination.
             */
            if (src_offset_modulo > dst_offset_modulo)
            {
                bit_diff_ls = src_offset_modulo - dst_offset_modulo;
                bit_diff_rs = CHAR_BIT - bit_diff_ls;

                c = static_cast<unsigned char>(*src++ << bit_diff_ls);
                c = static_cast<unsigned char>(c | (*src >> bit_diff_rs));
                c &= reverse_mask_xor[dst_offset_modulo];
            }
            else
            {
                bit_diff_rs = dst_offset_modulo - src_offset_modulo;
                bit_diff_ls = CHAR_BIT - bit_diff_rs;

                c = static_cast<unsigned char>(*src >> bit_diff_rs & reverse_mask_xor[dst_offset_modulo]);
            }
            PREPARE_FIRST_COPY();
            *dst++ |= c;

            /*
             * Middle: copy with only shifting the source.
             */
            int byte_len = int(src_len / CHAR_BIT);

            while (--byte_len >= 0)
            {
                c = static_cast<unsigned char>(*src++ << bit_diff_ls);
                c = static_cast<unsigned char>(c | (*src >> bit_diff_rs));
                *dst++ = c;
            }

            /*
             * End: copy the remaing bits;
             */
            unsigned src_len_modulo = src_len % CHAR_BIT;
            if (src_len_modulo > 0U)
            {
                c = static_cast<unsigned char>(*src++ << bit_diff_ls);
                c = static_cast<unsigned char>(c | (*src >> bit_diff_rs));
                c &= reverse_mask[src_len_modulo];

                *dst &= reverse_mask_xor[src_len_modulo];
                *dst |= c;
            }
        }
    }
}

#else

/*
 * Functions below were manually optimized in the most horrible way.
 */

void bitarrayCopyAlignedToUnaligned(const unsigned char* src_org, unsigned src_len,
                                    unsigned char* dst_org, unsigned dst_offset)
{
    if (src_len > 0U)
    {
        unsigned char* dst = dst_org + (dst_offset / CHAR_BIT);
        const unsigned dst_offset_modulo = dst_offset % CHAR_BIT;

        if (0U == dst_offset_modulo)
        {
            const unsigned byte_len = src_len / CHAR_BIT;
            const unsigned src_len_modulo = src_len % CHAR_BIT;

            if (byte_len > 0U)
            {
                (void)std::memcpy(dst, src_org, byte_len);
                src_org += byte_len;
                dst += byte_len;
            }
            if (src_len_modulo > 0U)
            {
                *dst &= reverse_mask_xor[src_len_modulo];
                *dst |= reverse_mask[src_len_modulo] & *src_org;
            }
        }
        else
        {
            const unsigned bit_diff_ls = CHAR_BIT - dst_offset_modulo;
            unsigned char c =
                static_cast<unsigned char>(*src_org >> dst_offset_modulo & reverse_mask_xor[dst_offset_modulo]);

            if (src_len >= (CHAR_BIT - dst_offset_modulo))
            {
                *dst &= reverse_mask[dst_offset_modulo];
                src_len -= CHAR_BIT - dst_offset_modulo;
            }
            else
            {
                *dst &= reverse_mask[dst_offset_modulo] | reverse_mask_xor[dst_offset_modulo + src_len + 1];
                c &= reverse_mask[dst_offset_modulo + src_len];
                src_len = 0;
            }

            *dst++ |= c;

            int byte_len = int(src_len / CHAR_BIT);

            while (--byte_len >= 0)
            {
                c = static_cast<unsigned char>(*src_org++ << bit_diff_ls);
                c = static_cast<unsigned char>(c | (*src_org >> dst_offset_modulo));
                *dst++ = c;
            }

            const unsigned src_len_modulo = src_len % CHAR_BIT;
            if (src_len_modulo > 0U)
            {
                c = static_cast<unsigned char>(*src_org++ << bit_diff_ls);
                c = static_cast<unsigned char>(c | (*src_org >> dst_offset_modulo));
                c &= reverse_mask[src_len_modulo];

                *dst &= reverse_mask_xor[src_len_modulo];
                *dst |= c;
            }
        }
    }
}

void bitarrayCopyUnalignedToAligned(const unsigned char* src_org, unsigned src_offset, unsigned src_len,
                                    unsigned char* dst_org)
{
    if (src_len > 0U)
    {
        const unsigned char* src = src_org + (src_offset / CHAR_BIT);

        const unsigned src_offset_modulo = src_offset % CHAR_BIT;

        if (src_offset_modulo == 0U)
        {
            const unsigned byte_len = src_len / CHAR_BIT;
            const unsigned src_len_modulo = src_len % CHAR_BIT;

            if (byte_len > 0U)
            {
                (void)std::memcpy(dst_org, src, byte_len);
                src += byte_len;
                dst_org += byte_len;
            }
            if (src_len_modulo > 0U)
            {
                *dst_org &= reverse_mask_xor[src_len_modulo];
                *dst_org |= reverse_mask[src_len_modulo] & *src;
            }
        }
        else
        {
            const unsigned bit_diff_rs = CHAR_BIT - src_offset_modulo;

            unsigned char c = static_cast<unsigned char>(*src++ << src_offset_modulo);
            c = static_cast<unsigned char>(c | (*src >> bit_diff_rs));

            if (src_len >= CHAR_BIT)
            {
                *dst_org &= 0x55U;
                src_len -= CHAR_BIT;
            }
            else
            {
                *dst_org &= 0x55U | reverse_mask_xor[src_len + 1];
                c &= reverse_mask[src_len];
                src_len = 0;
            }

            *dst_org++ |= c;

            int byte_len = int(src_len / CHAR_BIT);

            while (--byte_len >= 0)
            {
                c = static_cast<unsigned char>(*src++ << src_offset_modulo);
                c = static_cast<unsigned char>(c | (*src >> bit_diff_rs));
                *dst_org++ = c;
            }

            const unsigned src_len_modulo = src_len % CHAR_BIT;
            if (src_len_modulo > 0U)
            {
                c = static_cast<unsigned char>(*src++ << src_offset_modulo);
                c = static_cast<unsigned char>(c | (*src >> bit_diff_rs));
                c &= reverse_mask[src_len_modulo];

                *dst_org &= reverse_mask_xor[src_len_modulo];
                *dst_org |= c;
            }
        }
    }
}

#endif

}
