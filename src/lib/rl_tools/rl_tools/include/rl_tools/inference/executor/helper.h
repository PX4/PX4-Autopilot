#include <stdint.h>
#include <stddef.h>
#include <math.h>

static size_t rl_tools_inference_executor_u32_to_revstr(uint32_t v, char *buf, size_t cap){
    size_t idx = 0;
    if (v == 0 && idx < cap) {
        buf[idx++] = '0';
        return idx;
    }
    while (v && idx < cap) {
        buf[idx++] = (char)('0' + (v % 10));
        v /= 10;
    }
    return idx;
}

int rl_tools_inference_executor_int_to_str(int32_t v, char *dst, size_t dst_size){
    if (!dst || dst_size == 0)
        return 0;

    char     tmp[32];
    size_t   tmp_len = 0;

    int       neg = v < 0;
    uint32_t  u   = neg ? (uint32_t)(-(int64_t)v) : (uint32_t)v;

    tmp_len = rl_tools_inference_executor_u32_to_revstr(u, tmp, sizeof(tmp));

    size_t needed = tmp_len + (neg ? 1 : 0);
    size_t out    = 0;

    if (neg && out < dst_size - 1)
        dst[out++] = '-';

    for (size_t i = 0; i < tmp_len && out < dst_size - 1; ++i)
        dst[out++] = tmp[tmp_len - 1 - i];

    dst[out] = '\0';
    return (int)needed;
}


int rl_tools_inference_executor_float_to_str(float f, char *dst, size_t dst_size){
    static const double POW10_6 = 1e6;
    if (!dst || dst_size == 0)
        return 0;

    /* --- special IEEE‑754 values --- */
    if (std::isnan(f)) {
        const char *s = "nan";
        size_t i = 0;
        for (; s[i] && i < dst_size - 1; ++i) dst[i] = s[i];
        dst[i] = '\0';
        return 3;
    }
    if (std::isinf(f)) {
        const char *s = signbit(f) ? "-inf" : "inf";
        size_t len = signbit(f) ? 4 : 3, i = 0;
        for (; i < len && i < dst_size - 1; ++i) dst[i] = s[i];
        dst[i] = '\0';
        return (int)len;
    }

    int neg = (f < 0.0f);
    double d = neg ? -(double)f : (double)f;

    d += (double)0.5e-6;

    double int_d;
    double frac = modf(d, &int_d);

    char   tmp[64];
    size_t tmp_len = 0;

    if (int_d == (double)0.0) {
        tmp[tmp_len++] = '0';
    } else {
        while (int_d >= (double)1.0 && tmp_len < sizeof(tmp)) {
            int digit = (int)fmod(int_d, 10.0);
            tmp[tmp_len++] = (char)('0' + digit);
            int_d = floor(int_d / (double)10.0);
        }
    }

    size_t needed = tmp_len + (neg ? 1 : 0) + 1 /* '.' */ + 6;
    size_t out    = 0;

    if (neg && out < dst_size - 1)
        dst[out++] = '-';

    for (size_t i = 0; i < tmp_len && out < dst_size - 1; ++i)
        dst[out++] = tmp[tmp_len - 1 - i];

    if (out < dst_size - 1)
        dst[out++] = '.';

    unsigned int frac_i = (unsigned int)(frac * POW10_6);
    char frac_buf[6];
    for (int i = 5; i >= 0; --i) {
        frac_buf[i] = (char)('0' + (frac_i % 10));
        frac_i /= 10;
    }
    for (int i = 0; i < 6 && out < dst_size - 1; ++i)
        dst[out++] = frac_buf[i];

    dst[out] = '\0';
    return (int)needed;
}
