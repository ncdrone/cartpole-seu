/** @file Protect.cpp  @brief SECDED(39,32), CRC-32C, encoded store. See Protect.hpp. */
#include "fsw/Protect.hpp"

namespace fsw {

/* The 32 odd-weight columns of the H matrix: the weight-3 values of 7 bits in ascending order (0x64,0x68,0x70 unused).
 * Generated once offline: for v in 1..127 with popcount(v) == 3, take the first 32. A compile-time table, so no
 * lazy initialisation and no mutable static. test_protect checks the code exhaustively. */
static const U8 HSIAO_COLS[32] = {
    0x07, 0x0B, 0x0D, 0x0E, 0x13, 0x15, 0x16, 0x19, 0x1A, 0x1C, 0x23, 0x25, 0x26, 0x29, 0x2A, 0x2C,
    0x31, 0x32, 0x34, 0x38, 0x43, 0x45, 0x46, 0x49, 0x4A, 0x4C, 0x51, 0x52, 0x54, 0x58, 0x61, 0x62 };
static U8 popcount7(U8 v) { U8 c = 0; for (U32 i = 0; i < 7U; ++i) c = static_cast<U8>(c + ((v >> i) & 1U)); return c; }

U8 secded_encode(U32 data) {
    const U8* col = HSIAO_COLS;
    U8 c = 0;
    for (U32 i = 0; i < 32U; ++i) { if ((data >> i) & 1U) c = static_cast<U8>(c ^ col[i]); }
    return c;
}

SecdedStatus secded_decode(U32& data, U8& check) {
    const U8 syn = static_cast<U8>((secded_encode(data) ^ check) & 0x7FU);
    if (syn == 0U) return SECDED_OK;
    const U8 w = popcount7(syn);
    if ((w & 1U) == 0U) return SECDED_DED;               // even weight: two (or an even number of) flips
    const U8* col = HSIAO_COLS;
    for (U32 i = 0; i < 32U; ++i) {                      // odd weight matching a column: that data bit flipped
        if (col[i] == syn) { data ^= (1U << i); return SECDED_SEC; }
    }
    if (w == 1U) { check = static_cast<U8>(check ^ syn); return SECDED_SEC; }   // a check bit flipped; data intact
    return SECDED_DED;                                    // odd weight, no column: 3+ flips
}

U32 crc32c(const U8* bytes, U32 n) {
    U32 crc = 0xFFFFFFFFU;
    for (U32 i = 0; i < n; ++i) {
        crc ^= bytes[i];
        for (U32 b = 0; b < 8U; ++b) crc = (crc >> 1) ^ (0x82F63B78U & (0U - (crc & 1U)));
    }
    return crc ^ 0xFFFFFFFFU;
}

static U32 crc_words(const U32* plain, U32 n) {
    U8 buf[4U * STORE_MAX_WORDS];
    for (U32 i = 0; i < n; ++i) {                        // little-endian bytes, independent of host endianness
        buf[4U * i + 0U] = static_cast<U8>(plain[i] & 0xFFU);
        buf[4U * i + 1U] = static_cast<U8>((plain[i] >> 8) & 0xFFU);
        buf[4U * i + 2U] = static_cast<U8>((plain[i] >> 16) & 0xFFU);
        buf[4U * i + 3U] = static_cast<U8>((plain[i] >> 24) & 0xFFU);
    }
    return crc32c(buf, 4U * n);
}

void store_encode(const U32* plain, U32 n, volatile U32* words, volatile U8* check, volatile U32* crc, volatile U32* crc_inv) {
    for (U32 i = 0; i < n; ++i) { words[i] = plain[i]; check[i] = secded_encode(plain[i]); }
    const U32 c = crc_words(plain, n);
    *crc = c; *crc_inv = ~c;
}

bool store_decode(volatile U32* words, volatile U8* check, volatile const U32* crc, volatile const U32* crc_inv,
                  U32 n, U32* plain, StoreReport& rep) {
    rep.sec = 0U; rep.ded = 0U; rep.crc_fail = 0U; rep.first_word = 0xFFFFFFFFU;
    for (U32 i = 0; i < n; ++i) {
        U32 d = words[i]; U8 c = check[i];               // explicit volatile reads into locals
        const SecdedStatus s = secded_decode(d, c);
        if (s == SECDED_SEC) { words[i] = d; check[i] = c; rep.sec++; if (rep.first_word == 0xFFFFFFFFU) rep.first_word = i; }   // scrub
        else if (s == SECDED_DED) { rep.ded++; if (rep.first_word == 0xFFFFFFFFU) rep.first_word = i; }
        plain[i] = d;
    }
    const U32 c = crc_words(plain, n);
    const U32 stored = *crc, stored_inv = *crc_inv;
    // The CRC word is checked against both copies: a flip in either copy is a detection, never a silent pass.
    if (c != stored || c != static_cast<U32>(~stored_inv)) rep.crc_fail = 1U;
    return rep.ded == 0U && rep.crc_fail == 0U;
}

} // namespace fsw
