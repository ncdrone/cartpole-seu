/** @file test_protect.cpp  @brief Exhaustive codec tests + store behaviour. Plain asserts, NDEBUG undefined. */
#undef NDEBUG
#include <cassert>
#include <cstdio>
#include <cstring>
#include "fsw/Protect.hpp"

using namespace fsw;

int main() {
    // (a) CRC-32C known answer: "123456789" -> 0xE3069283
    assert(crc32c(reinterpret_cast<const U8*>("123456789"), 9U) == 0xE3069283U);

    // (b) exhaustive SECDED on the real Params words and on adversarial patterns
    const U32 samples[] = { 0x41A00000U, 0xC23D7EB3U, 0x3F800000U, 0x00000000U, 0xFFFFFFFFU, 0x80000000U, 0x00000001U, 0xDEADBEEFU };
    U32 singles = 0, check_singles = 0, doubles = 0, triples_sec_wrong = 0, triples_total = 0;
    for (U32 si = 0; si < 8U; ++si) {
        const U32 w0 = samples[si]; const U8 c0 = secded_encode(w0);
        for (U32 b = 0; b < 39U; ++b) {                       // every single flip (32 data + 7 check) corrects back
            U32 w = w0; U8 c = c0;
            if (b < 32U) w ^= (1U << b); else c = static_cast<U8>(c ^ (1U << (b - 32U)));
            const SecdedStatus s = secded_decode(w, c);
            assert(s == SECDED_SEC && w == w0 && c == c0);
            if (b < 32U) singles++; else check_singles++;
        }
        for (U32 a = 0; a < 39U; ++a) for (U32 b = a + 1; b < 39U; ++b) {   // every double flip -> DED, never SEC
            U32 w = w0; U8 c = c0;
            if (a < 32U) w ^= (1U << a); else c = static_cast<U8>(c ^ (1U << (a - 32U)));
            if (b < 32U) w ^= (1U << b); else c = static_cast<U8>(c ^ (1U << (b - 32U)));
            const SecdedStatus s = secded_decode(w, c);
            assert(s == SECDED_DED);
            doubles++;
        }
        for (U32 a = 0; a < 32U; ++a) for (U32 b = a + 1; b < 32U; ++b) for (U32 d = b + 1; d < 32U; ++d) {   // triples: count miscorrections
            U32 w = w0 ^ (1U << a) ^ (1U << b) ^ (1U << d); U8 c = c0;
            const SecdedStatus s = secded_decode(w, c);
            if (s == SECDED_SEC && w != w0) triples_sec_wrong++;
            triples_total++;
        }
    }
    std::printf("secded: %u single data flips corrected, %u check flips corrected, %u double flips detected, "
                "triples miscorrected %u/%u (%.0f%%): the CRC exists for those\n",
                singles, check_singles, doubles, triples_sec_wrong, triples_total, 100.0 * triples_sec_wrong / triples_total);

    // (c) store: encode -> decode clean; single flip anywhere corrected + scrubbed; double flip -> false; CRC word flip -> false
    U32 plain[6] = { 0x41A00000U, 0xC23D7EB3U, 0x3F800000U, 0x12345678U, 0x00000000U, 0xFFFFFFFFU };
    EncodedStore<6> st; StoreReport rep; U32 out[6];
    store_encode(plain, 6U, st.words, st.check, &st.crc, &st.crc_inv);
    assert(store_decode(st.words, st.check, &st.crc, &st.crc_inv, 6U, out, rep) && rep.sec == 0U && std::memcmp(out, plain, sizeof plain) == 0);
    for (U32 i = 0; i < 6U; ++i) for (U32 b = 0; b < 32U; ++b) {
        st.words[i] = st.words[i] ^ (1U << b);
        assert(store_decode(st.words, st.check, &st.crc, &st.crc_inv, 6U, out, rep) && rep.sec == 1U && rep.first_word == i);
        assert(std::memcmp(out, plain, sizeof plain) == 0 && st.words[i] == plain[i]);            // scrubbed in place
        assert(store_decode(st.words, st.check, &st.crc, &st.crc_inv, 6U, out, rep) && rep.sec == 0U);   // clean again
    }
    st.words[1] = st.words[1] ^ 0xC0000000U;                                                        // bits 30+31 of k2
    assert(!store_decode(st.words, st.check, &st.crc, &st.crc_inv, 6U, out, rep) && rep.ded == 1U);
    store_encode(plain, 6U, st.words, st.check, &st.crc, &st.crc_inv);
    st.crc = st.crc ^ 1U;                                                                           // the CRC word itself
    assert(!store_decode(st.words, st.check, &st.crc, &st.crc_inv, 6U, out, rep) && rep.crc_fail == 1U && rep.ded == 0U);
    store_encode(plain, 6U, st.words, st.check, &st.crc, &st.crc_inv);
    // (d) a triple flip that SECDED miscorrects must still be caught by the CRC
    U32 caught = 0, tried = 0;
    for (U32 a = 0; a < 32U && tried < 200U; a += 3) for (U32 b = a + 5; b < 32U && tried < 200U; b += 4) for (U32 d = b + 7; d < 32U && tried < 200U; d += 5) {
        st.words[1] = plain[1] ^ (1U << a) ^ (1U << b) ^ (1U << d); st.check[1] = secded_encode(plain[1]);
        const bool ok = store_decode(st.words, st.check, &st.crc, &st.crc_inv, 6U, out, rep);
        tried++; if (!ok) caught++;
        store_encode(plain, 6U, st.words, st.check, &st.crc, &st.crc_inv);
    }
    std::printf("store: %u/%u triple-bit patterns rejected by decode+CRC (SECDED alone would have passed the miscorrected ones)\n", caught, tried);
    assert(caught == tried);
    std::printf("ALL PROTECT TESTS PASSED\n");
    return 0;
}
