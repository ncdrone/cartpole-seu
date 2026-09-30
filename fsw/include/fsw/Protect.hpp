/** @file Protect.hpp  @brief Software EDAC for the protected build: Hsiao-class SECDED(39,32) per word, CRC-32C over a
 *  block, and the encoded store with its decode-then-CRC order. No heap, no exceptions, bounded loops, F32/U32 only.
 *
 *  Order on every read (SPEC-01 §6): decode + correct every word (write back on SEC) -> CRC-32C over the corrected
 *  plaintext -> range table (Controller side) -> use. SECDED alone miscorrects most 3-bit patterns; the CRC exists to
 *  catch that, and the range table catches what both miss.
 */
#ifndef FSW_PROTECT_HPP
#define FSW_PROTECT_HPP
#include "fsw/FswTypes.hpp"

namespace fsw {

/* ---------------- SECDED(39,32): 32 data bits + 7 check bits ----------------
 * Hsiao construction: each data bit maps to a distinct odd-weight (weight 3) 7-bit column; the 32 columns are the first
 * 32 weight-3 values of 7 bits in ascending order. A single flip yields a syndrome equal to that column (odd weight);
 * two flips XOR two odd columns into an even-weight syndrome, which no column matches -> detected, never "corrected".
 */
enum SecdedStatus { SECDED_OK = 0, SECDED_SEC = 1, SECDED_DED = 2 };

/** @brief 7 check bits for a 32-bit word. */
U8 secded_encode(U32 data);
/** @brief Decode: returns status; on SEC the corrected data (or unchanged data if the check bit itself flipped) is
 *  written to `data` and `check`. On DED nothing is trusted. */
SecdedStatus secded_decode(U32& data, U8& check);

/* ---------------- CRC-32C (Castagnoli, reflected 0x82F63B78) ---------------- */
U32 crc32c(const U8* bytes, U32 n);

/* ---------------- Encoded store ----------------
 * N words of plaintext are held as volatile words + check bytes + a CRC over the plaintext, so an upset in RAM is seen
 * the next time the store is decoded (the compiler cannot keep a volatile store in registers; research/02 finding 1).
 */
static const U32 STORE_MAX_WORDS = 32U;

struct StoreReport {
    U32 sec;       /**< single-bit corrections this decode */
    U32 ded;       /**< uncorrectable words this decode */
    U32 crc_fail;  /**< 1 if the CRC over the corrected plaintext mismatched */
    U32 first_word;/**< index of the first word with an event (for logging) */
};

template <U32 N>
struct EncodedStore {
    volatile U32 words[N];
    volatile U8  check[N];
    volatile U32 crc;      /**< CRC-32C over the N plaintext words */
    volatile U32 crc_inv;  /**< ~crc, so a flip in the CRC word itself is caught without trusting one copy */
};

/** @brief Encode `n` plaintext words into the store (call at init and after a verified reload). */
void store_encode(const U32* plain, U32 n, volatile U32* words, volatile U8* check, volatile U32* crc, volatile U32* crc_inv);

/**
 * @brief Decode the store into `plain`. Corrects single-bit errors in place (scrub) and counts them; any DED or CRC
 *        mismatch returns false and `plain` must not be used.
 */
bool store_decode(volatile U32* words, volatile U8* check, volatile const U32* crc, volatile const U32* crc_inv,
                  U32 n, U32* plain, StoreReport& rep);

} // namespace fsw
#endif
