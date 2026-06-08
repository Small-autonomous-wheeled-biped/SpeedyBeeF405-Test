#include "crc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define ASSERT_EQ_U16(actual, expected) \
    do { \
        uint16_t a = (actual); uint16_t e = (expected); \
        if (a != e) { \
            fprintf(stderr, "%s:%d: FAILED %s: expected 0x%04X got 0x%04X\n", \
                    __FILE__, __LINE__, #actual, e, a); \
            exit(1); \
        } \
    } while (0)

static void test_crc16_known_vector(void)
{
    /* CCITT-FALSE (poly=0x1021, init=0xFFFF): "123456789" → 0x29B1 */
    const uint8_t data[] = "123456789";
    ASSERT_EQ_U16(crc16_ccitt(data, 9U), 0x29B1U);
}

static void test_crc16_empty(void)
{
    /* Empty input = init value. */
    ASSERT_EQ_U16(crc16_ccitt(NULL, 0U), 0xFFFFU);
    const uint8_t empty[] = { 0 };
    ASSERT_EQ_U16(crc16_ccitt(empty, 0U), 0xFFFFU);
}

static void test_crc16_single_zero(void)
{
    const uint8_t data[] = { 0x00U };
    /* CRC of 0x00 with init 0xFFFF and poly 0x1021. */
    ASSERT_EQ_U16(crc16_ccitt(data, 1U), 0xE1F0U);
}

static void test_crc16_incremental_matches_single_pass(void)
{
    const uint8_t data[] = { 0x01U, 0x02U, 0x03U, 0x04U, 0xAAU, 0xBBU };
    const uint16_t full = crc16_ccitt(data, sizeof(data));
    /* Split into two chunks. */
    uint16_t inc = 0xFFFFU;
    inc = crc16_ccitt_update(inc, data, 3U);
    inc = crc16_ccitt_update(inc, data + 3U, 3U);
    ASSERT_EQ_U16(inc, full);
}

static void test_crc16_all_zeros(void)
{
    uint8_t zeros[32];
    memset(zeros, 0, sizeof(zeros));
    /* Just check it doesn't crash and returns something consistent. */
    const uint16_t crc1 = crc16_ccitt(zeros, sizeof(zeros));
    const uint16_t crc2 = crc16_ccitt(zeros, sizeof(zeros));
    ASSERT_EQ_U16(crc1, crc2);
}

int main(void)
{
    test_crc16_known_vector();
    test_crc16_empty();
    test_crc16_single_zero();
    test_crc16_incremental_matches_single_pass();
    test_crc16_all_zeros();

    puts("test_crc: all tests passed");
    return 0;
}
