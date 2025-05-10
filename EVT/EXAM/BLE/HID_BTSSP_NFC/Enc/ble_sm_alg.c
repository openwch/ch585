/********************************** (C) COPYRIGHT *******************************
 * File Name          : ble_sm_alg.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2025/02/11
 * Description        : ble_sm_alg for CH585/4.
 * Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#include <inttypes.h>
#include <string.h>
#include "cmac_mode.h"
#include "constants.h"
#include "utils.h"
#include "aes.h"
#include "glue.h"
#include "endian.h"
#include "CONFIG.h"

#include "P256_ECDH_RISCV.H"
/* P256_ECDH_RISCV有优化的ECC算法，大约占用8.5k flash，
 * 如果对速度有更高要求，则可以在ld中将段 *(.p256_ecdh); *(.p256_ecdh.*);添加到highcode中 */

#ifndef BLE_HS_LOG
#define BLE_HS_LOG(a,...)
#endif

#ifndef ble_hs_log_flat_buf
#define ble_hs_log_flat_buf(a,b)
#endif

static void
ble_sm_alg_log_buf(const char *name, const uint8_t *buf, int len)
{
    BLE_HS_LOG(DEBUG, "    %s=", name);
    ble_hs_log_flat_buf(buf, len);
    BLE_HS_LOG(DEBUG, "\n");
}

/**
 * Cypher based Message Authentication Code (CMAC) with AES 128 bit
 *
 * @param key                   128-bit key.
 * @param in                    Message to be authenticated.
 * @param len                   Length of the message in octets.
 * @param out                   Output; message authentication code.
 */
static int
ble_sm_alg_aes_cmac(const uint8_t *key, const uint8_t *in, size_t len,
                    uint8_t *out)
{
    struct tc_aes_key_sched_struct sched;
    struct tc_cmac_struct state;

    if (tc_cmac_setup(&state, key, &sched) == TC_CRYPTO_FAIL) {
        return BLE_HS_EUNKNOWN;
    }

    if (tc_cmac_update(&state, in, len) == TC_CRYPTO_FAIL) {
        return BLE_HS_EUNKNOWN;
    }

    if (tc_cmac_final(out, &state) == TC_CRYPTO_FAIL) {
        return BLE_HS_EUNKNOWN;
    }

    return 0;
}

int
ble_sm_alg_f4(uint8_t *u, uint8_t *v, uint8_t *x, uint8_t z,
              uint8_t *out_enc_data)
{
    uint8_t xs[16];
    uint8_t m[65];
    int rc;

    /* *
     * u
     * v
     * x
     * z
     * out_enc_data
     * */
    BLE_HS_LOG(DEBUG, "ble_sm_alg_f4()\n    u=");
    ble_hs_log_flat_buf(u, 32);
    BLE_HS_LOG(DEBUG, "\n    v=");
    ble_hs_log_flat_buf(v, 32);
    BLE_HS_LOG(DEBUG, "\n    x=");
    ble_hs_log_flat_buf(x, 16);
    BLE_HS_LOG(DEBUG, "\n    z=0x%02x\n", z);

    /*
     * U, V and Z are concatenated and used as input m to the function
     * AES-CMAC and X is used as the key k.
     *
     * Core Spec 4.2 Vol 3 Part H 2.2.5
     *
     * note:
     * ble_sm_alg_aes_cmac uses BE data; ble_sm_alg_f4 accepts LE so we swap.
     */
    swap_buf(m, u, 32);
    swap_buf(m + 32, v, 32);
    m[64] = z;

    swap_buf(xs, x, 16);

    rc = ble_sm_alg_aes_cmac(xs, m, sizeof(m), out_enc_data);
    if (rc != 0) {
        return BLE_HS_EUNKNOWN;
    }

    swap_in_place(out_enc_data, 16);

    BLE_HS_LOG(DEBUG, "    out_enc_data=");
    ble_hs_log_flat_buf(out_enc_data, 16);
    BLE_HS_LOG(DEBUG, "\n");

    return 0;
}

int
ble_sm_alg_f5(uint8_t *w, uint8_t *n1, uint8_t *n2, 
							uint8_t a1t, uint8_t *a1, uint8_t a2t, uint8_t *a2, uint8_t *mackey, uint8_t *ltk)
{
    static const uint8_t salt[16] = { 0x6c, 0x88, 0x83, 0x91, 0xaa, 0xf5,
                      0xa5, 0x38, 0x60, 0x37, 0x0b, 0xdb,
                      0x5a, 0x60, 0x83, 0xbe };
    uint8_t m[53] = {
        0x00, /* counter */
        0x62, 0x74, 0x6c, 0x65, /* keyID */
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*n1*/
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*2*/
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* a1 */
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* a2 */
        0x01, 0x00 /* length */
    };
    uint8_t ws[32];
    uint8_t t[16];
    int rc;

    BLE_HS_LOG(DEBUG, "ble_sm_alg_f5()\n");
    ble_sm_alg_log_buf("w", w, 32);
    ble_sm_alg_log_buf("n1", n1, 16);
    ble_sm_alg_log_buf("n2", n2, 16);

    swap_buf(ws, w, 32);

    rc = ble_sm_alg_aes_cmac(salt, ws, 32, t);
    if (rc != 0) {
        return BLE_HS_EUNKNOWN;
    }

    ble_sm_alg_log_buf("t", t, 16);

    swap_buf(m + 5, n1, 16);
    swap_buf(m + 21, n2, 16);
    m[37] = a1t;
    swap_buf(m + 38, a1, 6);
    m[44] = a2t;
    swap_buf(m + 45, a2, 6);

    rc = ble_sm_alg_aes_cmac(t, m, sizeof(m), mackey);
    if (rc != 0) {
        return BLE_HS_EUNKNOWN;
    }

    ble_sm_alg_log_buf("mackey", mackey, 16);

    swap_in_place(mackey, 16);

    /* Counter for ltk is 1. */
    m[0] = 0x01;

    rc = ble_sm_alg_aes_cmac(t, m, sizeof(m), ltk);
    if (rc != 0) {
        return BLE_HS_EUNKNOWN;
    }

    ble_sm_alg_log_buf("ltk", ltk, 16);

    swap_in_place(ltk, 16);

    return 0;
}

int
ble_sm_alg_f6(uint8_t *w, uint8_t *n1, uint8_t *n2,
              uint8_t *r, uint8_t *iocap, uint8_t a1t,
              uint8_t *a1, uint8_t a2t, uint8_t *a2,
              uint8_t *check)
{
    uint8_t ws[16];
    uint8_t m[65];
    int rc;

    BLE_HS_LOG(DEBUG, "ble_sm_alg_f6()\n");
    ble_sm_alg_log_buf("w", w, 16);
    ble_sm_alg_log_buf("n1", n1, 16);
    ble_sm_alg_log_buf("n2", n2, 16);
    ble_sm_alg_log_buf("r", r, 16);
    ble_sm_alg_log_buf("iocap", iocap, 3);
    ble_sm_alg_log_buf("a1t", &a1t, 1);
    ble_sm_alg_log_buf("a1", a1, 6);
    ble_sm_alg_log_buf("a2t", &a2t, 1);
    ble_sm_alg_log_buf("a2", a2, 6);

    swap_buf(m, n1, 16);
    swap_buf(m + 16, n2, 16);
    swap_buf(m + 32, r, 16);
    swap_buf(m + 48, iocap, 3);

    m[51] = a1t;
    memcpy(m + 52, a1, 6);
    swap_buf(m + 52, a1, 6);

    m[58] = a2t;
    memcpy(m + 59, a2, 6);
    swap_buf(m + 59, a2, 6);

    swap_buf(ws, w, 16);

    rc = ble_sm_alg_aes_cmac(ws, m, sizeof(m), check);
    if (rc != 0) {
        return BLE_HS_EUNKNOWN;
    }

    ble_sm_alg_log_buf("res", check, 16);

    swap_in_place(check, 16);

    return 0;
}

/**
 * @brief   used to malloc a size of memory
 *
 * @param   size, flag can not be 0.
 *
 * @return  void * a pointer to the memory.
 */
extern void *tmos_memory_allocate(uint16_t size, uint16_t flag);


/**
 * @brief   used to free a size of memory
 *
 * @param   the pointer of memory.
 *
 * @return  void *.
 */
extern void tmos_memory_free(void *ptr);

/* used by uECC to get random data */
int
ble_sm_alg_rand(uint8_t *dst, unsigned int size)
{
	size_t i;

	/* input sanity check: */
  if (dst == (uint8_t *) 0 || (size <= 0))
    return 0;
  //srand( (unsigned)time( NULL ) );

  for(i = 0 ; i < size ; i++)
  {
	  dst[i] = (uint8_t)(tmos_rand() & 0x000000FF);
  }

  return 1;
}

void
ble_sm_alg_ecc_init(void)
{
    uECC_set_rng(ble_sm_alg_rand);
}

int
ble_sm_alg_gen_dhkey(uint8_t *peer_pub_key_x, uint8_t *peer_pub_key_y,
                     uint8_t *our_priv_key, uint8_t *out_dhkey)
{
    uint8_t dh[32];
    uint8_t pk[64];
    uint8_t priv[32];

    uint8_t res;
    void *mem;

    tmos_memcpy(pk, peer_pub_key_x, 32);
    tmos_memcpy(&pk[32], peer_pub_key_y, 32);

    mem = tmos_memory_allocate(P256_ECDH_BUFFER_SIZE, 0xf);
    if(mem != NULL)
    {
        P256_ecdh_memory_set(mem);
    }
    else
    {
        PRINT("ble_sm_alg_gen_dhkey no memory\n");
        return 1;
    }

    res = P256_ecdh_shared_secret(out_dhkey, pk, our_priv_key);

    if(mem != NULL)
    {
        tmos_memory_free(mem);
        P256_ecdh_memory_reset();
    }

    if(res == 0)
    {
        return 1;
    }
    return 0;
}

/**
 * pub: 64 bytes
 * priv: 32 bytes
 */
int
ble_sm_alg_gen_key_pair(uint8_t *pub, uint8_t *priv)
{
    uint8_t res;
    void *mem;

    ble_sm_alg_rand(priv, 32);

    mem = tmos_memory_allocate(P256_ECDH_BUFFER_SIZE, 0xf);
    if(mem != NULL)
    {
        P256_ecdh_memory_set(mem);
    }
    else
    {
        PRINT("ble_sm_alg_gen_key_pair no memory\n");
        return 1;
    }

    res = P256_ecdh_keygen(pub, priv);

    if(mem != NULL)
    {
        tmos_memory_free(mem);
        P256_ecdh_memory_reset();
    }

    if(res == 0)
    {
        PRINT("P256_ecdh_keygen err\n");
        return 1;
    }
    return 0;
}

int
ble_sm_alg_g2(uint8_t *u, uint8_t *v, uint8_t *x, uint8_t *y,
              uint32_t *passkey)
{
    uint8_t m[80], xs[16];
    int rc;

    BLE_HS_LOG(DEBUG, "ble_sm_alg_g2()\n");
    ble_sm_alg_log_buf("u", u, 32);
    ble_sm_alg_log_buf("v", v, 32);
    ble_sm_alg_log_buf("x", x, 16);
    ble_sm_alg_log_buf("y", y, 16);

    swap_buf(m, u, 32);
    swap_buf(m + 32, v, 32);
    swap_buf(m + 64, y, 16);

    swap_buf(xs, x, 16);

    /* reuse xs (key) as buffer for result */
    rc = ble_sm_alg_aes_cmac(xs, m, sizeof(m), xs);
    if (rc != 0) {
        return BLE_HS_EUNKNOWN;
    }

    ble_sm_alg_log_buf("res", xs, 16);
    *passkey = get_be32(xs + 12) % 1000000;
    BLE_HS_LOG(DEBUG, "    passkey=%u\n", *passkey);

    return 0;
}

