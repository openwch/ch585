#ifndef __TC_TEST_H__
#define __TC_TEST_H__

#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include "constants.h"
#include "utils.h"
#include "ecc_dh.h"


#define BLE_HS_EAGAIN               1
#define BLE_HS_EALREADY             2
#define BLE_HS_EINVAL               3
#define BLE_HS_EMSGSIZE             4
#define BLE_HS_ENOENT               5
#define BLE_HS_ENOMEM               6
#define BLE_HS_ENOTCONN             7
#define BLE_HS_ENOTSUP              8
#define BLE_HS_EAPP                 9
#define BLE_HS_EBADDATA             10
#define BLE_HS_EOS                  11
#define BLE_HS_ECONTROLLER          12
#define BLE_HS_ETIMEOUT             13
#define BLE_HS_EDONE                14
#define BLE_HS_EBUSY                15
#define BLE_HS_EREJECT              16
#define BLE_HS_EUNKNOWN             17
#define BLE_HS_EROLE                18
#define BLE_HS_ETIMEOUT_HCI         19
#define BLE_HS_ENOMEM_EVT           20
#define BLE_HS_ENOADDR              21
#define BLE_HS_ENOTSYNCED           22
#define BLE_HS_EAUTHEN              23
#define BLE_HS_EAUTHOR              24
#define BLE_HS_EENCRYPT             25
#define BLE_HS_EENCRYPT_KEY_SZ      26
#define BLE_HS_ESTORE_CAP           27
#define BLE_HS_ESTORE_FAIL          28
#define BLE_HS_EPREEMPTED           29


void ble_sm_alg_ecc_init(void);
#ifndef BLE_MODE
#include "config.h"
#endif
int
ble_sm_alg_gen_key_pair(uint8_t *pub, uint8_t *priv);

int
ble_sm_alg_gen_dhkey(uint8_t *peer_pub_key_x, uint8_t *peer_pub_key_y,
                     uint8_t *our_priv_key, uint8_t *out_dhkey);
int
ble_sm_alg_f4(uint8_t *u, uint8_t *v, uint8_t *x, uint8_t z,
              uint8_t *out_enc_data);

int
ble_sm_alg_g2(uint8_t *u, uint8_t *v, uint8_t *x, uint8_t *y,
              uint32_t *passkey);

int
ble_sm_alg_f5(uint8_t *w, uint8_t *n1, uint8_t *n2,
              uint8_t a1t, uint8_t *a1, uint8_t a2t, uint8_t *a2, uint8_t *mackey, uint8_t *ltk);

int
ble_sm_alg_f6(uint8_t *w, uint8_t *n1, uint8_t *n2,
              uint8_t *r, uint8_t *iocap, uint8_t a1t,
              uint8_t *a1, uint8_t a2t, uint8_t *a2,
              uint8_t *check);
int
ble_sm_alg_rand(uint8_t *dst, unsigned int size);

#ifdef __cplusplus
}
#endif

#endif /* __TC_TEST_H__ */


