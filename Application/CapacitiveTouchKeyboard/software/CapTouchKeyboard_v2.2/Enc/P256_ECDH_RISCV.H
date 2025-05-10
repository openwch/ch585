/********************************** (C) COPYRIGHT *******************************
 * File Name          : P256_ECDH_RISCV.H
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2025/02/11
 * Description        : P256_ECDH_RISCV header file for CH585/4.
 * Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#ifndef _P256_ECDH_RISCV_H_
#define _P256_ECDH_RISCV_H_

#include "stdint.h"

/*
 * Generic point multiplication
 * Calculates scalar * point.
 * If include_y_in_result == 0, result_point should be an array of size 32 bytes where the resulting x coordinate will be written.
 * If include_y_in_result != 0, result_point should be an array of size 64 bytes where the resulting x coordinate concatenated with y will be written.
 * Returns true on success and false on invalid input (see the functions below what defines invalid input).
 * If false is returned, the result is not written.
 */
uint8_t P256_pointmult(uint8_t* result_point, const uint8_t point[64], const uint8_t scalar[32], uint8_t include_y_in_result);

/*
 * ECDH keygen
 * Multiplies the scalar private_key with the curve-defined base point.
 * The result should be an array of size 64 bytes where the x coordinate concatenated by the y coordinate will be written.
 * Returns true on success and false if the private_key lies outside the allowed range [1..n-1], where n is the curve order.
 * If false is returned, the result is not written. (At that point this function can be called again with a new randomized private_key.)
 */
uint8_t P256_ecdh_keygen(uint8_t result_my_public_point[64], const uint8_t private_key[32]);

/*
 * ECDH shared secret
 * Multiplies the scalar private_key with the other's public point.
 * The result should be an array of size 32 bytes where the x coordinate of the result will be written (y is discarded).
 * Returns true on success and false if any of the following occurs:
 *  - the scalar private_key lies outside the allowed range [1..n-1], where n is the curve order
 *  - a public point coordinate integer lies outside the allowed range [0..p-1], where p is the prime for the field used by the curve
 *  - the public point does not lie on the curve
 *  If false is returned, the result is not written.
 *  NOTE: the boolean return value MUST be checked in order to avoid different attacks.
 */
uint8_t P256_ecdh_shared_secret(uint8_t result_point_x[32], const uint8_t others_public_point[64], const uint8_t private_key[32]) __attribute__((warn_unused_result));

#define P256_ECDH_BUFFER_SIZE       1332

/* 必须给一个四字节对齐的指针，该指针指向的内存大小为P256_ECDH_BUFFER_SIZE个字节。 */
void P256_ecdh_memory_set(uint32_t *mem);

/* 取消P256_ecdh的缓存区 */
void P256_ecdh_memory_reset(void);

#endif /* _P256_ECDH_RISCV_H_ */
