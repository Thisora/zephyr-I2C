/*
 * Copyright Runtime.io 2018. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief A driver for sending and receiving mcumgr packets over I2C.
 *
 * @see include/mgmt/serial.h
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CONSOLE_I2C_MCUMGR_H_
#define ZEPHYR_INCLUDE_DRIVERS_CONSOLE_I2C_MCUMGR_H_

#include <stdlib.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Contains an mcumgr fragment received over I2C.
 */
struct i2c_mcumgr_rx_buf {
	void *fifo_reserved;   /* 1st word reserved for use by fifo */
	uint8_t data[CONFIG_I2C_MCUMGR_RX_BUF_SIZE];
	int length;
};

/** @typedef i2c_mcumgr_recv_fn
 * @brief Function that gets called when an mcumgr packet is received.
 *
 * Function that gets called when an mcumgr packet is received.  This function
 * gets called in the interrupt context.  Ownership of the specified buffer is
 * transferred to the callback when this function gets called.
 *
 * @param rx_buf                A buffer containing the incoming mcumgr packet.
 */
typedef void i2c_mcumgr_recv_fn(struct i2c_mcumgr_rx_buf *rx_buf);

/**
 * @brief Sends an mcumgr packet over I2C.
 *
 * @param data                  Buffer containing the mcumgr packet to send.
 * @param len                   The length of the buffer, in bytes.
 *
 * @return                      0 on success; negative error code on failure.
 */
int i2c_mcumgr_send(const uint8_t *data, int len);

/**
 * @brief Frees the supplied receive buffer.
 *
 * @param rx_buf                The buffer to free.
 */
void i2c_mcumgr_free_rx_buf(struct i2c_mcumgr_rx_buf *rx_buf);

/**
 * @brief Registers an mcumgr I2C receive handler.
 *
 * Configures the mcumgr I2C driver to call the specified function when an
 * mcumgr request packet is received.
 *
 * @param cb                    The callback to execute when an mcumgr request
 *                                  packet is received.
 */
void i2c_mcumgr_register(i2c_mcumgr_recv_fn *cb);

#ifdef __cplusplus
}
#endif

#endif
