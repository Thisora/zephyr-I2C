/*
 * Copyright Runtime.io 2018. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief A driver for sending and receiving mcumgr packets over UART.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/mgmt/mcumgr/transport/serial.h>
#include <zephyr/drivers/console/i2c_mcumgr.h>

static const struct device *const i2c_mcumgr_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_i2c_mcumgr));

/** Callback to execute when a valid fragment has been received. */
static i2c_mcumgr_recv_fn *i2c_mgumgr_recv_cb;

/** Contains the fragment currently being received. */
static struct i2c_mcumgr_rx_buf *i2c_mcumgr_cur_buf;

static struct i2c_target_callbacks i2c_target_callback;

static struct i2c_target_config i2c_slave_conf;

static int i2c_mcumgr_isr(struct i2c_target_config *config, uint8_t val){
	printk("RECIEVED NEW VALUE !! :P  %c \n", val);
}


static void i2c_mcumgr_setup(const struct device *i2c)
{
	i2c_target_callback = {
		.write_received = i2c_mcumgr_isr,
		// .write_requested = i2c_mcumgr_isr,
		.read_requested = i2c_mcumgr_isr,
		.read_processed = ,
		// .stop = i2c_stop,
	};

	i2c_slave_conf= {
		.address = 0xA, //TODO put it in Device Tree (or conf)!
		.callbacks = &i2c_target_callback,
	};

	int error = i2c_target_register(device, &i2c_slave_conf);
	/*TODO error ??*/
	/*TODO empty FIFO ?*/
}


void i2c_mcumgr_register(i2c_mcumgr_recv_fn *cb)
{
	i2c_mgumgr_recv_cb = cb;


	if (device_is_ready(i2c_mcumgr_dev)) {
		i2c_mcumgr_setup(i2c_mcumgr_dev);
	}
}
