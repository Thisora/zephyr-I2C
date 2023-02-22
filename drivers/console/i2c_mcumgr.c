/*
 * Copyright Runtime.io 2018. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief A driver for sending and receiving mcumgr packets over I2C.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/mgmt/mcumgr/transport/serial.h>
#include <zephyr/drivers/console/i2c_mcumgr.h>
#include <zephyr/irq.h>
#include <zephyr/sys/ring_buffer.h>

static const struct device *const i2c_mcumgr_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_i2c_mcumgr));


/** Callback to execute when a valid fragment has been received. */
static i2c_mcumgr_recv_fn *i2c_mgumgr_recv_cb;


static struct i2c_mcumgr_rx_buf *i2c_mcumgr_cur_buf;

static struct i2c_target_callbacks i2c_target_callback;

static struct i2c_target_config i2c_slave_conf;

/* Fragment incoming should be ignoring (too long). */
static bool i2c_mcumgr_ignoring;


/** Contains buffers to hold incoming request fragments. */
K_MEM_SLAB_DEFINE(i2c_mcumgr_slab_rx, sizeof(struct i2c_mcumgr_rx_buf),
		  CONFIG_I2C_MCUMGR_RX_BUF_COUNT, 1);


RING_BUF_DECLARE(i2c_mcumgr_ring_tx, CONFIG_MCUMGR_SMP_I2C_MTU * CONFIG_I2C_MCUMGR_TX_BUF_COUNT);

static struct i2c_mcumgr_rx_buf *i2c_mcumgr_alloc_rx_buf(void)
{
	struct i2c_mcumgr_rx_buf *rx_buf;
	void *block;
	int rc;
	rc = k_mem_slab_alloc(&i2c_mcumgr_slab_rx, &block, K_NO_WAIT);
	if (rc != 0) {
		return NULL;
	}

	rx_buf = block;
	rx_buf->length = 0;
	return rx_buf;
}



void i2c_mcumgr_free_rx_buf(struct i2c_mcumgr_rx_buf *rx_buf)
{
	unsigned int key = irq_lock();
	void *block;
	block = rx_buf;
	k_mem_slab_free(&i2c_mcumgr_slab_rx, &block);
	irq_unlock(key);
}

static int i2c_mcumgr_wreq_isr(struct i2c_target_config *config){
	return 0;
}

static int i2c_mcumgr_wrec_isr(struct i2c_target_config *config, uint8_t val){
	unsigned int key = irq_lock();
	struct i2c_mcumgr_rx_buf *rx_buf;

	/* init buffer to save incoming fragment */
	if(!i2c_mcumgr_ignoring){
		if(i2c_mcumgr_cur_buf == NULL){
			i2c_mcumgr_cur_buf = i2c_mcumgr_alloc_rx_buf();
			if(i2c_mcumgr_cur_buf == NULL){
				/* Not enough buffers to contain all fragments -> drop */
				i2c_mcumgr_ignoring = true;
			}
		}
	}

	rx_buf = i2c_mcumgr_cur_buf;
	if(!i2c_mcumgr_ignoring){
		if(rx_buf->length >= sizeof(rx_buf->data)){
			/* The line is bigger then max expected 
			   (CONFIG_I2C_MCUMGR_RX_BUF_SIZE)   */
			i2c_mcumgr_free_rx_buf(i2c_mcumgr_cur_buf);
			i2c_mcumgr_cur_buf = NULL;
			i2c_mcumgr_ignoring = true;
		}else{
			/* All good ! Storing data to the buffer */
			if(val != CONFIG_I2C_MCUMGR_TARGET_READ_REGISTER_ADDRESS){
				rx_buf->data[rx_buf->length++] = val;
			}
		}
	}
	/* Check if fragment is complete */
	if (val == '\n') {
		if (i2c_mcumgr_ignoring) {

			/* Stop ignoring */
			i2c_mcumgr_ignoring = false;
		} else {
			i2c_mcumgr_cur_buf = NULL;
			i2c_mgumgr_recv_cb(rx_buf);
		}
	}

	irq_unlock(key);
	return 0;
}

static int i2c_mcumgr_rreq_isr(struct i2c_target_config *config, uint8_t * val){
	unsigned int key = irq_lock();

	if (ring_buf_is_empty(&i2c_mcumgr_ring_tx) == 0) {
		int ret = ring_buf_get(&i2c_mcumgr_ring_tx, val, 1);
		if(ret != 1){
			irq_unlock(key);
			return -1;
		}
	} else {
		/* Signify to master that no more datas are available*/
		*val = 0xff;
		irq_unlock(key);
		return -1;
	}
	irq_unlock(key);
	return 0;
}

static int i2c_mcumgr_rpro_isr(struct i2c_target_config *config, uint8_t* val){

	return 0;
}

/**
 * @brief As we have to wait controller request to send raw
 * we store it to be ready
 */
static int i2c_mcumgr_store_raw(const void *data, int len){
	/* 	We need to make sure that master do not ask for
		a response until the response buffer is ready */
	unsigned int key = irq_lock();
	int ret = ring_buf_put(&i2c_mcumgr_ring_tx, data, len);
	if(ret != len){
		printk("Not enough data in Ring Buffer ");
	}

	irq_unlock(key);
	return 0;
}

int i2c_mcumgr_send(const uint8_t *data, int len){
	int ret = mcumgr_serial_tx_pkt(data, len, i2c_mcumgr_store_raw);
	return ret;
};

int i2c_mcumgr_stop(struct i2c_target_config *config){
	return 0;
}
static void i2c_mcumgr_setup(const struct device *i2c)
{
	i2c_target_callback.write_received = i2c_mcumgr_wrec_isr;
	i2c_target_callback.write_requested = i2c_mcumgr_wreq_isr;
	i2c_target_callback.read_requested = i2c_mcumgr_rreq_isr;
	i2c_target_callback.read_processed = i2c_mcumgr_rpro_isr;
	i2c_target_callback.stop = i2c_mcumgr_stop;

	i2c_slave_conf.address = CONFIG_I2C_MCUMGR_TARGET_ADDRESS;
	i2c_slave_conf.callbacks = &i2c_target_callback;
	int error = i2c_target_register(i2c, &i2c_slave_conf);
	if (error) {
		assert_print("Can't register device I2C: %d", error);
	}
}

void i2c_mcumgr_register(i2c_mcumgr_recv_fn *cb)
{
	i2c_mgumgr_recv_cb = cb;


	if (device_is_ready(i2c_mcumgr_dev)) {
		i2c_mcumgr_setup(i2c_mcumgr_dev);
	}
}
