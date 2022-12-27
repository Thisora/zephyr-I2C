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

static const struct device *const i2c_mcumgr_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_i2c_mcumgr));


/** Callback to execute when a valid fragment has been received. */
static i2c_mcumgr_recv_fn *i2c_mgumgr_recv_cb;


static struct i2c_mcumgr_rx_buf *i2c_mcumgr_cur_buf;

static struct i2c_target_callbacks i2c_target_callback;

static struct i2c_target_config i2c_slave_conf;

/* Fragment incoming should be ignoring (too long). */
static bool i2c_mcumgr_ignoring;
/** Contain the response to send to the controller **/

static struct i2c_mcumgr_tx_buf* i2c_mcumgr_next_buf; 

/** Contains buffers to hold incoming request fragments. */
K_MEM_SLAB_DEFINE(i2c_mcumgr_slab_rx, sizeof(struct i2c_mcumgr_rx_buf),
		  CONFIG_I2C_MCUMGR_RX_BUF_COUNT, 1);

/** Contains buffers that hold response until controller ask for it */
K_MEM_SLAB_DEFINE(i2c_mcumgr_slab_tx, sizeof(struct i2c_mcumgr_tx_buf),
		  CONFIG_I2C_MCUMGR_TX_BUF_COUNT, 1);

static struct i2c_mcumgr_rx_buf *i2c_mcumgr_alloc_rx_buf(void)
{
	struct i2c_mcumgr_rx_buf *rx_buf;
	void *block;
	int rc;
	rc = k_mem_slab_alloc(&i2c_mcumgr_slab_rx, &block, K_NO_WAIT);
	if (rc != 0) {
		printk("Oh seams to have memory issue\n");
		return NULL;
	}

	rx_buf = block;
	rx_buf->length = 0;
	return rx_buf;
}

static struct i2c_mcumgr_tx_buf* i2c_mcumgr_alloc_tx(void) {

	printk("Allocating a new TX buf\n");
	struct i2c_mcumgr_tx_buf* tx_buf;
	void *block;
	int rc;
	rc = k_mem_slab_alloc(&i2c_mcumgr_slab_tx, &block, K_NO_WAIT);
	if(rc != 0){
		return NULL;
	}
	tx_buf = block;
	tx_buf->length = 0;
	tx_buf->send = 0;
	return tx_buf;
}

void i2c_mcumgr_free_rx_buf(struct i2c_mcumgr_rx_buf *rx_buf)
{
	void *block;
	block = rx_buf;
	k_mem_slab_free(&i2c_mcumgr_slab_rx, &block);
}

void i2c_mcumgr_free_tx_buf(struct i2c_mcumgr_tx_buf *tx_buf)
{
	void *block;
	block = tx_buf;
	k_mem_slab_free(&i2c_mcumgr_slab_tx, &block);
	printk("!!Mem fred!!\n");
}

static int i2c_mcumgr_wreq_isr(struct i2c_target_config *config){
	return 0;
}

static int i2c_mcumgr_wrec_isr(struct i2c_target_config *config, uint8_t val){
	struct i2c_mcumgr_rx_buf *rx_buf;

	/* init buffer to save incoming fragment */
	if(!i2c_mcumgr_ignoring){
		if(i2c_mcumgr_cur_buf == NULL){
			i2c_mcumgr_cur_buf = i2c_mcumgr_alloc_rx_buf();
			printk("rx_buf->length : %d\n", i2c_mcumgr_cur_buf->length);
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
			/* All good storing data to the buffer */
			if(val != 0x05){
				rx_buf->data[rx_buf->length++] = val;
			}/*else{
				printk("Ok it was a read request\n");
			}*/
		}
	}
	/* Check if fragment is complete */
	if (val == '\n') {
		if (i2c_mcumgr_ignoring) {
			/**
			 * TODO : if fragment is being ignored and stop transmitting
			 * before '\n', the bool  i2c_mcumgr_ignoring will never be set
			 * to false. The following fragment will be totally ignored 
			 */

			/* Stop ignoring */
			i2c_mcumgr_ignoring = false;
		} else {
			i2c_mcumgr_cur_buf = NULL;
			i2c_mgumgr_recv_cb(rx_buf);
			printk(" Rx new frag: len: %d\n", rx_buf->length);
		}
	}
	return 0;
}

static int i2c_mcumgr_rreq_isr(struct i2c_target_config *config, uint8_t * val){
	unsigned int key = irq_lock();
	
	if(i2c_mcumgr_next_buf == NULL){
		printk("[I2C mcumgr] RREQ error\n");
		return -1;
	}

	if (i2c_mcumgr_next_buf->send < i2c_mcumgr_next_buf->length) {
		*val = i2c_mcumgr_next_buf->data[i2c_mcumgr_next_buf->send++];
		printk("[I2C mcumgr] responding: %u (%d/%d bytes send)\n", *val, i2c_mcumgr_next_buf->send, i2c_mcumgr_next_buf->length);
	}else {
		printk("[I2C mcumgr] Resp totally send \n");
		/* Response is totally send */
		i2c_mcumgr_free_tx_buf(i2c_mcumgr_next_buf);
		i2c_mcumgr_next_buf = NULL;
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
	const uint8_t *it = data; /* iterator to copy data */
	while (len--) {
		if(i2c_mcumgr_next_buf->length < sizeof(i2c_mcumgr_next_buf->data)){
			i2c_mcumgr_next_buf->data[i2c_mcumgr_next_buf->length++] = *it++;
		}
		else{
			return -1;
		}
	}
	printk("[I2C RAW] RAW response len: %d\n", i2c_mcumgr_next_buf->length);

	return 0;
}

int i2c_mcumgr_send(const uint8_t *data, int len){
	unsigned int key = irq_lock();
	if (i2c_mcumgr_next_buf == NULL){
		i2c_mcumgr_next_buf = i2c_mcumgr_alloc_tx();
		if(i2c_mcumgr_next_buf == NULL){
			printk("[I2C mcumgr] mem alloc failed \n");
			/* Mem allocation failed. Drop response */
			irq_unlock(key);
			return -1;
		}
		i2c_mcumgr_next_buf->send = 0;
		i2c_mcumgr_next_buf->length = 0;
	}
	if (len > sizeof(i2c_mcumgr_next_buf->data)){
		/* Response is too big. Dropping */
		i2c_mcumgr_free_tx_buf(i2c_mcumgr_next_buf);
		i2c_mcumgr_next_buf = NULL;
		printk("[I2C mcumgr] Response too long \n");
		irq_unlock(key);
		return -1;
	}
	printk("[I2C mcumgr] ok i have new response len: %d\n", len);
	int ret = mcumgr_serial_tx_pkt(data, len, i2c_mcumgr_store_raw);
	irq_unlock(key);
	return ret;
};
int i2c_mcumgr_stop(struct i2c_target_config *config){
	printk("Stop i2c mcumgr\n");
	/* Response is too big. Dropping */
	i2c_mcumgr_free_tx_buf(i2c_mcumgr_next_buf);
	i2c_mcumgr_next_buf = NULL;
	
}
static void i2c_mcumgr_setup(const struct device *i2c)
{
	i2c_target_callback.write_received = i2c_mcumgr_wrec_isr;
	i2c_target_callback.write_requested = i2c_mcumgr_wreq_isr;
	i2c_target_callback.read_requested = i2c_mcumgr_rreq_isr;
	i2c_target_callback.read_processed = i2c_mcumgr_rpro_isr;
	// i2c_target_callback.stop = i2c_mcumgr_stop;

	i2c_slave_conf.address = 0xA; // TODO put it in Device Tree (or conf)!
	i2c_slave_conf.callbacks = &i2c_target_callback;

	int error = i2c_target_register(i2c, &i2c_slave_conf);
	if(error){
		assert_print("Can't register device I2C: %d", error);
	}
	/*TODO empty FIFO ?*/
}



void i2c_mcumgr_register(i2c_mcumgr_recv_fn *cb)
{
	i2c_mgumgr_recv_cb = cb;


	if (device_is_ready(i2c_mcumgr_dev)) {
		i2c_mcumgr_setup(i2c_mcumgr_dev);
	}
}
