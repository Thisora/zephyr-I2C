#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/net/buf.h>
#include <zephyr/drivers/console/i2c_mcumgr.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>
#include <zephyr/mgmt/mcumgr/transport/smp.h>
#include <zephyr/mgmt/mcumgr/transport/serial.h>
#include <mgmt/mcumgr/transport/smp_internal.h>
struct device;
static struct mcumgr_serial_rx_ctxt smp_i2c_rx_ctxt;
static struct smp_transport smp_i2c_transport;

static void smp_i2c_process_rx_queue(struct k_work *work);

K_FIFO_DEFINE(smp_i2c_rx_fifo);
K_WORK_DEFINE(smp_i2c_work, smp_i2c_process_rx_queue);


static uint16_t smp_i2c_get_mtu(const struct net_buf *nb)
{
	return CONFIG_MCUMGR_SMP_I2C_MTU;
}


/**
 * Process a single fragment received from I2C driver
 */
static void smp_i2c_process_frag(struct i2c_mcumgr_rx_buf *rx_buf)
{
	struct net_buf *nb;

	/* Decode the fragment and write the result to the global receive
	 * context.
	 */
	nb = mcumgr_serial_process_frag(&smp_i2c_rx_ctxt,
					rx_buf->data, rx_buf->length);

	/* Release the encoded fragment. */
	i2c_mcumgr_free_rx_buf(rx_buf);

	/* If a complete packet has been received, pass it to SMP for
	 * processing.
	 */
	if (nb != NULL) {
		smp_rx_req(&smp_i2c_transport, nb);
	}
}

/**
 * Process every fragment arrived (in FIFO)
 */
static void smp_i2c_process_rx_queue(struct k_work *work){
    struct i2c_mcumgr_rx_buf *rx_buf;
    while ((rx_buf = k_fifo_get(&smp_i2c_rx_fifo, K_NO_WAIT)) != NULL) {
		smp_i2c_process_frag(rx_buf);
	}
}

/**
 * ISR context. Starting a new work to leave it
 */
static void smp_i2c_rx_frag(struct i2c_mcumgr_rx_buf *rx_buf)
{
	k_fifo_put(&smp_i2c_rx_fifo, rx_buf);
	k_work_submit(&smp_i2c_work);

}

static int smp_i2c_tx_pkt(struct net_buf *nb)
{

	int rc;

	rc = i2c_mcumgr_send(nb->data, nb->len);
	smp_packet_free(nb);

	return rc;

}

static int smp_i2c_init(const struct device* dev){
	ARG_UNUSED(dev);
	smp_transport_init(&smp_i2c_transport, smp_i2c_tx_pkt,
            smp_i2c_get_mtu, NULL, NULL, NULL);
	i2c_mcumgr_register(smp_i2c_rx_frag);

	return 0;
}

SYS_INIT(smp_i2c_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);