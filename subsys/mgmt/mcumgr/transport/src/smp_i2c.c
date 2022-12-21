#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/net/buf.h>
#include <zephyr/drivers/console/i2c_mcumgr.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>
#include <zephyr/mgmt/mcumgr/transport/smp.h>
#include <zephyr/mgmt/mcumgr/transport/serial.h>

struct device;
static struct smp_transport smp_i2c_transport;

static uint16_t smp_i2c_get_mtu(const struct net_buf *nb)
{
	return CONFIG_MCUMGR_SMP_I2C_MTU;
}

static void smp_i2c_rx_frag(struct i2c_mcumgr_rx_buf *rx_buf)
{
    /* TODO !
	k_fifo_put(&smp_uart_rx_fifo, rx_buf);
	k_work_submit(&smp_uart_work);
    */
}

static int smp_i2c_tx_pkt(struct net_buf *nb)
{
    /* TODO !
	int rc;

	rc = uart_mcumgr_send(nb->data, nb->len);
	smp_packet_free(nb);

	return rc;
    */
}

static int smp_i2c_init(const struct device* dev){
	ARG_UNUSED(dev);
	smp_transport_init(&smp_i2c_transport, smp_i2c_tx_pkt,
            smp_i2c_get_mtu, NULL, NULL, NULL);
	i2c_mcumgr_register(smp_i2c_rx_frag);

	return 0;
}

SYS_INIT(smp_i2c_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);