/*
 * Cypress CYRF6936 WirelessUSB device driver
 *
 * Copyright (c) 2011 Mikhail Gruzdev <michail.gruzdev@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/module.h>
#include <linux/if_arp.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/spi/spi.h>
#include <net/iw_handler.h>
#include "cyrf6936.h"

MODULE_DESCRIPTION("Cypress CYRF6936 wireless network driver");
MODULE_AUTHOR("Mikhail Gruzdev <michail.gruzdev@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:cyrf6936");

static int msg_level;
module_param_named(message, msg_level, int, 0);
MODULE_PARM_DESC(message, "message verbosity level (0 = none, 31 = all)");

#define CYRF6936_POLL_PERIOD		1
#define CYRF6936_OP_READ		0x00
#define CYRF6936_OP_WRITE		0x80

struct cyrf6936_net {
	struct net_device *netdev;
	struct spi_device *spi;
	struct mutex lock;
	struct sk_buff *tx_skb;
	struct net_device_stats stats;
	u32 msg_enable;
	struct spi_message spi_msg;
	struct spi_transfer spi_xfer;
	u8 spi_rxbuf[2];
	u8 spi_txbuf[2];
	struct work_struct poll_work;
	struct work_struct tx_work;
	int pollmode;
	struct timer_list poll_timer;
#ifdef CONFIG_WIRELESS_EXT
	spinlock_t iwstats_lock;
	u8 rssi;
	struct iw_statistics iwstats;
#endif
};

static void cyrf6936_wreg(struct cyrf6936_net *p, u8 addr, u8 val)
{
	struct spi_message *msg = &p->spi_msg;

	p->spi_txbuf[0] = addr | CYRF6936_OP_WRITE;
	p->spi_txbuf[1] = val;
	if (spi_sync(p->spi, msg) < 0)
		dev_err(&p->spi->dev, "spi_sync() failed\n");
}

static u8 cyrf6936_rreg(struct cyrf6936_net *p, u8 addr)
{
	struct spi_message *msg = &p->spi_msg;

	p->spi_txbuf[0] = addr | CYRF6936_OP_READ;
	if (spi_sync(p->spi, msg) < 0)
		dev_err(&p->spi->dev, "spi_sync() failed\n");
	return p->spi_rxbuf[1];
}

#ifdef CONFIG_WIRELESS_EXT
static int cyrf6936_iw_getname(struct net_device *dev,
			struct iw_request_info *info, char *name, char *extra)
{
	strcpy(name, "WirelessUSB");
	return 0;
}

static struct iw_statistics *cyrf6936_iw_stats(struct net_device *dev)
{
	struct cyrf6936_net *p = netdev_priv(dev);

	spin_lock(&p->iwstats_lock);
	p->iwstats.qual.level = p->rssi;
	spin_unlock(&p->iwstats_lock);
	return &p->iwstats;
}

static const iw_handler cyrf6936_iw_handlers[] = {
	(iw_handler) NULL,				/* SIOCSIWCOMMIT */
	(iw_handler) cyrf6936_iw_getname,		/* SIOCGIWNAME */
};

static const struct iw_handler_def cyrf6936_iwdef = {
	.standard		= cyrf6936_iw_handlers,
	.num_standard		= ARRAY_SIZE(cyrf6936_iw_handlers),
	.get_wireless_stats	= cyrf6936_iw_stats,
};

static void cyrf6936_iw_setup(struct cyrf6936_net *p)
{
	if (netif_msg_drv(p))
		pr_debug("cyrf6936: using wireless extensions\n");
	spin_lock_init(&p->iwstats_lock);
	p->netdev->wireless_handlers = &cyrf6936_iwdef;
}

static void cyrf6936_iw_rssi(struct cyrf6936_net *p)
{
	u8 val = cyrf6936_rreg(p, RSSI);
	if (val & SOP) {
		spin_lock(&p->iwstats_lock);
		p->rssi = val & RSSI_VAL_MASK;
		spin_unlock(&p->iwstats_lock);
	}
}
#else
static void cyrf6936_iw_setup(struct cyrf6936_net *p) {}
static void cyrf6936_iw_rssi(struct cyrf6936_net *p) {}
#endif

static void cyrf6936_rx_enable(struct cyrf6936_net *p)
{
	cyrf6936_wreg(p, RX_CTRL, RX_GO | RXC_IRQEN | RXE_IRQEN);
	cyrf6936_wreg(p, CLK_OVERRIDE, RXF);
	cyrf6936_wreg(p, XACT_CFG, END_STATE_RX);
}

static void cyrf6936_rx_tx(struct cyrf6936_net *p)
{
	u8 val, *data;
	size_t rx_len;
	struct sk_buff *skb;

	/* update signal level */
	cyrf6936_iw_rssi(p);
	/* rx */
	val = cyrf6936_rreg(p, RX_IRQ_STATUS);
	if (val & RXE_IRQ)
		goto err_rxe;
	if (val & RXC_IRQ) {
		/* debouncing, 2nd read */
		val = cyrf6936_rreg(p, RX_IRQ_STATUS);
		if (val & RXE_IRQ)
			goto err_rxe;
		/* get data length*/
		rx_len = cyrf6936_rreg(p, RX_LENGTH);
		/* allocate buffer */
		skb = dev_alloc_skb(rx_len + NET_IP_ALIGN);
		if (!skb)
			goto err_oom;
		skb_reserve(skb, NET_IP_ALIGN);
		data = skb_put(skb, rx_len);
		/* read data */
		cyrf6936_wreg(p, RX_IRQ_STATUS, RXOW_IRQ);
		while (rx_len--)
			*data++ = cyrf6936_rreg(p, RX_BUFFER);
		/* dump received packet data */
		if (netif_msg_pktdata(p))
			print_hex_dump_bytes("cyrf6936 rx data: ",
					DUMP_PREFIX_NONE, skb->data, skb->len);
		skb->dev = p->netdev;
		skb->protocol = htons(ETH_P_ALL);
		skb->ip_summed = CHECKSUM_UNNECESSARY;
		p->stats.rx_packets++;
		p->stats.rx_bytes += rx_len;
		netif_rx_ni(skb);
		cyrf6936_rx_enable(p);
	}
	/* tx */
	val = cyrf6936_rreg(p, TX_IRQ_STATUS);
	if (val & TXE_IRQ)
		goto err_txe;
	if (val & TXC_IRQ) {
		/* debouncing, 2nd read */
		val = cyrf6936_rreg(p, TX_IRQ_STATUS);
		if (val & TXE_IRQ)
			goto err_txe;
		/* tx ok*/
		p->stats.tx_packets++;
		p->stats.tx_bytes += p->tx_skb->len;
		if (netif_msg_tx_done(p))
			dev_dbg(&p->netdev->dev, "tx done\n");
		dev_kfree_skb(p->tx_skb);
		p->tx_skb = NULL;
		netif_wake_queue(p->netdev);
		cyrf6936_rx_enable(p);
	}

	if (!p->pollmode)
		enable_irq(p->netdev->irq);
	return;
err_rxe:
	if (netif_msg_rx_err(p))
		dev_info(&p->netdev->dev, "rx error\n");
	p->stats.rx_errors++;
	cyrf6936_rx_enable(p);
	return;
err_txe:
	if (netif_msg_tx_err(p))
		dev_info(&p->netdev->dev, "tx error\n");
	p->stats.tx_errors++;
	cyrf6936_rx_enable(p);
	return;
err_oom:
	dev_err(&p->netdev->dev, "out of memory, packet dropped\n");
	p->stats.rx_dropped++;
	cyrf6936_rx_enable(p);
}

static void cyrf6936_poll_work(struct work_struct *work)
{
	struct cyrf6936_net *p = container_of(work, struct cyrf6936_net, poll_work);
	mutex_lock(&p->lock);
	cyrf6936_rx_tx(p);
	mutex_unlock(&p->lock);
}

static void cyrf6936_tx_work(struct work_struct *work)
{
	struct cyrf6936_net *p = container_of(work, struct cyrf6936_net, tx_work);
	struct sk_buff *skb;
	u8 *data;
	size_t len;

	mutex_lock(&p->lock);
	skb = p->tx_skb;
	data = skb->data;
	len = skb->len;
	/* prepare transmission */
	cyrf6936_wreg(p, XACT_CFG, FRC_END | END_STATE_IDLE);
	/* write message length */
	cyrf6936_wreg(p, TX_LENGTH, len);
	/* clear tx buffer */
	cyrf6936_wreg(p, TX_CTRL, TX_CLR);
	/* fill tx buffer */
	while (len--)
		cyrf6936_wreg(p, TX_BUFFER, *data++);
	/* start transmision, enable Transmission Complete interrupt */
	cyrf6936_wreg(p, TX_CTRL, TX_GO | TXC_IRQEN);
	mutex_unlock(&p->lock);
}

static void cyrf6936_poll_func(unsigned long data)
{
	struct cyrf6936_net *p = (struct cyrf6936_net *)data;
	schedule_work(&p->poll_work);
	p->poll_timer.expires += CYRF6936_POLL_PERIOD * HZ;
	add_timer(&p->poll_timer);
}

static int cyrf6936_net_open(struct net_device *dev)
{
	struct cyrf6936_net *p = netdev_priv(dev);
	u8 sop[] = {0xdf, 0xb1, 0xc0, 0x49, 0x62, 0xdf, 0xc1, 0x49};
	int i;

	if (netif_msg_ifup(p))
		dev_dbg(&p->netdev->dev, "openinig network device\n");

	mutex_lock(&p->lock);
	/* initialize registers */
	cyrf6936_wreg(p, MODE_OVERRIDE, RST);
	cyrf6936_wreg(p, CLK_EN, RXF);
	cyrf6936_wreg(p, CHANNEL, 96);
	cyrf6936_wreg(p, TX_CFG, LEN | MODE_8DR | PA_4);
	cyrf6936_wreg(p, RX_CTRL, 0);
	cyrf6936_wreg(p, RX_CFG, AGC_EN | FAST_TURN_EN | RXOW_EN);
	cyrf6936_wreg(p, XACT_CFG, END_STATE_RX | ACK_TO_15x);
	cyrf6936_wreg(p, FRAMING_CFG, SOP_EN | SOP_LEN | LEN_EN | 0x0e);
	cyrf6936_wreg(p, DATA32_THOLD, 0x08);
	cyrf6936_wreg(p, DATA64_THOLD, 0x0e);
	cyrf6936_wreg(p, CRC_SEED_LSB, 0x14);
	cyrf6936_wreg(p, CRC_SEED_MSB, 0x14);
	cyrf6936_wreg(p, TX_OFFSET_LSB, 0x55);
	cyrf6936_wreg(p, TX_OFFSET_MSB, 0x05);
	cyrf6936_wreg(p, RX_OVERRIDE, DIS_CRC0);
	cyrf6936_wreg(p, XTAL_CFG, START_DLY);
	cyrf6936_wreg(p, AUTO_CAL_TIME, 0x3c);
	cyrf6936_wreg(p, AUTO_CAL_OFFSET, 0x14);
	cyrf6936_wreg(p, XTAL_CTRL, XOUT_FN_PA | FREQ_12M);
	for (i = 0; i < ARRAY_SIZE(sop); i++)
		cyrf6936_wreg(p, SOP_CODE, sop[i]);
	/* set rx mode */
	cyrf6936_rx_enable(p);
	if (p->pollmode) {
		/* start poll timer */
		p->poll_timer.function = cyrf6936_poll_func;
		p->poll_timer.data = (unsigned long)p;
		p->poll_timer.expires = jiffies + CYRF6936_POLL_PERIOD * HZ;
		add_timer(&p->poll_timer);
	}
	/* ready to receive data*/
	netif_start_queue(dev);
	mutex_unlock(&p->lock);
	return 0;
}

static int cyrf6936_net_stop(struct net_device *dev)
{
	struct cyrf6936_net *p = netdev_priv(dev);

	if (netif_msg_ifdown(p))
		dev_dbg(&p->netdev->dev, "closing network device\n");

	mutex_lock(&p->lock);
	netif_stop_queue(dev);
	if (p->pollmode)
		del_timer_sync(&p->poll_timer);
	flush_scheduled_work();
	if (p->tx_skb)
		dev_kfree_skb(p->tx_skb);
	mutex_unlock(&p->lock);

	return 0;
}

static int cyrf6936_net_tx(struct sk_buff *skb, struct net_device *dev)
{
	struct cyrf6936_net *p = netdev_priv(dev);
	netif_stop_queue(dev);
	p->tx_skb = skb;
	schedule_work(&p->tx_work);
	return 0;
}

static struct net_device_stats *cyrf6936_net_stats(struct net_device *dev)
{
	struct cyrf6936_net *p = netdev_priv(dev);
	return &p->stats;
}

static void cyrf6936_net_setup(struct net_device *dev)
{
	dev->type		= ARPHRD_NONE;
	dev->mtu		= 16;
	dev->addr_len		= 0;
	dev->tx_queue_len	= 10;
	dev->flags		= IFF_NOARP;
	dev->features		= NETIF_F_NO_CSUM;
	dev->hard_header_len	= 0;
	dev->open		= cyrf6936_net_open;
	dev->stop		= cyrf6936_net_stop;
	dev->hard_start_xmit	= cyrf6936_net_tx;
	dev->get_stats		= cyrf6936_net_stats;
}

static irqreturn_t cyrf6936_irq(int irq, void *dev_id)
{
	struct cyrf6936_net *p = dev_id;

	disable_irq_nosync(irq);
	schedule_work(&p->poll_work);
	return IRQ_HANDLED;
}

static int __devinit cyrf6936_probe(struct spi_device *spi)
{
	struct net_device *dev;
	struct cyrf6936_net *p;
	int ret;

	dev = alloc_netdev(sizeof(*p), "cwu%d", cyrf6936_net_setup);
	if (!dev) {
		dev_err(&spi->dev, "failed to allocate device\n");
		return -ENOMEM;
	}

	p = netdev_priv(dev);
	mutex_init(&p->lock);
	p->spi = spi;
	p->netdev = dev;
	p->msg_enable = netif_msg_init(msg_level, NETIF_MSG_DRV);

	INIT_WORK(&p->poll_work, cyrf6936_poll_work);
	INIT_WORK(&p->tx_work, cyrf6936_tx_work);

	SET_NETDEV_DEV(dev, &spi->dev);
	spi_set_drvdata(spi, p);

	/* initialize pre-made spi message */
	p->spi_xfer.len = 2;
	p->spi_xfer.bits_per_word = 16;
	p->spi_xfer.rx_buf = &p->spi_rxbuf;
	p->spi_xfer.tx_buf = &p->spi_txbuf;
	spi_message_init(&p->spi_msg);
	spi_message_add_tail(&p->spi_xfer, &p->spi_msg);

	/* reset device */
	cyrf6936_wreg(p, MODE_OVERRIDE, RST);

	/* check if chip is connected */
	if (cyrf6936_rreg(p, EOP_CTRL) != DEFVAL_EOP_CTRL) {
		if (netif_msg_probe(p))
			dev_err(&spi->dev, "chip not found\n");
		ret = -ENODEV;
		goto err_chk;
	}

	/* switch irq/poll mode */
	if (spi->irq) {
		p->netdev->irq = spi->irq;
		ret = request_irq(spi->irq, cyrf6936_irq, 0, dev->name, p);
		if (ret < 0) {
			if (netif_msg_probe(p))
				dev_err(&spi->dev, "request irq failed");
			goto err_irq;
		}
	} else {
		p->pollmode = 1;
		init_timer(&p->poll_timer);
	}

	/* setup wireless extensions */
	cyrf6936_iw_setup(p);

	ret = register_netdev(dev);
	if (ret) {
		if (netif_msg_probe(p))
			dev_err(&spi->dev, "register netdev error\n");
		goto err_reg;
	}

	return 0;

err_reg:
	free_irq(spi->irq, p);
err_chk:
err_irq:
	free_netdev(dev);
	return ret;
}

static int __devexit cyrf6936_remove(struct spi_device *spi)
{
	struct cyrf6936_net *p = dev_get_drvdata(&spi->dev);

	if (netif_msg_drv(p))
		dev_info(&spi->dev, "remove\n");
	unregister_netdev(p->netdev);
	if (p->pollmode)
		del_timer_sync(&p->poll_timer);
	else
		free_irq(spi->irq, p);
	free_netdev(p->netdev);
	return 0;
}

static struct spi_driver cyrf6936_driver = {
	.driver = {
		.name = "cyrf6936",
		.owner = THIS_MODULE,
	},
	.probe = cyrf6936_probe,
	.remove = __devexit_p(cyrf6936_remove),
};

static int __init cyrf6936_init(void)
{
	return spi_register_driver(&cyrf6936_driver);
}

static void __exit cyrf6936_exit(void)
{
	spi_unregister_driver(&cyrf6936_driver);
}

module_init(cyrf6936_init);
module_exit(cyrf6936_exit);
