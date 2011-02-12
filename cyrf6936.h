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
#ifndef CYRF6936_H_
#define CYRF6936_H_

#define CHANNEL				0x00
#define TX_LENGTH			0x01
#define TX_CTRL				0x02
#  define TX_GO				(1 << 7)
#  define TX_CLR			(1 << 6)
#  define TXB15_IRQEN			(1 << 5)
#  define TXB8_IRQEN			(1 << 4)
#  define TXB0_IRQEN			(1 << 3)
#  define TXBERR_IRQEN			(1 << 2)
#  define TXC_IRQEN			(1 << 1)
#  define TXE_IRQEN			(1 << 0)
#define TX_CFG				0x03
#  define LEN				(1 << 5)
#  define MODE_MASK			0x18
#  define MODE_GFSK			(0 << 3)
#  define MODE_8DR			(1 << 3)
#  define MODE_DDR			(2 << 3)
#  define MODE_SDR			(3 << 3)
#  define PA_MASK			0x07
#  define PA_35				(0 << 0)
#  define PA_30				(1 << 0)
#  define PA_24				(2 << 0)
#  define PA_18				(3 << 0)
#  define PA_13				(4 << 0)
#  define PA_5				(5 << 0)
#  define PA_0				(6 << 0)
#  define PA_4				(7 << 0)
#define TX_IRQ_STATUS			0x04
#  define OS_IRQ			(1 << 7)
#  define LV_IRQ			(1 << 6)
#  define TXB15_IRQ			(1 << 5)
#  define TXB8_IRQ			(1 << 4)
#  define TXB0_IRQ			(1 << 3)
#  define TXBERR_IRQ			(1 << 2)
#  define TXC_IRQ			(1 << 1)
#  define TXE_IRQ			(1 << 0)
#define RX_CTRL				0x05
#  define RX_GO				(1 << 7)
#  define RXB16_IRQEN			(1 << 5)
#  define RXB8_IRQEN			(1 << 4)
#  define RXB1_IRQEN			(1 << 3)
#  define RXBERR_IRQEN			(1 << 2)
#  define RXC_IRQEN			(1 << 1)
#  define RXE_IRQEN			(1 << 0)
#define RX_CFG				0x06
#  define AGC_EN			(1 << 7)
#  define LNA_EN			(1 << 6)
#  define ATT				(1 << 5)
#  define HILO				(1 << 4)
#  define FAST_TURN_EN			(1 << 3)
#  define RXOW_EN			(1 << 1)
#  define VLD_EN			(1 << 0)
#define RX_IRQ_STATUS			0x07
#  define RXOW_IRQ			(1 << 7)
#  define SOPDET_IRQ			(1 << 6)
#  define RXB16_IRQ			(1 << 5)
#  define RXB8_IRQ			(1 << 4)
#  define RXB1_IRQ			(1 << 3)
#  define RXBERR_IRQ			(1 << 2)
#  define RXC_IRQ			(1 << 1)
#  define RXE_IRQ			(1 << 0)
#define RX_STATUS			0x08
#  define RX_ACK			(1 << 7)
#  define PKT_ERR			(1 << 6)
#  define EOP_ERR			(1 << 5)
#  define CRC0				(1 << 4)
#  define BAD_CRC			(1 << 3)
#  define RX_CODE			(1 << 2)
#  define RX_DATA_MODE_MASK		0x03
#  define RX_DATA_MODE_GFSK		(0 << 0)
#  define RX_DATA_MODE_8DR		(1 << 0)
#  define RX_DATA_MODE_DDR		(2 << 0)
#  define RX_DATA_MODE_NVAL		(3 << 0)
#define RX_COUNT			0x09
#define RX_LENGTH			0x0a
#define PWR_CTRL			0x0b
#  define PMU_EN			(1 << 7)
#  define LVIRQ_EN			(1 << 6)
#  define PMU_MODE_FORCE		(1 << 5)
#  define LVI_TH_MASK			0x0c
#  define LVI_TH_PMU_OUTV		(0 << 2)
#  define LVI_TH_2V2			(1 << 2)
#  define LVI_TH_2V			(2 << 2)
#  define LVI_TH_1V8			(3 << 2)
#  define PMU_OUTV_MASK			0x03
#  define PMU_OUTV_2V7			(0 << 0)
#  define PMU_OUTV_2V6			(1 << 0)
#  define PMU_OUTV_2V5			(2 << 0)
#  define PMU_OUTV_2V4			(3 << 0)
#define XTAL_CTRL			0x0c
#  define XOUT_FN_MASK			0x0c
#  define XOUT_FN_CLK			(0 << 6)
#  define XOUT_FN_PA			(1 << 6)
#  define XOUT_FN_DATA			(2 << 6)
#  define XOUT_FN_GPIO			(3 << 6)
#  define XSIRQ_EN			(1 << 5)
#  define FREQ_MASK			0x07
#  define FREQ_12M			(0 << 0)
#  define FREQ_6M			(1 << 0)
#  define FREQ_3M			(2 << 0)
#  define FREQ_1M5			(3 << 0)
#  define FREQ_0M75			(4 << 0)
#define IO_CFG				0x0d
#  define IRQ_OD			(1 << 7)
#  define IRQ_POL			(1 << 6)
#  define MISO_OD			(1 << 5)
#  define XOUT_OD			(1 << 4)
#  define PACTL_OD			(1 << 3)
#  define PACTL_GPIO			(1 << 2)
#  define SPI_3PIN			(1 << 1)
#  define IRQ_GPIO			(1 << 0)
#define GPIO_CTRL			0x0e
#  define XOUT_OP			(1 << 7)
#  define MISO_OP			(1 << 6)
#  define PACTL_OP			(1 << 5)
#  define IRQ_OP			(1 << 4)
#  define XOUT_iP			(1 << 3)
#  define MISO_IP			(1 << 2)
#  define PACTL_IP			(1 << 1)
#  define IRQ_IP			(1 << 0)
#define XACT_CFG			0x0f
#  define ACK_EN			(1 << 7)
#  define FRC_END			(1 << 5)
#  define END_STATE_MASK		0x1c
#  define END_STATE_SLEEP		(0 << 2)
#  define END_STATE_IDLE		(1 << 2)
#  define END_STATE_RSYNTH		(2 << 2)
#  define END_STATE_TSYNTH		(3 << 2)
#  define END_STATE_RX			(4 << 2)
#  define ACK_TO_MASK			0x03
#  define ACK_TO_4x			(0 << 0)
#  define ACK_TO_8x			(1 << 0)
#  define ACK_TO_12x			(2 << 0)
#  define ACK_TO_15x			(3 << 0)
#define FRAMING_CFG			0x10
#  define SOP_EN			(1 << 7)
#  define SOP_LEN			(1 << 6)
#  define LEN_EN			(1 << 5)
#  define SOP_TH			0
#  define SOP_TH_MASK			0x1f
#define DATA32_THOLD			0x11
#  define TH32				0
#  define TH32_MASK			0x0f
#define DATA64_THOLD			0x12
#  define TH64				0
#  define TH64_MASK			0x1f
#define RSSI				0x13
#  define SOP				(1 << 7)
#  define LNA				(1 << 5)
#  define RSSI_VAL			0
#  define RSSI_VAL_MASK			0x1f
#define EOP_CTRL			0x14
#  define HEN				(1 << 7)
#  define HINT				4
#  define HINT_MASK			0x70
#  define EOP				0
#  define EOP_MASK			0x0f
#define CRC_SEED_LSB			0x15
#define CRC_SEED_MSB			0x16
#define TX_CRC_LSB			0x17
#define TX_CRC_MSB			0x18
#define RX_CRC_LSB			0x19
#define RX_CRC_MSB			0x1a
#define TX_OFFSET_LSB			0x1b
#define TX_OFFSET_MSB			0x1c
#  define MODE_OVERRIDE			0x1d
#  define FRC_SEN			(1 << 5)
#  define FRC_AWAKE_MASK		0x18
#  define FRC_AWAKE			(3 << 3)
#  define RST				(1 << 0)
#define RX_OVERRIDE			0x1e
#  define ACK_RX			(1 << 7)
#  define RXTX_DLY			(1 << 6)
#  define MAN_RXACK			(1 << 5)
#  define FRC_RXDR			(1 << 4)
#  define DIS_CRC0			(1 << 3)
#  define DIS_RXCRC			(1 << 2)
#  define ACE				(1 << 1)
#define TX_OVERRIDE			0x1f
#  define ACK_TX			(1 << 7)
#  define FRC_PRE			(1 << 6)
#  define MAN_TXACK			(1 << 4)
#  define OVRD_ACK			(1 << 3)
#  define DIS_TXCRC			(1 << 2)
#  define TX_INV			(1 << 0)
#define XTAL_CFG			0x26
#  define START_DLY			(1 << 3)
#define CLK_OVERRIDE			0x27
#  define RXF				(1 << 1)
#define CLK_EN				0x28
#  define RXF				(1 << 1)
#define RX_ABORT			0x29
#  define EN				(1 << 5)
#define AUTO_CAL_TIME			0x32
#define AUTO_CAL_OFFSET			0x35
#define ANALOG_CTRL			0x39
#  define RX_INV			(1 << 1)
#  define ALL_SLOW			(1 << 0)
#define TX_BUFFER			0x20
#define RX_BUFFER			0x21
#define SOP_CODE			0x22
#define DATA_CODE			0x23
#define PREAMBLE			0x24
#define MFG_ID				0x25

#define DEFVAL_TX_LENGTH		0x00
#define DEFVAL_TX_CTRL			0x03
#define DEFVAL_RX_CTRL			0x07
#define DEFVAL_RX_COUNT			0x00
#define DEFVAL_RX_LENGTH		0x00
#define DEFVAL_PWR_CTRL			0xa0
#define DEFVAL_IO_CFG			0x00
#define DEFVAL_FRAMING_CFG		0xa5
#define DEFVAL_EOP_CTRL			0xa4
#define DEFVAL_CRC_SEED_LSB		0x00
#define DEFVAL_CRC_SEED_MSB		0x00
#define DEFVAL_RX_CRC_LSB		0xff
#define DEFVAL_RX_CRC_MSB		0xff
#define DEFVAL_TX_OFFSET_LSB		0x00
#define DEFVAL_TX_OVERRIDE		0x00

#endif /* CYRF6936_H_ */
