/*
 *  Freescale lpuart serial port driver
 *
 *  Copyright 2012-2014 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#if defined(CONFIG_SERIAL_FSL_LPUART_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/tty_flip.h>
#include <asm/gpio.h>

/* All registers are 8-bit width */
#define UARTBDH			0x00
#define UARTBDL			0x01
#define UARTCR1			0x02
#define UARTCR2			0x03
#define UARTSR1			0x04
#define UARTCR3			0x06
#define UARTDR			0x07
#define UARTCR4			0x0a
#define UARTCR5			0x0b
#define UARTMODEM		0x0d
#define UARTPFIFO		0x10
#define UARTCFIFO		0x11
#define UARTSFIFO		0x12
#define UARTTWFIFO		0x13
#define UARTTCFIFO		0x14
#define UARTRWFIFO		0x15

#define UARTBDH_LBKDIE		0x80
#define UARTBDH_RXEDGIE		0x40
#define UARTBDH_SBR_MASK	0x1f

#define UARTCR1_LOOPS		0x80
#define UARTCR1_RSRC		0x20
#define UARTCR1_M		0x10
#define UARTCR1_WAKE		0x08
#define UARTCR1_ILT		0x04
#define UARTCR1_PE		0x02
#define UARTCR1_PT		0x01

#define UARTCR2_TIE		0x80
#define UARTCR2_TCIE		0x40
#define UARTCR2_RIE		0x20
#define UARTCR2_ILIE		0x10
#define UARTCR2_TE		0x08
#define UARTCR2_RE		0x04
#define UARTCR2_RWU		0x02
#define UARTCR2_SBK		0x01

#define UARTSR1_TDRE		0x80
#define UARTSR1_TC		0x40
#define UARTSR1_RDRF		0x20
#define UARTSR1_IDLE		0x10
#define UARTSR1_OR		0x08
#define UARTSR1_NF		0x04
#define UARTSR1_FE		0x02
#define UARTSR1_PE		0x01

#define UARTCR3_R8		0x80
#define UARTCR3_T8		0x40
#define UARTCR3_TXDIR		0x20
#define UARTCR3_TXINV		0x10
#define UARTCR3_ORIE		0x08
#define UARTCR3_NEIE		0x04
#define UARTCR3_FEIE		0x02
#define UARTCR3_PEIE		0x01

#define UARTCR4_MAEN1		0x80
#define UARTCR4_MAEN2		0x40
#define UARTCR4_M10		0x20
#define UARTCR4_BRFA_MASK	0x1f
#define UARTCR4_BRFA_OFF	0

#define UARTCR5_TDMAS		0x80
#define UARTCR5_RDMAS		0x20

#define UARTMODEM_RXRTSE	0x08
#define UARTMODEM_TXRTSPOL	0x04
#define UARTMODEM_TXRTSE	0x02
#define UARTMODEM_TXCTSE	0x01

#define UARTPFIFO_TXFE		0x80
#define UARTPFIFO_FIFOSIZE_MASK	0x7
#define UARTPFIFO_TXSIZE_OFF	4
#define UARTPFIFO_RXFE		0x08
#define UARTPFIFO_RXSIZE_OFF	0

#define UARTCFIFO_TXFLUSH	0x80
#define UARTCFIFO_RXFLUSH	0x40
#define UARTCFIFO_RXOFE		0x04
#define UARTCFIFO_TXOFE		0x02
#define UARTCFIFO_RXUFE		0x01

#define UARTSFIFO_TXEMPT	0x80
#define UARTSFIFO_RXEMPT	0x40
#define UARTSFIFO_RXOF		0x04
#define UARTSFIFO_TXOF		0x02
#define UARTSFIFO_RXUF		0x01

/* 32-bit register defination */
#define UARTBAUD		0x00
#define UARTSTAT		0x04
#define UARTCTRL		0x08
#define UARTDATA		0x0C
#define UARTMATCH		0x10
#define UARTMODIR		0x14
#define UARTFIFO		0x18
#define UARTWATER		0x1c

#define UARTBAUD_MAEN1		0x80000000
#define UARTBAUD_MAEN2		0x40000000
#define UARTBAUD_M10		0x20000000
#define UARTBAUD_TDMAE		0x00800000
#define UARTBAUD_RDMAE		0x00200000
#define UARTBAUD_MATCFG		0x00400000
#define UARTBAUD_BOTHEDGE	0x00020000
#define UARTBAUD_RESYNCDIS	0x00010000
#define UARTBAUD_LBKDIE		0x00008000
#define UARTBAUD_RXEDGIE	0x00004000
#define UARTBAUD_SBNS		0x00002000
#define UARTBAUD_SBR		0x00000000
#define UARTBAUD_SBR_MASK	0x1fff

#define UARTSTAT_LBKDIF		0x80000000
#define UARTSTAT_RXEDGIF	0x40000000
#define UARTSTAT_MSBF		0x20000000
#define UARTSTAT_RXINV		0x10000000
#define UARTSTAT_RWUID		0x08000000
#define UARTSTAT_BRK13		0x04000000
#define UARTSTAT_LBKDE		0x02000000
#define UARTSTAT_RAF		0x01000000
#define UARTSTAT_TDRE		0x00800000
#define UARTSTAT_TC		0x00400000
#define UARTSTAT_RDRF		0x00200000
#define UARTSTAT_IDLE		0x00100000
#define UARTSTAT_OR		0x00080000
#define UARTSTAT_NF		0x00040000
#define UARTSTAT_FE		0x00020000
#define UARTSTAT_PE		0x00010000
#define UARTSTAT_MA1F		0x00008000
#define UARTSTAT_M21F		0x00004000

#define UARTCTRL_R8T9		0x80000000
#define UARTCTRL_R9T8		0x40000000
#define UARTCTRL_TXDIR		0x20000000
#define UARTCTRL_TXINV		0x10000000
#define UARTCTRL_ORIE		0x08000000
#define UARTCTRL_NEIE		0x04000000
#define UARTCTRL_FEIE		0x02000000
#define UARTCTRL_PEIE		0x01000000
#define UARTCTRL_TIE		0x00800000
#define UARTCTRL_TCIE		0x00400000
#define UARTCTRL_RIE		0x00200000
#define UARTCTRL_ILIE		0x00100000
#define UARTCTRL_TE		0x00080000
#define UARTCTRL_RE		0x00040000
#define UARTCTRL_RWU		0x00020000
#define UARTCTRL_SBK		0x00010000
#define UARTCTRL_MA1IE		0x00008000
#define UARTCTRL_MA2IE		0x00004000
#define UARTCTRL_IDLECFG	0x00000100
#define UARTCTRL_LOOPS		0x00000080
#define UARTCTRL_DOZEEN		0x00000040
#define UARTCTRL_RSRC		0x00000020
#define UARTCTRL_M		0x00000010
#define UARTCTRL_WAKE		0x00000008
#define UARTCTRL_ILT		0x00000004
#define UARTCTRL_PE		0x00000002
#define UARTCTRL_PT		0x00000001

#define UARTDATA_NOISY		0x00008000
#define UARTDATA_PARITYE	0x00004000
#define UARTDATA_FRETSC		0x00002000
#define UARTDATA_RXEMPT		0x00001000
#define UARTDATA_IDLINE		0x00000800
#define UARTDATA_MASK		0x3ff

#define UARTMODIR_IREN		0x00020000
#define UARTMODIR_TXCTSSRC	0x00000020
#define UARTMODIR_TXCTSC	0x00000010
#define UARTMODIR_RXRTSE	0x00000008
#define UARTMODIR_TXRTSPOL	0x00000004
#define UARTMODIR_TXRTSE	0x00000002
#define UARTMODIR_TXCTSE	0x00000001

#define UARTFIFO_TXEMPT		0x00800000
#define UARTFIFO_RXEMPT		0x00400000
#define UARTFIFO_TXOF		0x00020000
#define UARTFIFO_RXUF		0x00010000
#define UARTFIFO_TXFLUSH	0x00008000
#define UARTFIFO_RXFLUSH	0x00004000
#define UARTFIFO_TXOFE		0x00000200
#define UARTFIFO_RXUFE		0x00000100
#define UARTFIFO_TXFE		0x00000080
#define UARTFIFO_FIFOSIZE_MASK	0x7
#define UARTFIFO_TXSIZE_OFF	4
#define UARTFIFO_RXFE		0x00000008
#define UARTFIFO_RXSIZE_OFF	0

#define UARTWATER_COUNT_MASK	0xff
#define UARTWATER_TXCNT_OFF	8
#define UARTWATER_RXCNT_OFF	24
#define UARTWATER_WATER_MASK	0xff
#define UARTWATER_TXWATER_OFF	0
#define UARTWATER_RXWATER_OFF	16

/* Rx DMA timeout in ms, which is used to calculate Rx ring buffer size */
//#define DMA_RX_TIMEOUT		(10)
//#define DMA_RX_TIMEOUT		(3)
#define DMA_RX_TIMEOUT			(5)

#define DRIVER_NAME	"fsl-lpuart"
#define DEV_NAME	"ttyLP"
#define UART_NR		6

static volatile int SilenceTimer=0;
/* uncomment this for using toggle of GPIO 88 when silence timer is reset */
//#define STGPIO_TEST
/* uncomment this for using toggle of GPIO 88 during the tx irq */
//#define TXINT_GPIO_TEST 1

static bool nodma = true;
module_param(nodma, bool, S_IRUGO);

struct lpuart_port {
	struct uart_port	port;
	struct clk		*clk;
	unsigned int		txfifo_size;
	unsigned int		rxfifo_size;
	bool			lpuart32;

	bool			lpuart_dma_tx_use;
	bool			lpuart_dma_rx_use;
	struct dma_chan		*dma_tx_chan;
	struct dma_chan		*dma_rx_chan;
	struct dma_async_tx_descriptor  *dma_tx_desc;
	struct dma_async_tx_descriptor  *dma_rx_desc;
	dma_cookie_t		dma_tx_cookie;
	dma_cookie_t		dma_rx_cookie;
	unsigned int		dma_tx_bytes;
	unsigned int		dma_rx_bytes;
	bool			dma_tx_in_progress;
	unsigned int		dma_rx_timeout;
	struct timer_list	lpuart_timer;
	struct scatterlist	rx_sgl, tx_sgl[2];
	struct circ_buf		rx_ring;
	int			rx_dma_rng_buf_len;
	unsigned int		dma_tx_nents;
	wait_queue_head_t	dma_wait;
};

static const struct of_device_id lpuart_dt_ids[] = {
	{
		.compatible = "fsl,vf610-lpuart",
	},
	{
		.compatible = "fsl,ls1021a-lpuart",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, lpuart_dt_ids);

/* Forward declare this for the dma callbacks*/
static void lpuart_dma_tx_complete(void *arg);

int lpuart_read_silencetimer(void)
{
        return SilenceTimer;
}
EXPORT_SYMBOL(lpuart_read_silencetimer);

void lpuart_reset_silencetimer(void)
{
#ifdef STGPIO_TEST
	static uint8_t gpio88val=0;
#endif

        if(SilenceTimer>=0) {
                SilenceTimer=0;
#ifdef STGPIO_TEST
		gpio88val^=1;
		gpio_set_value(88,gpioval);
#endif
	}
}
EXPORT_SYMBOL(lpuart_reset_silencetimer);

void lpuart_set_silencetimer(int val)
{
	SilenceTimer=val;
}
EXPORT_SYMBOL(lpuart_set_silencetimer);

static void lpuart_stop_tx(struct uart_port *port)
{
	unsigned char temp;

	temp = readb(port->membase + UARTCR2);
	temp &= ~(UARTCR2_TIE | UARTCR2_TCIE);
	writeb(temp, port->membase + UARTCR2);
}

static void lpuart_stop_rx(struct uart_port *port)
{
	unsigned char temp;

	temp = readb(port->membase + UARTCR2);
	writeb(temp & ~UARTCR2_RE, port->membase + UARTCR2);
}

static void lpuart_dma_tx(struct lpuart_port *sport)
{
	struct circ_buf *xmit = &sport->port.state->xmit;
	struct scatterlist *sgl = sport->tx_sgl;
	struct device *dev = sport->port.dev;
	int ret;

	if (sport->dma_tx_in_progress)
		return;

	sport->dma_tx_bytes = uart_circ_chars_pending(xmit);

	if (xmit->tail < xmit->head || xmit->head == 0) 		//we aren't in a wraparound, or head is at the beginning
	{
		sport->dma_tx_nents = 1;
		sg_init_one(sgl, xmit->buf + xmit->tail, sport->dma_tx_bytes);	//so we can just use one scatter gather table
	} 
	else 								//we are in the wraparound condition
	{
		sport->dma_tx_nents = 2;
		sg_init_table(sgl, 2);					//setup two scatter gather tables
		sg_set_buf(sgl, xmit->buf + xmit->tail,
				UART_XMIT_SIZE - xmit->tail);		//put buf..tail into the first buffer
		sg_set_buf(sgl + 1, xmit->buf, xmit->head);		//then buf..head into the second buffer
	}

	ret = dma_map_sg(dev, sgl, sport->dma_tx_nents, DMA_TO_DEVICE);
	if (!ret) {
		dev_err(dev, "DMA mapping error for TX.\n");
		return;
	}

	sport->dma_tx_desc = dmaengine_prep_slave_sg(sport->dma_tx_chan, sgl,
					sport->dma_tx_nents,
					DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);
	if (!sport->dma_tx_desc) {
		dma_unmap_sg(dev, sgl, sport->dma_tx_nents, DMA_TO_DEVICE);
		dev_err(dev, "Cannot prepare TX slave DMA!\n");
		return;
	}

	sport->dma_tx_desc->callback = lpuart_dma_tx_complete;
	sport->dma_tx_desc->callback_param = sport;
	sport->dma_tx_in_progress = true;
	sport->dma_tx_cookie = dmaengine_submit(sport->dma_tx_desc);
	dma_async_issue_pending(sport->dma_tx_chan);
}

static void lpuart_dma_tx_complete(void *arg)
{
	struct lpuart_port *sport = arg;
	struct scatterlist *sgl = &sport->tx_sgl[0];
	struct circ_buf *xmit = &sport->port.state->xmit;
	unsigned long flags;

	spin_lock_irqsave(&sport->port.lock, flags);

	dma_unmap_sg(sport->port.dev, sgl, sport->dma_tx_nents, DMA_TO_DEVICE);

	xmit->tail = (xmit->tail + sport->dma_tx_bytes) & (UART_XMIT_SIZE - 1);

	sport->port.icount.tx += sport->dma_tx_bytes;
	sport->dma_tx_in_progress = false;
	spin_unlock_irqrestore(&sport->port.lock, flags);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

	if (waitqueue_active(&sport->dma_wait)) {
		wake_up(&sport->dma_wait);
		return;
	}

	spin_lock_irqsave(&sport->port.lock, flags);

	if (!uart_circ_empty(xmit) && !uart_tx_stopped(&sport->port))
		lpuart_dma_tx(sport);
	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static int lpuart_dma_tx_request(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port,
					struct lpuart_port, port);
	struct dma_slave_config dma_tx_sconfig = {};
	int ret;

	dma_tx_sconfig.dst_addr = sport->port.mapbase + UARTDR;
	dma_tx_sconfig.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_tx_sconfig.dst_maxburst = 1;
	dma_tx_sconfig.direction = DMA_MEM_TO_DEV;
	ret = dmaengine_slave_config(sport->dma_tx_chan, &dma_tx_sconfig);

	if (ret) {
		dev_err(sport->port.dev,
				"DMA slave config failed, err = %d\n", ret);
		return ret;
	}

	return 0;
}

static void lpuart_flush_buffer(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port, struct lpuart_port, port);

	if (sport->lpuart_dma_tx_use) {
		if (sport->dma_tx_in_progress) {
			dma_unmap_sg(sport->port.dev, &sport->tx_sgl[0],
				sport->dma_tx_nents, DMA_TO_DEVICE);
			sport->dma_tx_in_progress = false;
		}
		dmaengine_terminate_all(sport->dma_tx_chan);
	}
}

#if defined(CONFIG_CONSOLE_POLL)

static int lpuart_poll_init(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port,
					struct lpuart_port, port);
	unsigned long flags;
	unsigned char temp;

	sport->port.fifosize = 0;

	spin_lock_irqsave(&sport->port.lock, flags);
	/* Disable Rx & Tx */
	writeb(0, sport->port.membase + UARTCR2);

	temp = readb(sport->port.membase + UARTPFIFO);
	/* Enable Rx and Tx FIFO */
	writeb(temp | UARTPFIFO_RXFE | UARTPFIFO_TXFE,
			sport->port.membase + UARTPFIFO);

	/* flush Tx and Rx FIFO */
	writeb(UARTCFIFO_TXFLUSH | UARTCFIFO_RXFLUSH,
			sport->port.membase + UARTCFIFO);

	/* explicitly clear RDRF */
	if (readb(sport->port.membase + UARTSR1) & UARTSR1_RDRF) {
		readb(sport->port.membase + UARTDR);
		writeb(UARTSFIFO_RXUF, sport->port.membase + UARTSFIFO);
	}

	//writeb(0, sport->port.membase + UARTTWFIFO);
	writeb(1, sport->port.membase + UARTTWFIFO);
	writeb(1, sport->port.membase + UARTRWFIFO);

	/* Enable Rx and Tx */
	writeb(UARTCR2_RE | UARTCR2_TE, sport->port.membase + UARTCR2);
	spin_unlock_irqrestore(&sport->port.lock, flags);

	return 0;
}

static void lpuart_poll_put_char(struct uart_port *port, unsigned char c)
{
	/* drain */
	while (!(readb(port->membase + UARTSR1) & UARTSR1_TDRE))
		barrier();

	writeb(c, port->membase + UARTDR);
}

static int lpuart_poll_get_char(struct uart_port *port)
{
	if (!(readb(port->membase + UARTSR1) & UARTSR1_RDRF))
		return NO_POLL_CHAR;

	return readb(port->membase + UARTDR);
}

#endif

static inline void lpuart_transmit_buffer(struct lpuart_port *sport)
{
	struct circ_buf *xmit = &sport->port.state->xmit;
	struct tty_port *port = &sport->port.state->port;

	while (!uart_circ_empty(xmit) &&
		(readb(sport->port.membase + UARTTCFIFO) < sport->txfifo_size)) 
	{
		writeb(xmit->buf[xmit->tail], sport->port.membase + UARTDR);
#ifdef TXINT_GPIO_TEST
		if(port->tty->index == 2) 
		{
		    gpio_set_value(88,0);
		}
#endif
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		sport->port.icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

	if (uart_circ_empty(xmit))
		lpuart_stop_tx(&sport->port);
}


static void lpuart_start_tx(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port,
			struct lpuart_port, port);
	struct circ_buf *xmit = &sport->port.state->xmit;
	unsigned char temp;

	temp = readb(port->membase + UARTCR2);
	writeb(temp | UARTCR2_TIE, port->membase + UARTCR2);

	if (sport->lpuart_dma_tx_use) {
		if (!uart_circ_empty(xmit) && !uart_tx_stopped(port))
			lpuart_dma_tx(sport);
	} else {
		if (readb(port->membase + UARTSR1) & UARTSR1_TDRE)
			lpuart_transmit_buffer(sport);
	}
}

int mstpTransmitComplete(struct tty_struct* tty)
{
	struct uart_state *state = tty->driver_data;
	struct uart_port *uport = state->uart_port;
	struct lpuart_port *sport = container_of(uport, struct lpuart_port, port);
	struct circ_buf *xmit = &sport->port.state->xmit;
	struct tty_port *port = &sport->port.state->port;
	unsigned char status_register1 = readb(uport->membase + UARTSR1);
	unsigned char sfifo = readb(uport->membase + UARTSFIFO);

	if(port->tty->index!=2)
		return 0;

	if((status_register1 & UARTSR1_TC)	&&
           (sfifo & UARTSFIFO_TXEMPT) 		&&
	   (uart_circ_empty(xmit)))
	{
		return 1;
	}
	return 0;
}
EXPORT_SYMBOL(mstpTransmitComplete);

/* return TIOCSER_TEMT when transmitter is not busy */
static unsigned int lpuart_tx_empty(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port,
			struct lpuart_port, port);
	unsigned char sr1 = readb(port->membase + UARTSR1);
	unsigned char sfifo = readb(port->membase + UARTSFIFO);

	if (sport->dma_tx_in_progress)
		return 0;

	if (sr1 & UARTSR1_TC && sfifo & UARTSFIFO_TXEMPT)
		return TIOCSER_TEMT;

	return 0;
}

static irqreturn_t lpuart_txint(int irq, void *dev_id)
{
	struct lpuart_port *sport = dev_id;
	struct circ_buf *xmit = &sport->port.state->xmit;
	struct tty_port *port = &sport->port.state->port;
	unsigned long flags;
	//int gpioval=0;
	//unsigned char cr2, old_cr2;

	spin_lock_irqsave(&sport->port.lock, flags);
#if 0
	if(port->tty->index == 2) 
	{
	    gpioval=gpio_get_value(88);
	    gpioval^=1;
	    gpio_set_value(88,gpioval);
	}
#endif
	if (sport->port.x_char) {		
			writeb(sport->port.x_char, sport->port.membase + UARTDR);
		goto out;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(&sport->port)) {		
			lpuart_stop_tx(&sport->port);
		goto out;
	}	

	lpuart_transmit_buffer(sport);
	if(port->tty->index == 2) 
	    lpuart_reset_silencetimer();

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

out:
	spin_unlock_irqrestore(&sport->port.lock, flags);
	return IRQ_HANDLED;
}

static irqreturn_t lpuart_rxint(int irq, void *dev_id)
{
	struct lpuart_port *sport = dev_id;
	//unsigned int flg, ignored = 0;
	unsigned int flg, /*ignored = 0,*/ overrun = 0, framing=0;
	struct tty_port *port = &sport->port.state->port;
	unsigned long flags;
	unsigned char received_char, status_register, control_register2;

	if (sport->port.irq_wake)
		pm_wakeup_event(port->tty->dev, 0);

	spin_lock_irqsave(&sport->port.lock, flags);

	while (!(readb(sport->port.membase + UARTSFIFO) & UARTSFIFO_RXEMPT)) {
		flg = TTY_NORMAL;
		sport->port.icount.rx++;
		/*
		 * to clear the FE, OR, NF, FE, PE flags,
		 * read SR1 then read DR
		 */
		status_register = readb(sport->port.membase + UARTSR1);
		received_char = readb(sport->port.membase + UARTDR);

		if(port->tty->index != 2)
		{
			if (uart_handle_sysrq_char(&sport->port, (unsigned char)received_char))
				continue;
		}

		if (status_register & (UARTSR1_PE | UARTSR1_OR | UARTSR1_FE)) {
			if (status_register & UARTSR1_PE)
				sport->port.icount.parity++;
			else if (status_register & UARTSR1_FE)
			{
				//sport->port.icount.frame++;
				framing++;
			}

			if (status_register & UARTSR1_OR)
			{
				//sport->port.icount.overrun++;
				overrun++;
			}

			//if(port->tty->index != 2)
			//{
				// if (status_register & sport->port.ignore_status_mask) {
				// 	//if the application configured to the port to ignore
				// 	//any of the 3 error conditions, then just get out,
				// 	//no need to continue, this way we reset the 
				// 	//UART as quickly as possible too.
				// 	if (++ignored > 100)
				// 		goto out;
				// 	continue;
				// }
			//}

			status_register &= sport->port.read_status_mask;

			if (status_register & UARTSR1_PE)
				flg = TTY_PARITY;
			else if (status_register & UARTSR1_FE)
				flg = TTY_FRAME;

			if (status_register & UARTSR1_OR)
				flg = TTY_OVERRUN;

#ifdef SUPPORT_SYSRQ
			sport->port.sysrq = 0;
#endif
		}

		if(port->tty->index==2)
		{
			if((!framing)&&(!overrun))
				tty_insert_flip_char(port, received_char, flg);
		}
		else	
			tty_insert_flip_char(port, received_char, flg);
		goto out;
	}

out:
	if ((overrun)||(framing)) {
		if(overrun)
			sport->port.icount.overrun += overrun;
		if(framing)
			sport->port.icount.frame += framing;
		/* Disable receiver during this operation... */
		control_register2 = readb(sport->port.membase + UARTCR2);
		control_register2 &= ~(UARTCR2_RE);
		writeb(control_register2, sport->port.membase + UARTCR2);
		/* Read DR to clear the error flags */
		readb(sport->port.membase + UARTDR);
		/*
		 * Overruns cause FIFO pointers to become missaligned.
		 * Flushing the receive FIFO reinitializes the pointers.
		 */
		writeb(UARTCFIFO_RXFLUSH, sport->port.membase + UARTCFIFO);
		writeb(UARTSFIFO_RXOF, sport->port.membase + UARTSFIFO);

		control_register2 |= UARTCR2_RE;
		writeb(control_register2, sport->port.membase + UARTCR2);
	}
	spin_unlock_irqrestore(&sport->port.lock, flags);
	/*
	 * NOTE: in interest of changing one thing at time, 
	 *	 I've restored the conditional setting of SilenceTimer here
	 * clear silence timer for our MS/TP port (ttyLP2)
	 * do this regardless of errors
	 */
	//if(port->tty->index == 2) lpuart_reset_silencetimer();
	if((!overrun)&&(!framing))
		if(port->tty->index == 2) lpuart_reset_silencetimer();
	tty_flip_buffer_push(port);
	return IRQ_HANDLED;
}

static irqreturn_t lpuart_int(int irq, void *dev_id)
{
	struct lpuart_port *sport = dev_id;
	unsigned char sts;

	sts = readb(sport->port.membase + UARTSR1);

	if (sts & UARTSR1_RDRF)
		lpuart_rxint(irq, dev_id);

	if (sts & UARTSR1_TDRE)
		lpuart_txint(irq, dev_id);

	return IRQ_HANDLED;
}

static void lpuart_copy_rx_to_tty(struct lpuart_port *sport)
{
	struct tty_port *port = &sport->port.state->port;
	struct dma_tx_state state;
	enum dma_status dmastat;
	struct circ_buf *ring = &sport->rx_ring;
	unsigned long flags;
	int count = 0;
	unsigned char sr;

	sr = readb(sport->port.membase + UARTSR1);

	if (sr & (UARTSR1_PE | UARTSR1_FE)) {
		unsigned char cr2;

		/* Disable receiver during this operation... */
		cr2 = readb(sport->port.membase + UARTCR2);
		cr2 &= ~(UARTCR2_RE);
		writeb(cr2, sport->port.membase + UARTCR2);

		/* Read DR to clear the error flags */
		readb(sport->port.membase + UARTDR);

		if (sr & UARTSR1_PE)
		    sport->port.icount.parity++;
		else if (sr & UARTSR1_FE)
		    sport->port.icount.frame++;

		/*
		 * At this point parity/framing error is cleared
		 * However, since the DMA already read the data register
		 * and we had to read it again after reading the status
		 * register to properly clear the flags, the FIFO actually
		 * underflowed... This requires a clearing of the FIFO...
		 */
		if (readb(sport->port.membase + UARTSFIFO) & UARTSFIFO_RXUF) {
			writeb(UARTSFIFO_RXUF, sport->port.membase + UARTSFIFO);
			writeb(UARTCFIFO_RXFLUSH, sport->port.membase + UARTCFIFO);
		}

		cr2 |= UARTCR2_RE;
		writeb(cr2, sport->port.membase + UARTCR2);
	}

	async_tx_ack(sport->dma_rx_desc);

	spin_lock_irqsave(&sport->port.lock, flags);

	dmastat = dmaengine_tx_status(sport->dma_rx_chan,
				sport->dma_rx_cookie,
				&state);

	if (dmastat == DMA_ERROR) {
		dev_err(sport->port.dev, "Rx DMA transfer failed!\n");
		spin_unlock_irqrestore(&sport->port.lock, flags);
		return;
	}

	/* CPU claims ownership of RX DMA buffer */
	dma_sync_sg_for_cpu(sport->port.dev, &sport->rx_sgl, 1, DMA_FROM_DEVICE);

	/*
	 * ring->head points to the end of data already written by the DMA.
	 * ring->tail points to the beginning of data to be read by the
	 * framework.
	 * The current transfer size should not be larger than the dma buffer
	 * length.
	 */
	ring->head = sport->rx_sgl.length - state.residue;
	BUG_ON(ring->head > sport->rx_sgl.length);
	/*
	 * At this point ring->head may point to the first byte right after the
	 * last byte of the dma buffer:
	 * 0 <= ring->head <= sport->rx_sgl.length
	 *
	 * However ring->tail must always points inside the dma buffer:
	 * 0 <= ring->tail <= sport->rx_sgl.length - 1
	 *
	 * Since we use a ring buffer, we have to handle the case
	 * where head is lower than tail. In such a case, we first read from
	 * tail to the end of the buffer then reset tail.
	 */
	if (ring->head < ring->tail) {
		count = sport->rx_sgl.length - ring->tail;

		tty_insert_flip_string(port, ring->buf + ring->tail, count);
		ring->tail = 0;
		sport->port.icount.rx += count;
	}

	/* Finally we read data from tail to head */
	if (ring->tail < ring->head) {
		count = ring->head - ring->tail;
		tty_insert_flip_string(port, ring->buf + ring->tail, count);
		/* Wrap ring->head if needed */
		if (ring->head >= sport->rx_sgl.length)
			ring->head = 0;
		ring->tail = ring->head;
		sport->port.icount.rx += count;
	}

	dma_sync_sg_for_device(sport->port.dev, &sport->rx_sgl, 1,
			       DMA_FROM_DEVICE);

	spin_unlock_irqrestore(&sport->port.lock, flags);

	tty_flip_buffer_push(port);
	mod_timer(&sport->lpuart_timer, jiffies + sport->dma_rx_timeout);
}

static void lpuart_dma_rx_complete(void *arg)
{
	struct lpuart_port *sport = arg;

	lpuart_copy_rx_to_tty(sport);
}

static void lpuart_timer_func(unsigned long data)
{
	struct lpuart_port *sport = (struct lpuart_port *)data;

	lpuart_copy_rx_to_tty(sport);
}

static inline int lpuart_start_rx_dma(struct lpuart_port *sport)
{
	struct dma_slave_config dma_rx_sconfig = {};
	struct circ_buf *ring = &sport->rx_ring;
	int ret, nent;
	int bits, baud;
	struct tty_struct *tty = tty_port_tty_get(&sport->port.state->port);
	struct ktermios *termios = &tty->termios;

	baud = tty_get_baud_rate(tty);

	bits = (termios->c_cflag & CSIZE) == CS7 ? 9 : 10;
	if (termios->c_cflag & PARENB)
		bits++;

	/*
	 * Calculate length of one DMA buffer size to keep latency below
	 * 10ms at any baud rate.
	 */
	sport->rx_dma_rng_buf_len = (DMA_RX_TIMEOUT * baud /  bits / 1000) * 2;
	sport->rx_dma_rng_buf_len = (1 << (fls(sport->rx_dma_rng_buf_len) - 1));
	if (sport->rx_dma_rng_buf_len < 16)
		sport->rx_dma_rng_buf_len = 16;

	ring->buf = kmalloc(sport->rx_dma_rng_buf_len, GFP_ATOMIC);
	if (!ring->buf) {
		dev_err(sport->port.dev, "Ring buf alloc failed\n");
		return -ENOMEM;
	}

	sg_init_one(&sport->rx_sgl, ring->buf, sport->rx_dma_rng_buf_len);
	sg_set_buf(&sport->rx_sgl, ring->buf, sport->rx_dma_rng_buf_len);
	nent = dma_map_sg(sport->port.dev, &sport->rx_sgl, 1, DMA_FROM_DEVICE);

	if (!nent) {
		dev_err(sport->port.dev, "DMA Rx mapping error\n");
		return -EINVAL;
	}

	dma_rx_sconfig.src_addr = sport->port.mapbase + UARTDR;
	dma_rx_sconfig.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_rx_sconfig.src_maxburst = 1;
	dma_rx_sconfig.direction = DMA_DEV_TO_MEM;
	ret = dmaengine_slave_config(sport->dma_rx_chan, &dma_rx_sconfig);

	if (ret < 0) {
		dev_err(sport->port.dev,
				"DMA Rx slave config failed, err = %d\n", ret);
		return ret;
	}

	sport->dma_rx_desc = dmaengine_prep_dma_cyclic(sport->dma_rx_chan,
				 sg_dma_address(&sport->rx_sgl),
				 sport->rx_sgl.length,
				 sport->rx_sgl.length / 2,
				 DMA_DEV_TO_MEM,
				 DMA_PREP_INTERRUPT);
	if (!sport->dma_rx_desc) {
		dev_err(sport->port.dev, "Cannot prepare cyclic DMA\n");
		return -EFAULT;
	}

	sport->dma_rx_desc->callback = lpuart_dma_rx_complete;
	sport->dma_rx_desc->callback_param = sport;
	sport->dma_rx_cookie = dmaengine_submit(sport->dma_rx_desc);
	dma_async_issue_pending(sport->dma_rx_chan);

	writeb(readb(sport->port.membase + UARTCR5) | UARTCR5_RDMAS,
				sport->port.membase + UARTCR5);

	return 0;
}

static void lpuart_dma_rx_free(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port,
					struct lpuart_port, port);

	if (sport->dma_rx_chan)
		dmaengine_terminate_all(sport->dma_rx_chan);

	dma_unmap_sg(sport->port.dev, &sport->rx_sgl, 1, DMA_FROM_DEVICE);
	kfree(sport->rx_ring.buf);
	sport->rx_ring.tail = 0;
	sport->rx_ring.head = 0;
	sport->dma_rx_desc = NULL;
	sport->dma_rx_cookie = -EINVAL;
}

static int lpuart_config_rs485(struct uart_port *port,
			struct serial_rs485 *rs485)
{
	struct lpuart_port *sport = container_of(port,
			struct lpuart_port, port);

	u8 modem = readb(sport->port.membase + UARTMODEM) &
		~(UARTMODEM_TXRTSPOL | UARTMODEM_TXRTSE);
	writeb(modem, sport->port.membase + UARTMODEM);

	if (rs485->flags & SER_RS485_ENABLED) {
		/* Enable auto RS-485 RTS mode */
		modem |= UARTMODEM_TXRTSE;

		/*
		 * RTS needs to be logic HIGH either during transer _or_ after
		 * transfer, other variants are not supported by the hardware.
		 */

		if (!(rs485->flags & (SER_RS485_RTS_ON_SEND |
				SER_RS485_RTS_AFTER_SEND)))
			rs485->flags |= SER_RS485_RTS_ON_SEND;

		if (rs485->flags & SER_RS485_RTS_ON_SEND &&
				rs485->flags & SER_RS485_RTS_AFTER_SEND)
			rs485->flags &= ~SER_RS485_RTS_AFTER_SEND;

		/*
		 * The hardware defaults to RTS logic HIGH while transfer.
		 * Switch polarity in case RTS shall be logic HIGH
		 * after transfer.
		 * Note: UART is assumed to be active high.
		 */
		if (rs485->flags & SER_RS485_RTS_ON_SEND)
			modem &= ~UARTMODEM_TXRTSPOL;
		else if (rs485->flags & SER_RS485_RTS_AFTER_SEND)
			modem |= UARTMODEM_TXRTSPOL;
	}

	/* Store the new configuration */
	sport->port.rs485 = *rs485;

	writeb(modem, sport->port.membase + UARTMODEM);
	return 0;
}

static unsigned int lpuart_get_mctrl(struct uart_port *port)
{
	unsigned int temp = 0;
	unsigned char reg;

	reg = readb(port->membase + UARTMODEM);
	if (reg & UARTMODEM_TXCTSE)
		temp |= TIOCM_CTS;

	if (reg & UARTMODEM_RXRTSE)
		temp |= TIOCM_RTS;

	return temp;
}

static void lpuart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	unsigned char temp;
	struct lpuart_port *sport = container_of(port,
				struct lpuart_port, port);

	/* Make sure RXRTSE bit is not set when RS485 is enabled */
	if (!(sport->port.rs485.flags & SER_RS485_ENABLED)) {
		temp = readb(sport->port.membase + UARTMODEM) &
			~(UARTMODEM_RXRTSE | UARTMODEM_TXCTSE);

		if (mctrl & TIOCM_RTS)
			temp |= UARTMODEM_RXRTSE;

		if (mctrl & TIOCM_CTS)
			temp |= UARTMODEM_TXCTSE;

		writeb(temp, port->membase + UARTMODEM);
	}
}

static void lpuart_break_ctl(struct uart_port *port, int break_state)
{
	unsigned char temp;

	temp = readb(port->membase + UARTCR2) & ~UARTCR2_SBK;

	if (break_state != 0)
		temp |= UARTCR2_SBK;

	writeb(temp, port->membase + UARTCR2);
}

static void lpuart_setup_watermark(struct lpuart_port *sport)
{
	unsigned char val, cr2;
	unsigned char cr2_saved;

	cr2 = readb(sport->port.membase + UARTCR2);
	cr2_saved = cr2;
	cr2 &= ~(UARTCR2_TIE | UARTCR2_TCIE | UARTCR2_TE |
			UARTCR2_RIE | UARTCR2_RE);
	writeb(cr2, sport->port.membase + UARTCR2);

	/* enabled TX and RX FIFO */
	val = readb(sport->port.membase + UARTPFIFO);
	writeb(val | UARTPFIFO_TXFE | UARTPFIFO_RXFE,
			sport->port.membase + UARTPFIFO);

	/* flush Tx and Rx FIFO */
	writeb(UARTCFIFO_TXFLUSH | UARTCFIFO_RXFLUSH,
			sport->port.membase + UARTCFIFO);

	/* explicitly clear RDRF */
	if (readb(sport->port.membase + UARTSR1) & UARTSR1_RDRF) {
		readb(sport->port.membase + UARTDR);
		writeb(UARTSFIFO_RXUF, sport->port.membase + UARTSFIFO);
	}

	//writeb(0, sport->port.membase + UARTTWFIFO);	/* not sure how 0 affects the watermark */
	writeb(1, sport->port.membase + UARTTWFIFO);	/* this means the TX irq should occur every 1 byte */
	writeb(1, sport->port.membase + UARTRWFIFO);

	/* Restore cr2 */
	writeb(cr2_saved, sport->port.membase + UARTCR2);
}

static void rx_dma_timer_init(struct lpuart_port *sport)
{
		setup_timer(&sport->lpuart_timer, lpuart_timer_func,
				(unsigned long)sport);
		sport->lpuart_timer.expires = jiffies + sport->dma_rx_timeout;
		add_timer(&sport->lpuart_timer);
}

static int lpuart_startup(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port, struct lpuart_port, port);
	struct tty_port *ttyport = &sport->port.state->port;
	int ret;
	unsigned long flags;
	unsigned char temp;

	/* determine FIFO size and enable FIFO mode */
	temp = readb(sport->port.membase + UARTPFIFO);

	sport->txfifo_size = 0x1 << (((temp >> UARTPFIFO_TXSIZE_OFF) &
		UARTPFIFO_FIFOSIZE_MASK) + 1);

	sport->port.fifosize = sport->txfifo_size;

	sport->rxfifo_size = 0x1 << (((temp >> UARTPFIFO_RXSIZE_OFF) &
		UARTPFIFO_FIFOSIZE_MASK) + 1);

	if(ttyport->tty->index == 2) 
	{
		printk("FSL_LPUART: TXFIFO size is %d\n",sport->txfifo_size);
		printk("FSL_LPUART: RXFIFO size is %d\n",sport->rxfifo_size);
	}

	ret = devm_request_irq(port->dev, port->irq, lpuart_int, 0,
				DRIVER_NAME, sport);
	if (ret)
		return ret;

	spin_lock_irqsave(&sport->port.lock, flags);

	lpuart_setup_watermark(sport);

	temp = readb(sport->port.membase + UARTCR2);
	temp |= (UARTCR2_RIE | UARTCR2_TIE | UARTCR2_RE | UARTCR2_TE);
	writeb(temp, sport->port.membase + UARTCR2);

	if (sport->dma_rx_chan && !lpuart_start_rx_dma(sport)) {
		/* set Rx DMA timeout */
		sport->dma_rx_timeout = msecs_to_jiffies(DMA_RX_TIMEOUT);
		if (!sport->dma_rx_timeout)
		     sport->dma_rx_timeout = 1;

		sport->lpuart_dma_rx_use = true;
		rx_dma_timer_init(sport);
	} else {
		sport->lpuart_dma_rx_use = false;
	}

	if (sport->dma_tx_chan && !lpuart_dma_tx_request(port)) {
		init_waitqueue_head(&sport->dma_wait);
		sport->lpuart_dma_tx_use = true;
		temp = readb(port->membase + UARTCR5);
		writeb(temp | UARTCR5_TDMAS, port->membase + UARTCR5);
	} else {
		sport->lpuart_dma_tx_use = false;
	}

	spin_unlock_irqrestore(&sport->port.lock, flags);

	return 0;
}

static void lpuart_shutdown(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port, struct lpuart_port, port);
	unsigned char temp;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	/* disable Rx/Tx and interrupts */
	temp = readb(port->membase + UARTCR2);
	temp &= ~(UARTCR2_TE | UARTCR2_RE |
			UARTCR2_TIE | UARTCR2_TCIE | UARTCR2_RIE);
	writeb(temp, port->membase + UARTCR2);

	spin_unlock_irqrestore(&port->lock, flags);

	devm_free_irq(port->dev, port->irq, sport);

	if (sport->lpuart_dma_rx_use) {
		del_timer_sync(&sport->lpuart_timer);
		lpuart_dma_rx_free(&sport->port);
	}

	if (sport->lpuart_dma_tx_use) {
		if (wait_event_interruptible(sport->dma_wait,
			!sport->dma_tx_in_progress) != false) {
			sport->dma_tx_in_progress = false;
			dmaengine_terminate_all(sport->dma_tx_chan);
		}

		lpuart_stop_tx(port);
	}
}

static void
lpuart_set_termios(struct uart_port *port, struct ktermios *termios,
		   struct ktermios *old)
{
	struct lpuart_port *sport = container_of(port, struct lpuart_port, port);
	unsigned long flags;
	unsigned char cr1, old_cr1, old_cr2, cr3, cr4, bdh, modem;
	unsigned int  baud;
	unsigned int old_csize = old ? old->c_cflag & CSIZE : CS8;
	unsigned int sbr, brfa;

	cr1 = old_cr1 = readb(sport->port.membase + UARTCR1);
	old_cr2 = readb(sport->port.membase + UARTCR2);
	cr3 = readb(sport->port.membase + UARTCR3);
	cr4 = readb(sport->port.membase + UARTCR4);
	bdh = readb(sport->port.membase + UARTBDH);
	modem = readb(sport->port.membase + UARTMODEM);
	/*
	 * only support CS8 and CS7, and for CS7 must enable PE.
	 * supported mode:
	 *  - (7,e/o,1)
	 *  - (8,n,1)
	 *  - (8,m/s,1)
	 *  - (8,e/o,1)
	 */
	while ((termios->c_cflag & CSIZE) != CS8 &&
		(termios->c_cflag & CSIZE) != CS7) {
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= old_csize;
		old_csize = CS8;
	}

	if ((termios->c_cflag & CSIZE) == CS8 ||
		(termios->c_cflag & CSIZE) == CS7)
		cr1 = old_cr1 & ~UARTCR1_M;

	if (termios->c_cflag & CMSPAR) {
		if ((termios->c_cflag & CSIZE) != CS8) {
			termios->c_cflag &= ~CSIZE;
			termios->c_cflag |= CS8;
		}
		cr1 |= UARTCR1_M;
	}

	/*
	 * When auto RS-485 RTS mode is enabled,
	 * hardware flow control need to be disabled.
	 */
	if (sport->port.rs485.flags & SER_RS485_ENABLED)
		termios->c_cflag &= ~CRTSCTS;

	if (termios->c_cflag & CRTSCTS) {
		modem |= (UARTMODEM_RXRTSE | UARTMODEM_TXCTSE);
	} else {
		termios->c_cflag &= ~CRTSCTS;
		modem &= ~(UARTMODEM_RXRTSE | UARTMODEM_TXCTSE);
	}

	if (termios->c_cflag & CSTOPB)
		termios->c_cflag &= ~CSTOPB;

	/* parity must be enabled when CS7 to match 8-bits format */
	if ((termios->c_cflag & CSIZE) == CS7)
		termios->c_cflag |= PARENB;

	if ((termios->c_cflag & PARENB)) {
		if (termios->c_cflag & CMSPAR) {
			cr1 &= ~UARTCR1_PE;
			if (termios->c_cflag & PARODD)
				cr3 |= UARTCR3_T8;
			else
				cr3 &= ~UARTCR3_T8;
		} else {
			cr1 |= UARTCR1_PE;
			if ((termios->c_cflag & CSIZE) == CS8)
				cr1 |= UARTCR1_M;
			if (termios->c_cflag & PARODD)
				cr1 |= UARTCR1_PT;
			else
				cr1 &= ~UARTCR1_PT;
		}
	}

	/* ask the core to calculate the divisor */
	baud = uart_get_baud_rate(port, termios, old, 50, port->uartclk / 16);

	/*
	 * Need to update the Ring buffer length according to the selected
	 * baud rate and restart Rx DMA path.
	 *
	 * Since timer function acqures sport->port.lock, need to stop before
	 * acquring same lock because otherwise del_timer_sync() can deadlock.
	 */
	if (old && sport->lpuart_dma_rx_use) {
		del_timer_sync(&sport->lpuart_timer);
		lpuart_dma_rx_free(&sport->port);
	}

	spin_lock_irqsave(&sport->port.lock, flags);

	sport->port.read_status_mask = 0;
	if (termios->c_iflag & INPCK)
		sport->port.read_status_mask |=	(UARTSR1_FE | UARTSR1_PE);
	if (termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		sport->port.read_status_mask |= UARTSR1_FE;

	/* characters to ignore */
	sport->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		sport->port.ignore_status_mask |= UARTSR1_PE;
	if (termios->c_iflag & IGNBRK) {
		sport->port.ignore_status_mask |= UARTSR1_FE;
		/*
		 * if we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			sport->port.ignore_status_mask |= UARTSR1_OR;
	}

	/* update the per-port timeout */
	uart_update_timeout(port, termios->c_cflag, baud);

	/* wait transmit engin complete */
	while (!(readb(sport->port.membase + UARTSR1) & UARTSR1_TC))
		barrier();

	/* disable transmit and receive */
	writeb(old_cr2 & ~(UARTCR2_TE | UARTCR2_RE),
			sport->port.membase + UARTCR2);

	sbr = sport->port.uartclk / (16 * baud);
	brfa = ((sport->port.uartclk - (16 * sbr * baud)) * 2) / baud;
	bdh &= ~UARTBDH_SBR_MASK;
	bdh |= (sbr >> 8) & 0x1F;
	cr4 &= ~UARTCR4_BRFA_MASK;
	brfa &= UARTCR4_BRFA_MASK;
	writeb(cr4 | brfa, sport->port.membase + UARTCR4);
	writeb(bdh, sport->port.membase + UARTBDH);
	writeb(sbr & 0xFF, sport->port.membase + UARTBDL);
	writeb(cr3, sport->port.membase + UARTCR3);
	writeb(cr1, sport->port.membase + UARTCR1);
	writeb(modem, sport->port.membase + UARTMODEM);

	/* restore control register */
	writeb(old_cr2, sport->port.membase + UARTCR2);

	if (old && sport->lpuart_dma_rx_use) {
		if (!lpuart_start_rx_dma(sport))
			rx_dma_timer_init(sport);
		else
			sport->lpuart_dma_rx_use = false;
	}

	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static const char *lpuart_type(struct uart_port *port)
{
	return "FSL_LPUART";
}

static void lpuart_release_port(struct uart_port *port)
{
	/* nothing to do */
}

static int lpuart_request_port(struct uart_port *port)
{
	return  0;
}

/* configure/autoconfigure the port */
static void lpuart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_LPUART;
}

static int lpuart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;

	if (ser->type != PORT_UNKNOWN && ser->type != PORT_LPUART)
		ret = -EINVAL;
	if (port->irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != UPIO_MEM)
		ret = -EINVAL;
	if (port->uartclk / 16 != ser->baud_base)
		ret = -EINVAL;
	if (port->iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}

static const struct uart_ops lpuart_pops = {
	.tx_empty	= lpuart_tx_empty,
	.set_mctrl	= lpuart_set_mctrl,
	.get_mctrl	= lpuart_get_mctrl,
	.stop_tx	= lpuart_stop_tx,
	.start_tx	= lpuart_start_tx,
	.stop_rx	= lpuart_stop_rx,
	.break_ctl	= lpuart_break_ctl,
	.startup	= lpuart_startup,
	.shutdown	= lpuart_shutdown,
	.set_termios	= lpuart_set_termios,
	.type		= lpuart_type,
	.request_port	= lpuart_request_port,
	.release_port	= lpuart_release_port,
	.config_port	= lpuart_config_port,
	.verify_port	= lpuart_verify_port,
	.flush_buffer	= lpuart_flush_buffer,
#if defined(CONFIG_CONSOLE_POLL)
	.poll_init	= lpuart_poll_init,
	.poll_get_char	= lpuart_poll_get_char,
	.poll_put_char	= lpuart_poll_put_char,
#endif
};

static struct lpuart_port *lpuart_ports[UART_NR];

#ifdef CONFIG_SERIAL_FSL_LPUART_CONSOLE
static void lpuart_console_putchar(struct uart_port *port, int ch)
{
	while (!(readb(port->membase + UARTSR1) & UARTSR1_TDRE))
		barrier();

	writeb(ch, port->membase + UARTDR);
}

static void
lpuart_console_write(struct console *co, const char *s, unsigned int count)
{
	struct lpuart_port *sport = lpuart_ports[co->index];
	unsigned char  old_cr2, cr2;
	unsigned long flags;
	int locked = 1;

	if (sport->port.sysrq || oops_in_progress)
		locked = spin_trylock_irqsave(&sport->port.lock, flags);
	else
		spin_lock_irqsave(&sport->port.lock, flags);

	/* first save CR2 and then disable interrupts */
	cr2 = old_cr2 = readb(sport->port.membase + UARTCR2);
	cr2 |= (UARTCR2_TE |  UARTCR2_RE);
	cr2 &= ~(UARTCR2_TIE | UARTCR2_TCIE | UARTCR2_RIE);
	writeb(cr2, sport->port.membase + UARTCR2);

	uart_console_write(&sport->port, s, count, lpuart_console_putchar);

	/* wait for transmitter finish complete and restore CR2 */
	while (!(readb(sport->port.membase + UARTSR1) & UARTSR1_TC))
		barrier();

	writeb(old_cr2, sport->port.membase + UARTCR2);

	if (locked)
		spin_unlock_irqrestore(&sport->port.lock, flags);
}


/*
 * if the port was already initialised (eg, by a boot loader),
 * try to determine the current setup.
 */
static void __init
lpuart_console_get_options(struct lpuart_port *sport, int *baud,
			   int *parity, int *bits)
{
	unsigned char cr, bdh, bdl, brfa;
	unsigned int sbr, uartclk, baud_raw;

	cr = readb(sport->port.membase + UARTCR2);
	cr &= UARTCR2_TE | UARTCR2_RE;
	if (!cr)
		return;

	/* ok, the port was enabled */

	cr = readb(sport->port.membase + UARTCR1);

	*parity = 'n';
	if (cr & UARTCR1_PE) {
		if (cr & UARTCR1_PT)
			*parity = 'o';
		else
			*parity = 'e';
	}

	if (cr & UARTCR1_M)
		*bits = 9;
	else
		*bits = 8;

	bdh = readb(sport->port.membase + UARTBDH);
	bdh &= UARTBDH_SBR_MASK;
	bdl = readb(sport->port.membase + UARTBDL);
	sbr = bdh;
	sbr <<= 8;
	sbr |= bdl;
	brfa = readb(sport->port.membase + UARTCR4);
	brfa &= UARTCR4_BRFA_MASK;

	uartclk = clk_get_rate(sport->clk);
	/*
	 * baud = mod_clk/(16*(sbr[13]+(brfa)/32)
	 */
	baud_raw = uartclk / (16 * (sbr + brfa / 32));

	if (*baud != baud_raw)
		printk(KERN_INFO "Serial: Console lpuart rounded baud rate"
				"from %d to %d\n", baud_raw, *baud);
}

static int __init lpuart_console_setup(struct console *co, char *options)
{
	struct lpuart_port *sport;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index == -1 || co->index >= ARRAY_SIZE(lpuart_ports))
		co->index = 0;

	sport = lpuart_ports[co->index];
	if (sport == NULL)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		lpuart_console_get_options(sport, &baud, &parity, &bits);
	
	lpuart_setup_watermark(sport);

	return uart_set_options(&sport->port, co, baud, parity, bits, flow);
}

static struct uart_driver lpuart_reg;
static struct console lpuart_console = {
	.name		= DEV_NAME,
	.write		= lpuart_console_write,
	.device		= uart_console_device,
	.setup		= lpuart_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &lpuart_reg,
};

static void lpuart_early_write(struct console *con, const char *s, unsigned n)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, n, lpuart_console_putchar);
}

static int __init lpuart_early_console_setup(struct earlycon_device *device,
					  const char *opt)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = lpuart_early_write;
	return 0;
}


OF_EARLYCON_DECLARE(lpuart, "fsl,vf610-lpuart", lpuart_early_console_setup);
EARLYCON_DECLARE(lpuart, lpuart_early_console_setup);

#define LPUART_CONSOLE	(&lpuart_console)
#else
#define LPUART_CONSOLE	NULL
#endif

static struct uart_driver lpuart_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= DRIVER_NAME,
	.dev_name	= DEV_NAME,
	.nr		= ARRAY_SIZE(lpuart_ports),
	.cons		= LPUART_CONSOLE,
};

static struct dma_chan *lpuart_request_dma_chan(struct lpuart_port *sport,
						const char *name)
{
	struct dma_chan *chan;

	chan = dma_request_slave_channel(sport->port.dev, name);
	if (!chan)
		dev_info(sport->port.dev, "DMA %s channel request failed, "
				"operating without %s DMA\n", name, name);
	return chan;
}

static int lpuart_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct lpuart_port *sport;
	struct resource *res;
	int ret;

	gpio_request(88,"tx int debug");
	gpio_direction_output(88,1);
	gpio_set_value(88,0);

	sport = devm_kzalloc(&pdev->dev, sizeof(*sport), GFP_KERNEL);
	if (!sport)
		return -ENOMEM;

	pdev->dev.coherent_dma_mask = 0;

	ret = of_alias_get_id(np, "serial");
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get alias id, errno %d\n", ret);
		return ret;
	}
	sport->port.line = ret;
	//sport->lpuart32 = of_device_is_compatible(np, "fsl,ls1021a-lpuart");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sport->port.membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(sport->port.membase))
		return PTR_ERR(sport->port.membase);

	sport->port.mapbase = res->start;
	sport->port.dev = &pdev->dev;
	sport->port.type = PORT_LPUART;
	sport->port.iotype = UPIO_MEM;
	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot obtain irq\n");
		return ret;
	}
	sport->port.irq = ret;
	
	sport->port.ops = &lpuart_pops;
	sport->port.flags = UPF_BOOT_AUTOCONF;

	sport->port.rs485_config = lpuart_config_rs485;

	sport->clk = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(sport->clk)) {
		ret = PTR_ERR(sport->clk);
		dev_err(&pdev->dev, "failed to get uart clk: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(sport->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable uart clk: %d\n", ret);
		return ret;
	}

	sport->port.uartclk = clk_get_rate(sport->clk);

	lpuart_ports[sport->port.line] = sport;

	platform_set_drvdata(pdev, &sport->port);
	
	lpuart_reg.cons = LPUART_CONSOLE;

	ret = uart_add_one_port(&lpuart_reg, &sport->port);
	if (ret) {
		clk_disable_unprepare(sport->clk);
		return ret;
	}

	if (!nodma) {
		sport->dma_tx_chan = lpuart_request_dma_chan(sport, "tx");
		sport->dma_rx_chan = lpuart_request_dma_chan(sport, "rx");
	}

	if (of_property_read_bool(np, "linux,rs485-enabled-at-boot-time")) {
		sport->port.rs485.flags |= SER_RS485_ENABLED;
		/* Disable RTS at startup	*/
		sport->port.rs485.flags &= ~SER_RS485_RTS_ON_SEND;
                sport->port.rs485.flags |= SER_RS485_RTS_AFTER_SEND;
                writeb(UARTMODEM_TXRTSE | UARTMODEM_TXRTSPOL, sport->port.membase + UARTMODEM);
	}

	return 0;
}

static int lpuart_remove(struct platform_device *pdev)
{
	struct lpuart_port *sport = platform_get_drvdata(pdev);

	uart_remove_one_port(&lpuart_reg, &sport->port);

	clk_disable_unprepare(sport->clk);

	if (sport->dma_tx_chan)
		dma_release_channel(sport->dma_tx_chan);

	if (sport->dma_rx_chan)
		dma_release_channel(sport->dma_rx_chan);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int lpuart_suspend(struct device *dev)
{
	struct lpuart_port *sport = dev_get_drvdata(dev);
	unsigned long temp;

	/* disable Rx/Tx and interrupts */
	temp = readb(sport->port.membase + UARTCR2);
	temp &= ~(UARTCR2_TE | UARTCR2_TIE | UARTCR2_TCIE);
	writeb(temp, sport->port.membase + UARTCR2);
	
	uart_suspend_port(&lpuart_reg, &sport->port);

	if (sport->lpuart_dma_rx_use) {
		/*
		 * EDMA driver during suspend will forcefully release any
		 * non-idle DMA channels. If port wakeup is enabled or if port
		 * is console port or 'no_console_suspend' is set the Rx DMA
		 * cannot resume as as expected, hence gracefully release the
		 * Rx DMA path before suspend and start Rx DMA path on resume.
		 */
		if (sport->port.irq_wake) {
			del_timer_sync(&sport->lpuart_timer);
			lpuart_dma_rx_free(&sport->port);
		}

		/* Disable Rx DMA to use UART port as wakeup source */
		writeb(readb(sport->port.membase + UARTCR5) & ~UARTCR5_RDMAS,
					sport->port.membase + UARTCR5);
	}

	if (sport->lpuart_dma_tx_use) {
		sport->dma_tx_in_progress = false;
		dmaengine_terminate_all(sport->dma_tx_chan);
	}

	if (sport->port.suspended && !sport->port.irq_wake)
		clk_disable_unprepare(sport->clk);

	return 0;
}

static int lpuart_resume(struct device *dev)
{
	struct lpuart_port *sport = dev_get_drvdata(dev);
	unsigned long temp;

	if (sport->port.suspended && !sport->port.irq_wake)
		clk_prepare_enable(sport->clk);

	lpuart_setup_watermark(sport);
	temp = readb(sport->port.membase + UARTCR2);
	temp |= (UARTCR2_RIE | UARTCR2_TIE | UARTCR2_RE | UARTCR2_TE);
	writeb(temp, sport->port.membase + UARTCR2);
	
	if (sport->lpuart_dma_rx_use) {
		if (sport->port.irq_wake) {
			if (!lpuart_start_rx_dma(sport))
				rx_dma_timer_init(sport);
			else
				sport->lpuart_dma_rx_use = false;
		}
	}

	if (sport->dma_tx_chan && !lpuart_dma_tx_request(&sport->port)) {
			init_waitqueue_head(&sport->dma_wait);
			sport->lpuart_dma_tx_use = true;
			writeb(readb(sport->port.membase + UARTCR5) |
				UARTCR5_TDMAS, sport->port.membase + UARTCR5);
	} else {
		sport->lpuart_dma_tx_use = false;
	}

	uart_resume_port(&lpuart_reg, &sport->port);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(lpuart_pm_ops, lpuart_suspend, lpuart_resume);

static struct platform_driver lpuart_driver = {
	.probe		= lpuart_probe,
	.remove		= lpuart_remove,
	.driver		= {
		.name	= "fsl-lpuart",
		.of_match_table = lpuart_dt_ids,
		.pm	= &lpuart_pm_ops,
	},
};

static int __init lpuart_serial_init(void)
{
	int ret = uart_register_driver(&lpuart_reg);

	if (ret)
		return ret;

	ret = platform_driver_register(&lpuart_driver);
	if (ret)
		uart_unregister_driver(&lpuart_reg);

	return ret;
}

static void __exit lpuart_serial_exit(void)
{
	platform_driver_unregister(&lpuart_driver);
	uart_unregister_driver(&lpuart_reg);
}

module_init(lpuart_serial_init);
module_exit(lpuart_serial_exit);

MODULE_DESCRIPTION("Freescale lpuart serial port driver");
MODULE_LICENSE("GPL v2");
