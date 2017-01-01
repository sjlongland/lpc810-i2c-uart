/*!
 * I2C-driven UART firmware
 * (C) Stuart Longland <me@vk4msl.id.au>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301
 * USA
 */

#include "LPC8xx.h"

/*!
 * Baud rate divisor settings.  This maps the baud rate to the register
 * settings needed for the MCUs serial port baud rate settings.
 */
struct baud_t {
	uint32_t baudrate;
	uint8_t uartclkdiv;
	uint8_t uartfrgdiv;
	uint8_t uartfrgmult;
	uint8_t brgval;
};

/*!
 * Baud rate look-up table, for 12MHz internal RC oscillator.
 */
const static struct baud_t standard_rates[] = {
	{
		.baudrate	= 110,
		.uartclkdiv	= 238,
		.uartfrgdiv	= 255,
		.uartfrgmult	= 226,
		.brgval		= 242
	},
	{
		.baudrate	= 134,
		.uartclkdiv	= 242,
		.uartfrgdiv	= 255,
		.uartfrgmult	= 224,
		.brgval		= 196
	},
	{
		.baudrate	= 150,
		.uartclkdiv	= 250,
		.uartfrgdiv	= 255,
		.uartfrgmult	= 225,
		.brgval		= 169
	},
	{
		.baudrate	= 200,
		.uartclkdiv	= 250,
		.uartfrgdiv	= 255,
		.uartfrgmult	= 195,
		.brgval		= 135
	},
	{
		.baudrate	= 300,
		.uartclkdiv	= 250,
		.uartfrgdiv	= 255,
		.uartfrgmult	= 225,
		.brgval		= 84
	},
	{
		.baudrate	= 600,
		.uartclkdiv	= 250,
		.uartfrgdiv	= 255,
		.uartfrgmult	= 153,
		.brgval		= 49
	},
	{
		.baudrate	= 1200,
		.uartclkdiv	= 250,
		.uartfrgdiv	= 255,
		.uartfrgmult	= 153,
		.brgval		= 24
	},
	{
		.baudrate	= 2400,
		.uartclkdiv	= 250,
		.uartfrgdiv	= 255,
		.uartfrgmult	= 85,
		.brgval		= 14
	},
	{
		.baudrate	= 4800,
		.uartclkdiv	= 250,
		.uartfrgdiv	= 0,
		.uartfrgmult	= 0,
		.brgval		= 9
	},
	{
		.baudrate	= 9600,
		.uartclkdiv	= 250,
		.uartfrgdiv	= 0,
		.uartfrgmult	= 0,
		.brgval		= 4
	},
	{
		.baudrate	= 19200,
		.uartclkdiv	= 125,
		.uartfrgdiv	= 0,
		.uartfrgmult	= 0,
		.brgval		= 4
	},
	{
		.baudrate	= 38400,
		.uartclkdiv	= 101,
		.uartfrgdiv	= 255,
		.uartfrgmult	= 8,
		.brgval		= 2
	},
	{
		.baudrate	= 57600,
		.uartclkdiv	= 25,
		.uartfrgdiv	= 255,
		.uartfrgmult	= 170,
		.brgval		= 4
	},
	{
		.baudrate	= 115200,
		.uartclkdiv	= 101,
		.uartfrgdiv	= 255,
		.uartfrgmult	= 8,
		.brgval		= 0
	},
	{
		.baudrate	= 230400,
		.uartclkdiv	= 29,
		.uartfrgdiv	= 255,
		.uartfrgmult	= 203,
		.brgval		= 0
	},
	{
		.baudrate	= 460800,
		.uartclkdiv	= 4,
		.uartfrgdiv	= 255,
		.uartfrgmult	= 77,
		.brgval		= 4
	},
	{
		.baudrate	= 921600,
		.uartclkdiv	= 2,
		.uartfrgdiv	= 255,
		.uartfrgmult	= 77,
		.brgval		= 4
	},
	/* End of list */
	{
		.baudrate	= 0,
		.uartclkdiv	= 0,
		.uartfrgdiv	= 0,
		.uartfrgmult	= 0,
		.brgval		= 0,
	}
};

/* ----- Interrupt helper macros ----- */

#define ENABLE_NINTERRUPT(en) \
	LPC_GPIO_PORT->DIR0 = ((en) << 1)
#define ASSERT_NINTERRUPT \
	LPC_GPIO_PORT->CLR0 = (1 << 1)
#define DISASSERT_NINTERRUPT \
	LPC_GPIO_PORT->SET0 = (1 << 1)

/* ----- Bit twiddling helpers ----- */

#define GET_BITS(reg, offset, mask) \
	((reg >> (offset)) & (mask))

#define SET_BITS(reg, offset, mask, value) \
	reg = ((reg & ((mask) << (offset))) | ((value) << (offset)))

/* ----- I2C identification data ----- */

/*! I2C firmware identification string, registers 0…13 */
const static char reg_ident[] = "VK4MSL I2CUART";
/*! I2C firmware identification string register start address */
#define REG_IDENT_ADDR		(0x00)

/*! Major firmware version number */
#define REG_VERSION_MAJOR	(0)
/*! Address of major firmware version number */
#define REG_VERSION_MAJOR_ADDR	(0x0e)
/*! Minor firmware version number */
#define REG_VERSION_MINOR	(0)
/*! Address of minor firmware version number */
#define REG_VERSION_MINOR_ADDR	(0x0f)

/* ----- I2C slave state ----- */

/*!
 * Await new register address.  If set, then the next written byte is to
 * be interpreted as a new register address, we should store this in
 * `i2c_reg_addr` below and `ACK` the byte.
 */
uint8_t i2c_rx_reg_addr = 0;

/*!
 * The last requested address.  This is incremented with each data byte
 * read or written, and gets overwritten by the first byte received after
 * an ID+W byte.
 */
uint8_t i2c_reg_addr	= 0;

#define SLVSTATE_ADDR	(0)	/*!< Address received */
#define SLVSTATE_M2S	(1)	/*!< Master to Slave transfer */
#define SLVSTATE_S2M	(2)	/*!< Slave to Master transfer */

/* ----- I2C helpers ----- */

#define I2C_NAK 					\
	LPC_I2C->SLVCTL |= (1 << 1)

#define I2C_NEXT 					\
	LPC_I2C->SLVCTL |= (1 << 0)

/* ----- FIFO definitions ----- */

/*! Size of each FIFO buffer */
#define FIFO_SZ		(128)

/*!
 * FIFO data structure
 */
struct fifo_t {
	/*! FIFO data */
	volatile uint8_t data[FIFO_SZ];
	/*! Read pointer */
	volatile uint8_t read_ptr;
	/*! Write pointer */
	volatile uint8_t write_ptr;
};

/*! Transmit FIFO */
static struct fifo_t tx_fifo;

/*! Receive FIFO */
static struct fifo_t rx_fifo;

/*!
 * Reset the FIFO
 */
void fifo_reset(struct fifo_t* const fifo) {
	fifo->read_ptr = 0;
	fifo->write_ptr = 0;
}

/*!
 * Enqueue a byte to the FIFO.
 * @param	fifo	FIFO to write to
 * @param	byte	Byte to enqueue
 * @returns	Number of bytes stored
 */
uint8_t fifo_enqueue(struct fifo_t* const fifo, uint8_t byte) {
	uint8_t next = fifo->write_ptr + 1;
	if (next == fifo->read_ptr)
		return 0;	/* FIFO is full */
	fifo->data[fifo->write_ptr] = byte;
	fifo->write_ptr = next;
	return 1;
}

/*!
 * Dequeue a byte from the FIFO.
 * @param	fifo	FIFO to read from
 * @returns	Byte read
 * @retval	-1	if empty
 */
int16_t fifo_dequeue(struct fifo_t* const fifo) {
	if (fifo->write_ptr == fifo->read_ptr)
		return -1;	/* FIFO is empty */
	return fifo->data[fifo->read_ptr++];
}

/*!
 * Return the number of bytes in the FIFO.
 */
uint8_t fifo_size(struct fifo_t* const fifo) {
	return ((fifo->write_ptr - fifo->read_ptr) % FIFO_SZ) + 1;
}

/* ----- Core software registers ----- */

/*!
 * Master Status Register.  This indicates the status of the various flags
 * for the UART peripheral.
 */
volatile uint8_t reg_msr = 0;

/*!
 * Bad configuration supplied.  This bit is set when applying an invalid
 * configuration, and is cleared by writing a 1 to the corresponding
 * MST_ACK bit.
 */
#define REG_MSR_CFG_ERR		(1 << 7)

/*!
 * Frame error detected.  A badly formatted frame was encountered (e.g.
 * parity error, low stop bit, etc.)  Reset by writing this bit to the
 * MST_ACK register.
 */
#define REG_MSR_FRAME_ERR	(1 << 6)

/*!
 * Break detected.  A "break" signal was detected on the UART.
 * Reset by writing this bit to the ACK register.
 */
#define REG_MSR_BREAK		(1 << 5)

/*!
 * Transmit interrupt triggered.  See the TX_* registers.
 */
#define REG_MSR_TX_INT		(1 << 1)

/*!
 * Receive interrupt triggered.  See the RX_* registers.
 */
#define REG_MSR_RX_INT		(1 << 0)

/*! Master Status Register address */
#define REG_MSR_ADDR		(0x18)

/*!
 * Master control register.  This is used to put the UART transceiver
 * offline, controls the global enabling of interrupts and applies pending
 * configuration settings.
 */
volatile uint8_t reg_mcr = 0;
/*! Address of major control register */
#define REG_MCR_ADDR		(0x17)

/*! Apply currently defined settings */
#define REG_MCR_APPLY		(1 << 7)

/*! Revert to currently active settings */
#define REG_MCR_REVERT		(1 << 6)

/*!
 * Enable/Disable interrupts.  When disabled, nINTERRUPT is floating
 * and may be pulled low by the master before pulsing nRESET to initiate
 * firmware replacement.
 */
#define REG_MCR_IRQ_EN		(1 << 5)

/*!
 * Enable/Disable transmitter.  When disabled, transmit data is held in
 * the FIFO until the transmitter is enabled or the FIFO is flushed.  This
 * can be used to implement handling of hardware or software flow control.
 */
#define REG_MCR_TX_EN		(1 << 4)

/*!
 * Master Status Interrupt Acknowledge register address.
 * Setting ones here clears corresponding bits in the status register.
 */
#define REG_MASTER_ACK_ADDR	(0x15)

/*!
 * Master Interrupt Enable register
 */
volatile uint8_t reg_master_int = 0;
/*!
 * Master Interrupt Enable register address.
 */
#define REG_MASTER_INT_ADDR	(0x16)

/*!
 * UART frame configuration register:
 * - Bits 7…5: Frame size in bits:
 *   15…9: Reserved
 *   8: 8-bits
 *   7: 7-bits
 *   5…0: Reserved
 * - Bit 4: Number of stop bits:
 *   1: 2 bits
 *   0: 1 bit
 * - Bit 3…2: Parity mode
 *   3: Even
 *   2: Odd
 *   1: Reserved
 *   0: None
 * - Bits 1…0: Reserved
 */
uint8_t reg_frame = 0;
#define REG_FRAME_SZ_OFFSET	(5)		/*!< Frame size bit# */
#define REG_FRAME_SZ_MASK	(7)		/*!< Frame size mask */
#define REG_FRAME_SZ_8BIT	(8)		/*!< Frame size = 8 bits */
#define REG_FRAME_SZ_7BIT	(7)		/*!< Frame size = 7 bits */
#define REG_FRAME_STOP_OFFSET	(4)		/*!< Stop bit count bit# */
#define REG_FRAME_STOP_MASK	(1)		/*!< Stop bit count mask */
#define REG_FRAME_STOP_2BIT	(1)		/*!< 2 stop bits */
#define REG_FRAME_STOP_1BIT	(1)		/*!< 1 stop bit */
#define REG_FRAME_PARITY_OFFSET	(2)		/*!< Frame parity bit# */
#define REG_FRAME_PARITY_MASK	(3)		/*!< Frame parity mask */
#define REG_FRAME_PARITY_EVEN	(3)		/*!< Even parity */
#define REG_FRAME_PARITY_ODD	(2)		/*!< Odd parity */
#define REG_FRAME_PARITY_NONE	(0)		/*!< No parity */

/*! Address of UART frame configuration register */
#define REG_FRAME_ADDR		(0x14)

/*! UART baud rate register.  Given in big-endian format. */
uint32_t reg_baud = 0;

/*! Configured baud rate, for easy reverting */
uint32_t cfg_baud = 0;

/*! Address of baud rate register */
#define REG_BAUD_ADDR		(0x10)

/*
 * Receive buffer minimum fill level.  This is the minimum level that will
 * trigger an interrupt indicating data is waiting.  Triggers the MIN_WAIT
 * interrupt.
 */
uint8_t reg_rx_min = 1;
/*! Address of Receive buffer minimum fill level register. */
#define REG_RX_MIN_ADDR		(0x23)

/*
 * Receive buffer maximum fill level.  This is the maximum level that will
 * trigger an interrupt indicating the buffer is nearly full.  Triggers the
 * MAX_WAIT interrupt.
 */
uint8_t reg_rx_max = FIFO_SZ;
/*! Address of Receive buffer minimum fill level register. */
#define REG_RX_MAX_ADDR		(0x24)

/*! Address of Receive interrupt acknowledge register */
#define REG_RX_ACK_ADDR		(0x25)

/*! Receive status interrupt enable register */
uint8_t reg_rx_int = 0;
/*! Address of Receive interrupt enable register */
#define REG_RX_INT_ADDR		(0x26)

/*! Address of Receive buffer control register */
#define REG_RX_CTRL_ADDR	(0x27)

/*! Flush the receive buffer */
#define REG_RX_CTRL_FLUSH	(1 << 7)

/*! Receive status interrupt enable register */
uint8_t reg_rx_stat = 0;
/*! Address of Receive status register */
#define REG_RX_STAT_ADDR	(0x28)

/*! Address of receive data waiting register */
#define REG_RX_WAIT_ADDR	(0x29)

/*! Address of receive data buffer free register */
#define REG_RX_FREE_ADDR	(0x2a)

/*!
 * Received bytes have been dropped as the FIFO is full.  Clear by
 * writing this bit to REG_RX_ACK.
 */
#define REG_RX_INT_OVER		(1 << 7)
/*!
 * The FIFO is now full and further incoming traffic will generate
 * RX_INT_OVER.  Cleared by:
 * - writing this bit to RX_ACK
 * - reading bytes from the FIFO
 * - emptying the FIFO.
 */
#define REG_RX_INT_FULL		(1 << 6)
/*!
 * The FIFO has at least RX_MAX bytes stored for reading.  Cleared by:
 * - writing this bit to RX_ACK
 * - reading bytes from the FIFO
 * - emptying the FIFO.
 */
#define REG_RX_INT_MAX_WAIT	(1 << 5)
/*!
 * The FIFO has at least RX_MIN bytes stored for reading.  Cleared by:
 * - writing this bit to RX_ACK
 * - reading bytes from the FIFO
 * - emptying the FIFO.
 */
#define REG_RX_INT_MIN_WAIT	(1 << 4)
/*!
 * The FIFO is completely empty.  Cleared by writing this bit to RX_ACK.
 */
#define REG_RX_INT_EMPTY	(1 << 3)

/*
 * Transmit buffer minimum fill level.  This is the minimum level that will
 * trigger an interrupt indicating data is waiting.  Triggers the MIN_WAIT
 * interrupt.
 */
uint8_t reg_tx_min = 1;
/*! Address of Transmit buffer minimum fill level register. */
#define REG_TX_MIN_ADDR		(0x33)

/*
 * Transmit buffer maximum fill level.  This is the maximum level that will
 * trigger an interrupt indicating the buffer is nearly full.  Triggers the
 * MAX_WAIT interrupt.
 */
uint8_t reg_tx_max = FIFO_SZ;
/*! Address of Transmit buffer minimum fill level register. */
#define REG_TX_MAX_ADDR		(0x34)

/*! Address of Transmit interrupt acknowledge register */
#define REG_TX_ACK_ADDR		(0x35)

/*! Transmit status interrupt enable register */
uint8_t reg_tx_int = 0;
/*! Address of Transmit interrupt enable register */
#define REG_TX_INT_ADDR		(0x36)

/*! Transmit status interrupt enable register */
uint8_t reg_tx_ctrl = 0;
/*! Address of Transmit interrupt enable register */
#define REG_TX_CTRL_ADDR	(0x37)

/*! Flush the transmit buffer */
#define REG_TX_CTRL_FLUSH	(1 << 7)

/*! Transmit status interrupt enable register */
uint8_t reg_tx_stat = 0;
/*! Address of Transmit status register */
#define REG_TX_STAT_ADDR	(0x38)

/*! Address of transmit data waiting register */
#define REG_TX_WAIT_ADDR	(0x39)

/*! Address of transmit data buffer free register */
#define REG_TX_FREE_ADDR	(0x3a)

/*!
 * Transmitted bytes have been dropped as the FIFO is full.  Clear by
 * writing this bit to REG_TX_ACK.
 */
#define REG_TX_INT_OVER		(1 << 7)
/*!
 * The FIFO is now full and further incoming traffic will generate
 * TX_INT_OVER.  Cleared by:
 * - writing this bit to TX_ACK
 * - emptying the FIFO.
 */
#define REG_TX_INT_FULL		(1 << 6)
/*!
 * The FIFO has at least TX_MAX bytes stored for writing.  Cleared by:
 * - writing this bit to TX_ACK
 * - emptying the FIFO.
 */
#define REG_TX_INT_MAX_WAIT	(1 << 5)
/*!
 * The FIFO has at least TX_MIN bytes stored for writing.  Cleared by:
 * - writing this bit to TX_ACK
 * - emptying the FIFO.
 */
#define REG_TX_INT_MIN_WAIT	(1 << 4)
/*!
 * The FIFO is completely empty.  Cleared by writing this bit to TX_ACK.
 */
#define REG_TX_INT_EMPTY	(1 << 3)

/* ----- Core application ----- */

void apply(void);
void revert(void);

int main(void) {
	/* ----- Clock setup ----- */

	LPC_SYSCON->SYSAHBCLKCTRL |= ( /* UM10601 4.6.13 */
		(1 <<  5) |	/* I2C */
		(1 <<  6) |	/* GPIO */
		(1 <<  7) |	/* Switch matrix */
		(1 << 14)	/* UART0 */
	);
	LPC_SYSCON->PRESETCTRL = (
		(1 <<  0) |	/* assert SPI0 reset */
		(1 <<  1) |	/* assert SPI1 reset */
		(0 <<  2) |	/* disassert UARTFRG reset */
		(0 <<  3) |	/* disassert UART0 reset */
		(1 <<  4) |	/* assert UART1 reset */
		(1 <<  5) |	/* assert UART2 reset */
		(0 <<  6) |	/* disassert I2C reset */
		(1 <<  7) |	/* assert MRT reset */
		(1 <<  8) |	/* assert SCT reset */
		(1 <<  9) |	/* assert WKT reset */
		(0 << 10) |	/* disassert GPIO reset */
		(1 << 11) |	/* assert FLASH reset */
		(1 << 12)	/* assert ACMP reset */
	);

	/* ----- Switch matrix setup ----- */

	/* Assign pins */
	LPC_SWM->PINASSIGN0 = (	/* UM10601 9.5.1 */
		/* U0_CTS_I = No Connect */
		(0xfful << 24) |
		/* U0_RTS_O = No Connect */
		(0xfful << 16) |
		/* U0_RXD_I = GPIO 0.0, Pin 8 */
		(0x00ul << 8) |
		/* U0_TXD_O = GPIO 0.4, Pin 2 */
		(0x04ul << 0)
	);
	LPC_SWM->PINASSIGN1 = ( /* UM10601 9.5.2 */
		/* All: no connect */
		0xfffffffful
	);
	LPC_SWM->PINASSIGN2 = ( /* UM10601 9.5.3 */
		/* All: no connect */
		0xfffffffful
	);
	LPC_SWM->PINASSIGN3 = (	/* UM10601 9.5.4 */
		/* All: no connect */
		0xfffffffful
	);
	LPC_SWM->PINASSIGN4 = ( /* UM10601 9.5.5 */
		/* All: no connect */
		0xfffffffful
	);
	LPC_SWM->PINASSIGN5 = ( /* UM10601 9.5.6 */
		/* All: no connect */
		0xfffffffful
	);
	LPC_SWM->PINASSIGN6 = ( /* UM10601 9.5.7 */
		/* All: no connect */
		0xfffffffful
	);
	LPC_SWM->PINASSIGN7 = ( /* UM10601 9.5.8 */
		/* I2C_SDA_IO: GPIO 0.3, Pin 3 */
		(0x03ul << 24) |
		/* CTOUT_3_O: No connect */
		(0xfful << 16) |
		/* CTOUT_2_0: No connect */
		(0xfful << 8) |
		/* CTOUT_1_0: No connect */
		(0xfful << 0)
	);
	LPC_SWM->PINASSIGN8 = ( /* UM10601 9.5.9 */
		/* GPIO_INT_BMAT_O: no connect */
		(0xfful << 24) |
		/* CLKOUT_O: no connect */
		(0xfful << 16) |
		/* ACMP_O_O: no connect */
		(0xfful << 8) |
		/* I2C_SCL_IO: GPIO 0.2, Pin 4 */
		(0x02ul << 0)
	);
	LPC_SWM->PINENABLE0 = ( /* UM10601 9.5.10 */
		(0 << 0) |	/* ACMP_I1_EN */
		(0 << 1) |	/* ACMP_I2_EN */
		(0 << 2) |	/* SWDCLK_EN */
		(0 << 3) |	/* SWDIO_EN */
		(0 << 4) |	/* XTALIN_EN */
		(0 << 5) |	/* XTALOUT_EN */
		(1 << 6) |	/* RESET_EN */
		(0 << 7) |	/* CLKIN */
		(0 << 8)	/* VDDCMP */
	);

	/*
	 * At this point, we should have the following pin assignments:
	 * 1: nRESET	(power on default)
	 * 2: TxD
	 * 3: SDA
	 * 4: SCL
	 * 5: GPIO	(will be nINTERRUPT, doubles as nISPEntry)
	 * 6: 3v3
	 * 7: 0v
	 * 8: RxD
	 */

	/* ----- GPIO setup ----- */

	/*
	 * GPIO 0_1 (aka pin 5) will be an output.  All others will be
	 * inputs.  We do this with a macro since we'll make provision
	 * for it to be turned off with a I2C register later.
	 */
	DISASSERT_NINTERRUPT;
	ENABLE_NINTERRUPT(0);

	/* ----- I2C setup ----- */

	/* Slave address will be 0x55 (0xaa = read, 0xab = write) */

	LPC_I2C->SLVADR0 = 0xaa;	/* Read address given here */
	LPC_I2C->CFG = (1 << 1);	/* Enable slave */
	LPC_I2C->INTENSET = (
		(1 << 8)		/* Enable Slave Pending */
	);

	/* ----- NVIC setup ----- */

	NVIC->ISER[0] = ( /* UM10601 3.4.1 */
			(1 << 3) |	/* Enable UART0 interrupts */
			(1 << 8)	/* Enable I2C interrupts */
	);

	/* ----- Main loop ----- */

	while(1) {
		/* Check control commands */
		if (reg_mcr & REG_MCR_APPLY) {
			apply();
			reg_mcr &= ~REG_MCR_APPLY;
		}
		if (reg_mcr & REG_MCR_REVERT) {
			revert();
			reg_mcr &= ~REG_MCR_REVERT;
		}

		/* Check interrupt flags */
		if (reg_rx_stat)
			reg_msr |= REG_MSR_RX_INT;
		else
			reg_msr &= ~REG_MSR_RX_INT;

		if (reg_tx_stat)
			reg_msr |= REG_MSR_TX_INT;
		else
			reg_msr &= ~REG_MSR_TX_INT;

		/* Assert or disassert interrupt pin */
		if (reg_msr && (reg_mcr & REG_MCR_IRQ_EN))
			ASSERT_NINTERRUPT;
		else
			DISASSERT_NINTERRUPT;
	}
}


/*!
 * Apply the configuration specified in the registers.
 */
void apply(void) {
	/* Figure out a valid baud rate */
	const struct baud_t* baud = standard_rates;
	uint8_t data_sz, stop_sz, parity;
	uint32_t cfg;

	while ((baud->baudrate) && (baud->baudrate != reg_baud))
		baud++;

	if (!baud->baudrate) {
		/* Baud rate not valid */
		goto invalid;
	}

	/* Sanity check the other settings */
	data_sz = (reg_frame >> REG_FRAME_SZ_OFFSET) & REG_FRAME_SZ_MASK;
	stop_sz = (reg_frame >> REG_FRAME_STOP_OFFSET)
		& REG_FRAME_STOP_MASK;
	parity = (reg_frame >> REG_FRAME_PARITY_OFFSET)
		& REG_FRAME_PARITY_MASK;

	/* Initial bits */
	cfg =	(1 << 0);	/* Enable the USART */

	/* UM10601 section 15.6.1 bits 3:2 */
	switch (data_sz) {
		case REG_FRAME_SZ_7BIT:
			cfg |= (0x00 << 2);
			break;
		case REG_FRAME_SZ_8BIT:
			cfg |= (0x01 << 2);
			break;
		default:
			goto invalid;
	}

	/* We basically copy the others straight from NXP's datasheet */
	switch (parity) {
		case REG_FRAME_PARITY_NONE:
		case REG_FRAME_PARITY_ODD:
		case REG_FRAME_PARITY_EVEN:
			cfg |= (parity << 4);
			break;
		default:
			goto invalid;
	}

	if (stop_sz)
		cfg |= (1 << 6);

	/* All good, apply the settings */
	LPC_SYSCON->UARTCLKDIV = baud->uartclkdiv;
	LPC_SYSCON->UARTFRGDIV = baud->uartfrgdiv;
	LPC_SYSCON->UARTFRGMULT = baud->uartfrgmult;
	LPC_USART0->CFG = cfg;
	LPC_USART0->BRG = baud->brgval;
	cfg_baud = reg_baud;
	reg_msr &= ~REG_MSR_CFG_ERR;
	return;
invalid:
	reg_msr |= REG_MSR_CFG_ERR;
}

/*!
 * Revert the configuration settings to what is presently set.
 */
void revert(void) {
	uint8_t frame = 0;

	switch ((LPC_USART0->CFG >> 2) & 0x03) {
		case 0:
			frame |= (REG_FRAME_SZ_7BIT) << REG_FRAME_SZ_OFFSET;
			break;
		case 1:
			frame |= (REG_FRAME_SZ_8BIT) << REG_FRAME_SZ_OFFSET;
			break;
		default:
			break;
	}

	frame |= ((LPC_USART0->CFG >> 4) & REG_FRAME_PARITY_MASK)
			<< REG_FRAME_PARITY_OFFSET;
	if (LPC_USART0->CFG & (1 << 6))
		frame |= (REG_FRAME_STOP_2BIT << REG_FRAME_STOP_OFFSET);

	reg_baud = cfg_baud;
	reg_frame = frame;
}


void I2C_IRQHandler(void) {
	/* I2C has summoned us!  What is it this time? */
	uint8_t state = (LPC_I2C->STAT >> 9) & 0x03;
	if (state == SLVSTATE_ADDR) {
		/*
		 * This is just the address + r/w, make sure it's us
		 * then just ACK it, or NAK it if it isn't.
		 */
		uint8_t idx = (LPC_I2C->STAT >> 12) & 0x03;
		if (idx) {
			/* We're not using that index */
			I2C_NAK;
		} else {
			/* If writing, expect a register address */
			i2c_rx_reg_addr = 1;
		}

		goto done_noinc;
	} else if (state == SLVSTATE_S2M) {
		/* Read the next byte off the register bus */
		uint8_t byte = 0;
		i2c_rx_reg_addr = 0;
		if (!(i2c_reg_addr & 0xf0)) {
			/* Address 0x00…0x0f */
			switch(i2c_reg_addr) {
				case REG_VERSION_MAJOR_ADDR:
					byte = REG_VERSION_MAJOR;
					break;
				case REG_VERSION_MINOR_ADDR:
					byte = REG_VERSION_MINOR;
					break;
				default:
					byte = reg_ident[
						i2c_reg_addr & 0x0f
					];
			}
		} else if (!(i2c_reg_addr & 0xe0)) {
			/* Address 0x10…0x1f */
			switch(i2c_reg_addr) {
				case REG_MCR_ADDR:
					byte = reg_mcr;
					break;
				case REG_MSR_ADDR:
					byte = reg_msr;
					break;
				case REG_MASTER_ACK_ADDR:
					byte = reg_msr;
					break;
				case REG_MASTER_INT_ADDR:
					byte = reg_master_int;
					break;
				case REG_FRAME_ADDR:
					byte = reg_frame;
					break;
				case REG_BAUD_ADDR:
					byte = GET_BITS(reg_baud,
							24, 0xff);
					break;
				case REG_BAUD_ADDR+1:
					byte = GET_BITS(reg_baud,
							16, 0xff);
					break;
				case REG_BAUD_ADDR+2:
					byte = GET_BITS(reg_baud,
							8, 0xff);
					break;
				case REG_BAUD_ADDR+3:
					byte = GET_BITS(reg_baud,
							0, 0xff);
					break;
				default:
					byte = 0xff;
			}
		} else if ((i2c_reg_addr & 0xf0) == 0x20) {
			/* Address 0x20…0x2f */
			switch(i2c_reg_addr) {
				case REG_RX_MIN_ADDR:
					byte = reg_rx_min;
					break;
				case REG_RX_MAX_ADDR:
					byte = reg_rx_max;
					break;
				case REG_RX_ACK_ADDR:
					byte = reg_rx_stat;
					break;
				case REG_RX_STAT_ADDR:
					byte = reg_rx_stat;
					break;
				case REG_RX_INT_ADDR:
					byte = reg_rx_int;
					break;
				case REG_RX_WAIT_ADDR:
					byte = fifo_size(&rx_fifo);
					break;
				case REG_RX_FREE_ADDR:
					byte = FIFO_SZ
						- fifo_size(&rx_fifo);
					break;
				default:
					byte = 0xff;
			}
		} else if ((i2c_reg_addr & 0xf0) == 0x30) {
			/* Address 0x30…0x3f */
			switch(i2c_reg_addr) {
				case REG_TX_MIN_ADDR:
					byte = reg_tx_min;
					break;
				case REG_TX_MAX_ADDR:
					byte = reg_tx_max;
					break;
				case REG_TX_ACK_ADDR:
					byte = reg_tx_stat;
					break;
				case REG_TX_STAT_ADDR:
					byte = reg_tx_stat;
					break;
				case REG_TX_INT_ADDR:
					byte = reg_tx_int;
					break;
				case REG_TX_WAIT_ADDR:
					byte = fifo_size(&tx_fifo);
					break;
				case REG_TX_FREE_ADDR:
					byte = FIFO_SZ
						- fifo_size(&tx_fifo);
					break;
				default:
					byte = 0xff;
			}
		} else if (!(i2c_reg_addr & 0x80)) {
			/* TODO */
			byte = 0xff;
		} else {
			/* Read from FIFO */
			int16_t b = fifo_dequeue(&rx_fifo);
			if (b < 0)
				byte = 0xff;
			else
				byte = b & 0xff;
		}
		LPC_I2C->SLVDAT = byte;
	} else if (state == SLVSTATE_M2S) {
		if (i2c_rx_reg_addr) {
			/* This is a register address */
			i2c_reg_addr = LPC_I2C->SLVDAT;
			i2c_rx_reg_addr = 0;
			/* Do not increment */
			goto done_noinc;
		} else {
			/* Write the incoming byte to the given address */
			if (!(i2c_reg_addr & 0xf0)) {
				/* Address 0x00…0x0f: Read only */
				I2C_NAK;
				goto done;
			} else if (!(i2c_reg_addr & 0xe0)) {
				/* Address 0x10…0x1f */
				switch(i2c_reg_addr) {
					case REG_MCR_ADDR:
						reg_mcr =
							LPC_I2C->SLVDAT;
						break;
					case REG_MASTER_ACK_ADDR:
						reg_msr &=
							~LPC_I2C->SLVDAT;
						break;
					case REG_MASTER_INT_ADDR:
						reg_master_int =
							LPC_I2C->SLVDAT;
						break;
					case REG_FRAME_ADDR:
						reg_frame =
							LPC_I2C->SLVDAT;
						break;
					case REG_BAUD_ADDR:
						SET_BITS(reg_baud,
							24, 0xff,
							LPC_I2C->SLVDAT);
						break;
					case REG_BAUD_ADDR+1:
						SET_BITS(reg_baud,
							16, 0xff,
							LPC_I2C->SLVDAT);
						break;
					case REG_BAUD_ADDR+2:
						SET_BITS(reg_baud,
							8, 0xff,
							LPC_I2C->SLVDAT);
						break;
					case REG_BAUD_ADDR+3:
						SET_BITS(reg_baud,
							0, 0xff,
							LPC_I2C->SLVDAT);
						break;
					default:
						I2C_NAK;
						goto done;
				}
			} else if ((i2c_reg_addr & 0xf0) == 0x20) {
				/* Address 0x20…0x2f */
				switch(i2c_reg_addr) {
					case REG_RX_MIN_ADDR:
						reg_rx_min =
							LPC_I2C->SLVDAT;
						break;
					case REG_RX_MAX_ADDR:
						reg_rx_max =
							LPC_I2C->SLVDAT;
						break;
					case REG_RX_ACK_ADDR:
						reg_rx_stat &=
							~LPC_I2C->SLVDAT;
						break;
					case REG_RX_INT_ADDR:
						reg_rx_int =
							LPC_I2C->SLVDAT;
						break;
					default:
						I2C_NAK;
						goto done;
				}
			} else if ((i2c_reg_addr & 0xf0) == 0x30) {
				/* Address 0x30…0x3f */
				switch(i2c_reg_addr) {
					case REG_TX_MIN_ADDR:
						reg_tx_min =
							LPC_I2C->SLVDAT;
						break;
					case REG_TX_MAX_ADDR:
						reg_tx_max =
							LPC_I2C->SLVDAT;
						break;
					case REG_TX_ACK_ADDR:
						reg_tx_stat &=
							~LPC_I2C->SLVDAT;
						break;
					case REG_TX_INT_ADDR:
						reg_tx_int;
							LPC_I2C->SLVDAT;
						break;
					default:
						I2C_NAK;
						goto done;
				}
			} else if (!(i2c_reg_addr & 0x80)) {
				/* TODO */
				if (!fifo_enqueue(&tx_fifo,
						LPC_I2C->SLVDAT))
					I2C_NAK;
				goto done;
			}
		}
	}
done:
	/* Increment the pointer */
	i2c_reg_addr++;
done_noinc:
	I2C_NEXT;
}
