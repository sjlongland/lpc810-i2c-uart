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

/* ----- Interrupt helper macros ----- */

#define ENABLE_NINTERRUPT(en) \
	LPC_GPIO_PORT->DIR0 = ((en) << 1)
#define ASSERT_NINTERRUPT \
	LPC_GPIO_PORT->CLR0 = (1 << 1)
#define DISASSERT_NINTERRUPT \
	LPC_GPIO_PORT->SET0 = (1 << 1)

/* ----- I2C identification data ----- */

/* Registers 0…13 */
const static char i2c_ident[] = "VK4MSL I2CUART";

/* Registers 14 and 15 */
#define I2C_VERSION_MAJOR	(0)
#define I2C_VERSION_MINOR	(0)

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

#define I2C_ACK 					\
	LPC_I2C->SLVCTL &= ~(1 << 1)

#define I2C_NEXT 					\
	LPC_I2C->SLVCTL |= (1 << 0)

/* ----- Core application ----- */

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
	ENABLE_NINTERRUPT(1);

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
	}
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
			I2C_ACK;
		}

		I2C_NEXT;
		return;
	} else if (state == SLVSTATE_S2M) {
		/* Read the next byte off the register bus */
		uint8_t byte = 0;
		i2c_rx_reg_addr = 0;
		if (!(i2c_reg_addr & 0xf0)) {
			switch(i2c_reg_addr) {
				case 0x0e:
					byte = I2C_VERSION_MAJOR;
					break;
				case 0x0f:
					byte = I2C_VERSION_MINOR;
					break;
				default:
					byte = i2c_ident[
						i2c_reg_addr & 0x0f
					];
			}
		} else if (!(i2c_reg_addr & 0x80)) {
			/* TODO */
			byte = 0xff;
		} else {
			/* Read from FIFO: TODO */
			byte = 0xff;
		}
		LPC_I2C->SLVDAT = byte;
	} else if (state == SLVSTATE_M2S) {
		if (i2c_rx_reg_addr) {
			/* This is a register address */
			i2c_reg_addr = LPC_I2C->SLVDAT;
			i2c_rx_reg_addr = 0;
			/* Do not increment, ACK and return */
			I2C_ACK;
			I2C_NEXT;
			return;
		} else {
			/* Write the incoming byte to the given address */
			if (!(i2c_reg_addr & 0xf0)) {
				/* Read only addresses */
				I2C_NAK;
				goto done;
			} else if (!(i2c_reg_addr & 0x80)) {
				/* TODO */
				I2C_NAK;
				goto done;
			}
		}
	}
done:
	/* Increment the pointer */
	i2c_reg_addr++;
	I2C_NEXT;
}
