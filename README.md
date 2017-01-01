LPC810-based I2C UART
=====================

This is a very basic I2C slave peripheral intended to provide UART
capabilities in places where only I2C is available.

At the moment, it's a work in progress.

Pin configuration
-----------------

* 1: `nRESET`: Used to reset the I2C UART peripheral, may also be used to
  initiate in-circuit firmware update over the UART by toggling this with
  `nINTERRUPT`/`nISPEntry` held low.
* 2: `TxD`: Transmit data from the UART
* 3: `SDA`: I2C Data
* 4: `SCL`: I2C Clock
* 5: `nINTERRUPT`/`nISPEntry`: In normal operation, this is the hardware
  interrupt line, used by the peripheral to signal status changes to the
  master.  During the reset phase, holding this line LOW puts the
  peripheral into firmware update mode, in which case `lpc21isp` can be
  used via the `TxD` and `RxD` pins to replace the firmware.
* 6: 3v3 supply
* 7: 0v supply
* 8: `RxD`: Receive data to the UART

I2C interface information
-------------------------

The serial interface presents a block of 256 registers, 128 of which
correspond to a FIFO buffer which may be read from or written to, the
other are configuration and status registers.

The I2C slave hardware in the LPC810 takes care of the mechanics of the I2C
interface, presenting the state of the interface in a single register;
`I2C_STAT` (at address 0x40050004).  The `SLVPENDING` and `SLVSTATE` bits
give the state of the I2C interface, which may be one of the following:

* `SLVPENDING`=0, `SLVSTATE`=don't care: Interface is idle.  It waits for a
  START bit followed by one of the four addresses assigned.
* `SLVPENDING`=1, `SLVSTATE`=0: The master has summoned us.  The LPC810 can
  match up to 4 addresses, the `SLVIDX` bits tell us which one.  We respond
  with `ACK` if address index 0 is matched.  We expect to go into
  `SLVSTATE`=1 or 2.
* `SLVPENDING`=1, `SLVSTATE`=1: The master wishes to tell us something.
* `SLVPENDING`=1, `SLVSTATE`=2: The master wishes us to tell it something.

Upon entering `SLVSTATE`=0, we reset the `i2c_rx_reg_addr` flag then send
an `ACK`.  The I2C hardware then moves to either `SLVSTATE`=1 or
`SLVSTATE`=2.

In the former case, the master is doing a write, the first byte of which
will be the register address where we are to begin writing.  We store the
byte received as `i2c_reg_addr`, clear `i2c_rx_reg_addr`, then send
an `ACK`.

If we remain in `SLVSTATE`=1, subsequent bytes are written to the
addresses specified, and are `ACK`ed, unless the address is read-only, in
which case, a `NACK` is emitted.  Each `ACK`ed byte also increments
`i2c_reg_addr`, which wraps at address 256.

When the master wishes to read a register, typically we'll see a write
first, to choose a starting register, then before submitting data, it'll
re-send the start sequence and request a read, causing the I2C slave
hardware to enter `SLVSTATE`=2.  Upon receipt of this re-start, we clear
`i2c_rx_reg_addr` and send each byte requested.

Register addressing structure
-----------------------------

The register structure is loosely based on what was seen in the TTY
interface of the Linux kernel and the registers available in the LPC810.

Identification registers:

* Address 0x00…0x0d:
  The raw string, "VK4MSL I2CUART", used for debugging with i2cread, etc.
* Address 0x0e: Firmware version major number (currently 0)
* Address 0x0f: Firmware version minor number (currently 0)

* Address 0x14: Frame configuration register
  - Bits 7…5: Number of data bits
    - 15…9: Reserved
    - 8: 8 bits
    - 7: 7 bits
    - 5…0: Reserved
  - Bit 4: Number of stop bits
    - 1: 2 bits
    - 0: 1 bit
  - Bits 3…2: Parity settings
    - 3: Even parity
    - 2: Odd parity
    - 1: Reserved (do not use)
    - 0: No parity
  - Bits 1…0: Reserved.
* Address 0x15: Master Interrupt Acknowledge register.  This acknowledges
  bits that have been set in the status register.
* Address 0x16: Master Interrupt Enable register.  This enables select
  interrupt flags.  If the corresponding bits in the Status register are
  set *AND* `nINTERRUPT` is enabled (bit 5, master control register), then
  the `nINTERRUPT` line will be held low until all bits are cleared.
* Address 0x17: Master control register
  - Bit 7: Apply currently configured settings, attempts to set the UART
    interface according to the settings given.
  - Bit 6: Revert currently configured settings, rolls back the
    configuration to one consistent with the UARTs current settings.
  - Bit 5: Enable/Disable `nINTERRUPT`.  If enabled, the `nINTERRUPT`
    pin will be held logic high until an interrupt condition occurrs,
    in which case the pin will be pulled low.
    If disabled, the pin floats, and should be pulled high by an
    external resistor.
  - Bit 4: Transmitter Enable/Disable.  If disabled, data is held in the
    transmit buffers until the transmitter is re-enabled.
  - Bits 3…0: Reserved.
* Address: 0x18: Master Status Register
  - Bit 7: Configurtion error.  An attempt to apply settings has failed
    due to an invalid combination of settings.
  - Bit 6: Frame Error.  An invalid UART frame was received.
  - Bit 5: Break detected.  A BREAK signal was detected on the UART.
  - Bit 4…2: Reserved.
  - Bit 1: Transmit interrupt raised, one or more bits are set in `TX_STAT`.
  - Bit 0: Receive interrupt raised, one or more bits are set in `RX_STAT`.
* Address 0x19…0x22: Reserved (reads as 0xff)
* Address 0x23: Receive buffer minimum fill level register.
* Address 0x24: Receive buffer maximum fill level register.
* Address 0x25: Receive interrupt acknowledge register.
* Address 0x26: Receive interrupt enable register.
* Address 0x27: Receive buffer control register:
  - Bit 7: Flush receive buffer
  - Bits 6…0: Reserved (write only zeroes)
* Address 0x28: Receive buffer status register:
  - Bit 7: Receive buffer overflow.  A received byte has been dropped
    because the buffer is full.
  - Bit 6: Receive buffer full.  The receive buffer is now full and will
    begin dropping incoming data unless emptied.
  - Bit 5: Receive buffer maximum bytes waiting.  The maximum fill level
    has been reached.
  - Bit 4: Receive buffer minimum bytes waiting.  The minimum fill level
    has been reached.
  - Bit 3: Receive buffer is empty.
* Address 0x29: Receive buffer bytes waiting.
* Address 0x2a: Receive buffer bytes free.
* Address 0x2b…0x32: Reserved
* Address 0x33: Transmit buffer minimum fill level register.
* Address 0x34: Transmit buffer maximum fill level register.
* Address 0x35: Transmit interrupt acknowledge register.
* Address 0x36: Transmit interrupt enable register.
* Address 0x37: Transmit buffer control register:
  - Bit 7: Flush transmit buffer
  - Bits 6…0: Reserved (write only zeroes)
* Address 0x38: Transmit buffer status register:
  - Bit 7: Transmit buffer overflow.  A transmitted byte has been dropped
    because the buffer is full.
  - Bit 6: Transmit buffer full.  The transmit buffer is now full and will
    begin dropping incoming data unless emptied.
  - Bit 5: Transmit buffer maximum bytes waiting.  The maximum fill level
    has been reached.
  - Bit 4: Transmit buffer minimum bytes waiting.  The minimum fill level
    has been reached.
  - Bit 3: Transmit buffer is empty.
* Address 0x39: Transmit buffer bytes waiting.
* Address 0x3a: Transmit buffer bytes free.
* Address 0x80…0xff: FIFO buffer

FIFO read/write operation
-------------------------

The FIFO buffer permits up to 128 bytes to be read or written from the
buffer at any one time.

Each time a byte is read from this address range, it is removed from the
receive FIFO, and each byte written to this address range is pushed into
the transmit FIFO.

The I2C register address is incremented each time, and will eventually
roll around back to address 0, causing the identification string and
firmware version information to be read back out or for the written bytes
to be NACKed.

Thus, it is recommended that each read or write of the FIFO starts at
address 0x80.
