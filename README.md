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
* Address 0x10…0x7f: Reserved (reads as 0xff)
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
