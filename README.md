# OLED_I2C_w_interrupts
Experiments with a TWI ( I2C ) interrupt handler\
\
The two altered header files are part of the OLED_I2C library by Rinky Dink Electronics and the ino file is an altered example that uses interrupts to avoid waiting on I2C transfers to complete.\
\
In the zip is another version, based upon the Nokia library LCD_Basic.  The SPI has been converted to I2C.  This version does not use a frame buffer and is useful for small memory processors like the UNO,Nano.
\
The major part of the contents here is copyrighted by Rinky-Dink Electronics.\
\
The original idea was to have a process that empties a circular buffer of transactions, but it requires polling when the buffer fills.  The combination of interrupts and polling sometimes confuses the processing of the interrupt flag.
OL06_Basic_Interrupt.ino - This version processes one transaction at a time, works better without hang ups, but waits if a process is active and a second one is queued.  A double buffered to be written version could buffer transactions in a circular buffer, and process one transaction in a separate buffer. ( a transaction being a sequence of start, writes and stop )


