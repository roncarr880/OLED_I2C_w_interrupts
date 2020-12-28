
// rwc
extern void i2init(void);
extern void i2start( unsigned char adr );
extern void i2send( unsigned int data );
extern void i2stop(void);
//extern void i2flush(void);
//extern void i2queue_read( unsigned char adr, unsigned char reg, unsigned char qty );
//extern int  i2read_int(void);
//extern uint8_t i2available(void);
//extern uint8_t i2poll(void);

void OLED::_convert_float(char *buf, double num, int width, byte prec)
{
	dtostrf(num, width, prec, buf);
}

void OLED::_initTWI()
{
   i2init();
}

void OLED::update(){
    for( uint8_t i = 0; i < 8; ++i ) update_row(i);
}

// buffers about 128 i2c actions and returns without waiting on done bits ( wait for 8 with 128 size buffer )
void OLED::update_row(uint8_t r){ 
int b;
uint8_t count;


         r &= 7;
         b = (int)r * 128;

	_sendTWIcommand(SSD1306_SET_COLUMN_ADDR);
	_sendTWIcommand(0);
	_sendTWIcommand(127);

	_sendTWIcommand(SSD1306_SET_PAGE_ADDR);
	_sendTWIcommand(r);
	_sendTWIcommand(7);
	
         i2start( SSD1306_ADDR );
         i2send(SSD1306_DATA_CONTINUE);

         for( count = 0; count < 128; ++count ) i2send( scrbuf[b++] );
    
         i2stop();
}

void OLED::_sendTWIcommand(uint8_t value){

    i2start(SSD1306_ADDR);
    i2send(SSD1306_COMMAND);
    i2send(value);
    i2stop();
}

/*   old code

        #ifdef NOWAY
	for (uint16_t b=0; b<_bufsize; b++)		// Send data
		if (_use_hw)
		{
			TWDR = scrbuf[b];
			TWCR = _BV(TWEN) | _BV(TWINT) | _BV(TWEA);									// Clear TWINT to proceed
			while ((TWCR & _BV(TWINT)) == 0) {};										// Wait for TWI to be ready
		}
		else
		{
			_writeByte(scrbuf[b]);
			_waitForAck();
		}
        #endif
#ifdef NOWAY
	// activate internal pullups for twi.
	digitalWrite(SDA, HIGH);
	digitalWrite(SCL, HIGH);
	//delay(1);  // Workaround for a linker bug

	// initialize twi prescaler and bit rate
	__cbi2(TWSR, TWPS0);
	__cbi2(TWSR, TWPS1);
	TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;

	// enable twi module, acks, and twi interrupt
	TWCR = _BV(TWEN) | _BV(TWIE)/* | _BV(TWEA)* /;
#endif

	if (_use_hw)					// Send TWI Start
	{
		// Send start address
	//	TWCR = _BV(TWEN) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
	//	while ((TWCR & _BV(TWINT)) == 0) {};
	//	TWDR = SSD1306_ADDR<<1;
	//	TWCR = _BV(TWEN) | _BV(TWINT) | _BV(TWEA);
	//	while ((TWCR & _BV(TWINT)) == 0) {};
	//	TWDR = SSD1306_DATA_CONTINUE;
	//	TWCR = _BV(TWEN) | _BV(TWINT) | _BV(TWEA);
	//	while ((TWCR & _BV(TWINT)) == 0) {};
	}
	else
	{
		_sendStart(SSD1306_ADDR<<1);
		_waitForAck();
		_writeByte(SSD1306_DATA_CONTINUE);
		_waitForAck();
	}


		//	TWDR = scrbuf[b++];
		//	TWCR = _BV(TWEN) | _BV(TWINT) | _BV(TWEA);									// Clear TWINT to proceed
		//	while ((TWCR & _BV(TWINT)) == 0) {};										// Wait for TWI to be ready

	//if (_use_hw)					// Send TWI Stop
	       //TWCR = _BV(TWEN)| _BV(TWINT) | _BV(TWSTO);									// Send STOP
	//else
	//	_sendStop();
	//interrupts();

      #ifdef NOWAY
	if (_use_hw)
	{
		// Send start address
		TWCR = _BV(TWEN) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);						// Send START
		while ((TWCR & _BV(TWINT)) == 0) {};										// Wait for TWI to be ready
		TWDR = SSD1306_ADDR<<1;
		TWCR = _BV(TWEN) | _BV(TWINT) | _BV(TWEA);									// Clear TWINT to proceed
		while ((TWCR & _BV(TWINT)) == 0) {};										// Wait for TWI to be ready

		TWDR = SSD1306_COMMAND;
		TWCR = _BV(TWEN) | _BV(TWINT) | _BV(TWEA);									// Clear TWINT to proceed
		while ((TWCR & _BV(TWINT)) == 0) {};										// Wait for TWI to be ready
		TWDR = value;
		TWCR = _BV(TWEN) | _BV(TWINT) | _BV(TWEA);									// Clear TWINT to proceed
		while ((TWCR & _BV(TWINT)) == 0) {};										// Wait for TWI to be ready

		TWCR = _BV(TWEN)| _BV(TWINT) | _BV(TWSTO);									// Send STOP
	}
	else
	{
		_sendStart(SSD1306_ADDR<<1);
		_waitForAck();
		_writeByte(SSD1306_COMMAND);
		_waitForAck();
		_writeByte(value);
		_waitForAck();
		_sendStop();
	}
    #endif

*/
