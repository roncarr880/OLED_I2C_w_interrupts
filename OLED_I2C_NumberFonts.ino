// rwc - testing altered version of OLED_I2C library and a I2C interrupt handler
//       Why wait for a slow system like I2C to do transfers. Why block for 20-40ms.
//
//       The AVR part of the library was altered to call routines contained in this program.
//       Also the library was altered to allow updating per row rather than the entire screen.
//       The I2C transfers are buffered and an interrupt process empties the buffer.
//
//       summary:   it looks like with the 128 size buffer, writing a complete row of 128 bytes
// + the 9 bytes of overhead, at most 1 wait is required.  So discounting the time required to
// actually write the data into the buffer, the screen update time for just one row is basically zero.
// The system is ready to write another row in 3ms. ( 137 characters * 8bits at 400k baud = 2.74ms )
//
// To process the entire screen, one could write 1 row every 3ms using the time in between to 
// do other tasks.   Set write_row to zero to start a refresh.  One could use a mask to only
// update the rows wanted.  ( just made up variable names for this example )
/*      Somewhere in loop()
 *       
 *      if( write_row < 8 && millis() - update_time > 2 ){
 *         if( row_mask & ( 1 << write_row ) ){
 *            myOLED.update_row(write_row);
 *            update_time = millis();
 *         }   
 *         write_row++;
 *      }
 *      do other stuff.  The next 3ms is now recovered processing time.
 */

// OLED_I2C_NumberFonts
// Copyright (C)2018 Rinky-Dink Electronics, Henning Karlsen. All right reserved
// web: http://www.RinkyDinkElectronics.com/
//
// **********************************************
// *** This demo is for 128x64 pixel displays ***
// **********************************************

// A quick demo of how to use my OLED_I2C library.
//
// To use the hardware I2C (TWI) interface of the Arduino you must connect
// the pins as follows:
//
// Arduino Uno/2009:
// ----------------------
// Display:  SDA pin   -> Arduino Analog 4 or the dedicated SDA pin
//           SCL pin   -> Arduino Analog 5 or the dedicated SCL pin
//
// Arduino Leonardo:
// ----------------------
// Display:  SDA pin   -> Arduino Digital 2 or the dedicated SDA pin
//           SCL pin   -> Arduino Digital 3 or the dedicated SCL pin
//
// Arduino Mega:
// ----------------------
// Display:  SDA pin   -> Arduino Digital 20 (SDA) or the dedicated SDA pin
//           SCL pin   -> Arduino Digital 21 (SCL) or the dedicated SCL pin
//
// Arduino Due:
// ----------------------
// Display:  SDA pin   -> Arduino Digital 20 (SDA) or the dedicated SDA1 (Digital 70) pin
//           SCL pin   -> Arduino Digital 21 (SCL) or the dedicated SCL1 (Digital 71) pin
//

#include <Arduino.h>
#include <avr/interrupt.h>

#define I2TBUFSIZE 128             // size power of 2
#define I2RBUFSIZE 16              // set to expected #of reads in row, power of 2
#define I2INT_ENABLED 1            // 0 for polling in loop, 1 for interrupts
//  I2C buffers and indexes
unsigned int i2buf[I2TBUFSIZE];   // writes
uint8_t i2rbuf[I2RBUFSIZE];       // reads
volatile uint8_t i2in,i2out;
volatile uint8_t i2rin,i2rout;
volatile uint8_t  gi2state;

#include <OLED_I2C.h>
#include <arduino.h>
#include <avr/interrupt.h>



OLED  myOLED(SDA, SCL); // Remember to add the RESET pin if your display requires it...

extern uint8_t SmallFont[];
extern uint8_t MediumNumbers[];
extern uint8_t BigNumbers[];

uint32_t waits;
uint32_t ints;


void setup()
{
 // if(!myOLED.begin(SSD1306_128X64))   moved below in case hang can still get serial message
 //   while(1);   // In case the library failed to allocate enough RAM for the display buffer...
    
 // myOLED.setFont(SmallFont);
 
  Serial.begin(38400);

 // Timer0 is already used for millis() - we'll just interrupt somewhere in middle
 // OCR0A = 0xAF;
 // TIMSK0 |= _BV(OCIE0A);

  Serial.println(F(".... Always wake up, 3ms delays seem optimum"));
  delay(1000);

  if(!myOLED.begin(SSD1306_128X64))
    while(1);   // In case the library failed to allocate enough RAM for the display buffer...    
  myOLED.setFont(SmallFont);

} 
 
// TWI interrupt version
// if twi interrupts enabled, then need a handler
ISR(TWI_vect){
  i2poll();
  if( gi2state == 0 ) i2poll();   // needed to get out of state zero, else double ints
  ++ints;
}

//ISR(TIMER0_COMPA_vect){
//   i2poll();             // polling with a timer instead of in loop
//}                        // about 8k baud  or 1000 bytes/sec

void loop(){
static uint8_t flip;       // update by row vs complete screen updates

  //i2poll();              // a polling only version, disable interrupts with #define I2INT_ENABLED 0
                           // doesn't really work with this example due to the large amount of screen
                           // writes.
  for (int i=0; i<=1000; i++)
  {
    myOLED.setFont(MediumNumbers);
    myOLED.printNumF(float(i)/3, 2, RIGHT, 0);
    myOLED.setFont(BigNumbers);
    myOLED.printNumI(i, RIGHT, 40);
    myOLED.setFont(SmallFont);
    myOLED.printNumI(i,LEFT,40);
    if( i == 0 || flip ) myOLED.update();      // 1st write after clear screen buffer (40ms)
    else{                                      
       myOLED.update_row(0); delay(3);      // the delays represent time saved to do useful work
       myOLED.update_row(1); delay(3);      // if we had something to do
       myOLED.update_row(5); delay(3);
       myOLED.update_row(6); delay(3);
       myOLED.update_row(7); delay(3);
    }
    delay(1);
    
  }

    delay(1000);   // did it finish ( big numbers completely formed ? )
    
  myOLED.setFont(SmallFont);
  myOLED.print("|", LEFT, 24);
  myOLED.print("|", RIGHT, 24);
  if( flip )myOLED.update();
  else myOLED.update_row(3);
  delay(500);
  for (int i=0; i<19; i++)
  {
    myOLED.print("\\", 7+(i*6), 24);
    if( flip ) myOLED.update();
    else myOLED.update_row(3);
    delay(250);
  }
  myOLED.clrScr();
  Serial.print(F("Waits..")); Serial.print(waits);
  Serial.print(F("  Interrupts..")); Serial.println(ints);
  waits = 0;
  ints = 0;
  flip ^= 1;

}


/*****  Non-blocking  I2C  functions   ******/

void i2init(){
  TWBR = 12;   //8 500k,  12 400k, 72 100k   for 16 meg clock. ((F_CPU/freq)-16)/2
  TWDR = 0xFF;    // ?? why
  // PRR = 0;        // bit 7 should be cleared  &= 7F
  PRR &= 0x7F;
  //TWSR = 0;
  TWSR = 1<<TWEN;
  // i2stop();    // put a stop on the bus on powerup, hangs everything, why?
}
// use some upper bits in the buffer for control
#define ISTART 0x100
#define ISTOP  0x200

void i2start( unsigned char adr ){
unsigned int dat;
  // shift the address over and add the start flag
  dat = ( adr << 1 ) | ISTART;
  i2send( dat );
}


void i2send( unsigned int data ){   // just save stuff in the buffer
uint8_t  next;

  // check for buffer full
  next = (i2in + 1) & (I2TBUFSIZE-1);
  if( next == i2out ){
     while( next == i2out ){
         noInterrupts();
         i2poll();        // wait for i2out to move
         interrupts();
     }
    ++waits;
  }
  
  i2buf[i2in++] = data;
  i2in &= (I2TBUFSIZE - 1);
  
  noInterrupts();
  i2poll();         // wake up interrupts when new data arrives in case stopped in state zero
  interrupts();

}

void i2stop( ){
   i2send( ISTOP );   // que a stop condition
}


void i2flush(){  // call flush to empty out the buffer, waits on I2C transactions completed 
uint8_t  ex;

  ex = 1;
  while(ex){
     noInterrupts();
     ex = i2poll(); 
     interrupts();
  }
}

// queue a read that will complete later
void i2queue_read( unsigned char adr, unsigned char reg, unsigned char qty ){
unsigned int dat;

   i2start( adr );
   i2send( reg );
   // i2stop();     // or repeated start
   dat = ((unsigned int)qty << 10) | ( adr << 1 ) | ISTART | 1;   // a start with the read bit set
   i2send( dat );
   i2stop();        // stop to complete the transaction
}

int i2read_int(){     // returns 2 values in i2c read queue as a signed integer
int data;

      if( i2rout == i2rin ) return 0;
      data = i2rbuf[i2rout++];
      i2rout &= (I2RBUFSIZE-1);
      data <<= 8;
      if( i2rout == i2rin ) return data;
      data |= i2rbuf[i2rout++];
      i2rout &= (I2RBUFSIZE-1);
      return data;
}

uint8_t i2available(){
uint8_t qty;

     qty = i2rin - i2rout;
     if( qty > I2RBUFSIZE ) qty += I2RBUFSIZE;    // some funky unsigned math
     return qty;
}



uint8_t i2poll(){    // everything happens here.  Call this from loop. or interrupt
static  uint8_t state = 0;
static unsigned int data;
static uint8_t delay_counter;
static unsigned int read_qty;
static uint8_t first_read;

   /*****                 // don't want this delay if using interrupts
   if( delay_counter ){   // the library code has a delay after loading the transmit buffer
     --delay_counter;     //  and before the status bits are tested
     return (16 + delay_counter);
   }
   *****/
   
   switch( state ){    
      case 0:      // idle state or between characters
        if( i2in != i2out ){   // get next character
           data = i2buf[i2out++];
           i2out &= (I2TBUFSIZE - 1 );
         
           if( data & ISTART ){   // start
              if( data & 1 ) read_qty = data >> 10;     // read queued
              data &= 0xff;
              // set start condition
              TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (I2INT_ENABLED); 
              state = 1; 
           }
           else if( data & ISTOP ){  // stop
              // set stop condition
              TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO) | (I2INT_ENABLED);
              state = 3;
           }
           else{   // just data to send
              TWDR = data;
              TWCR = (1<<TWINT) | (1<<TWEN) | (I2INT_ENABLED);
              delay_counter = 5;   // delay for transmit active to come true
              state = 2;
           }
        }
        else TWCR = (1<<TWEN);      // stop interrupts, keep enabled
      break; 
      case 1:  // wait for start to clear, send saved data which has the device address
         if( (TWCR & (1<<TWINT)) ){
            state = ( data & 1 ) ? 4 : 2;    // read or write pending?
            first_read = 1;
            TWDR = data;
            TWCR = (1<<TWINT) | (1<<TWEN) | (I2INT_ENABLED);
            delay_counter = 5;
         }
      break;
      case 2:  // wait for done
         if( (TWCR & (1<<TWINT)) ){  
            state = 0;
         }
      break;
      case 3:  // wait for stop to clear
         if( (TWCR & (1<<TWSTO)) == 0 ){
            state = 0;
            delay_counter = 5;  // a little delay at the end of a sequence
         }
      break;
      case 4:  // read until count has expired
         if( (TWCR & (1<<TWINT)) ){
            // read data
            if( first_read ) first_read = 0;       // discard the 1st read, need 8 more clocks for real data
            else{
               i2rbuf[i2rin++] = TWDR;
               i2rin &= ( I2RBUFSIZE - 1 );
               if( --read_qty == 0 ) state = 0;    // done
            }
            
            if( read_qty ){                        // any left ?
               if( read_qty > 1 ) TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (I2INT_ENABLED);  // not the last read
               else TWCR = (1<<TWINT) | (1<<TWEN) | (I2INT_ENABLED);                            // nack the last read
            }
            delay_counter = 5;
         }
      break;    
   }
   gi2state = state;
   if( i2in != i2out ) return (state + 8);
   else return state;
}

/****************************************************/
