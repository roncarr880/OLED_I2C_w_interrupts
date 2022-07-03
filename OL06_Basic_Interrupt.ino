
// Testing a modified version of LCD_Basic for OLED1306 128x64 I2C display. Text based display.
// If the buffer is large enough, the interrupts can handle all transfers.
// else if the transfer fills the buffer it is required to use polling to start the transfer and to make sure
// it finishes.   ( transfer fills the buffer if the data between start and stop is longer than the buffer )


// LCD5110_ViewFont 
// Copyright (C)2015 Rinky-Dink Electronics, Henning Karlsen. All right reserved
// web: http://www.RinkyDinkElectronics.com/
//
// This program is a demo of the included full font.
//

#include <Arduino.h>
#include <avr/interrupt.h>
#include <OLED1306_Basic.h>
// #include "DefaultFonts.c"         // needed if OLED Lib has fonts commmented out for another project ( usdx dual display project )
                                     // guess not needed, finding the fonts somewhere else
#define ROW0 0
#define ROW1 8
#define ROW2  16
#define ROW3  24
#define ROW4  32
#define ROW5  40
#define ROW6  48
#define ROW7  56

  OLED1306 myGLCD;

extern unsigned char SmallFont[];
extern unsigned char MediumNumbers[];
extern unsigned char BigNumbers[];

uint8_t display_dirty;


#define I2TBUFSIZE 256              // size power of 2. Size 128 can buffer a full row, max 256 as using 8 bit index
#define I2RBUFSIZE 16              // set to expected #of reads in row, power of 2
#define I2INT_ENABLED 1            // 0 for polling in loop or timer, 1 for TWI interrupts

//  I2C buffers and indexes
unsigned int i2buf[I2TBUFSIZE];   // writes
uint8_t i2rbuf[I2RBUFSIZE];       // reads
volatile uint8_t i2in,i2out;
volatile uint8_t i2rin,i2rout;
volatile uint8_t  gi2state;
uint8_t i2done = 1;
uint8_t polling;                  // if fill the buffer, may need to poll to finish the transfer

unsigned int ints, waits, extra;

void setup()
{

  Serial.begin(38400);
  Serial.println("TWI interrupt test");
  pinMode(13,OUTPUT);     // testing 
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);

  // Timer0 is already used for millis() - interrupt somewhere in middle
 // OCR0A = 0xAF;
 // TIMSK0 |= _BV(OCIE0A);
}

// TWI interrupt version
// if twi interrupts enabled, then need a handler
// Stop does not produce an interrupt, so need to also poll in loop or via the timer 
ISR(TWI_vect){
  i2poll();
  if( gi2state == 0 ) i2poll();   // needed to get out of state zero, else double ints
  ++ints;
}

// timer polling.  about 1000 cps or 8k baud
//ISR(TIMER0_COMPA_vect){
//   i2poll();
//}

void loop(){

  if( polling ){            // may need to finish a transfer if the buffer was at max, int flag 
     noInterrupts();
     polling = i2poll();
     interrupts();
  }
  
  int i;
  ints = 0; waits = 0; extra = 0;
  
  myGLCD.setFont(SmallFont);
  myGLCD.clrScr();
  myGLCD.print("Hello    ",0,ROW1);
  delay(1000);   Serial.print(polling);
  myGLCD.print("Upper case: p0r0", LEFT, ROW0);
  myGLCD.print("ABCDEFGHIJKLM", CENTER, ROW2);
  myGLCD.print("NOPQRSTUVWXYZ", CENTER, ROW3);
  myGLCD.print("  Page 1  Write ", CENTER, ROW5);
  delay(1000);  Serial.print(polling);

  if( polling ){            // may need to finish a transfer if the buffer was at max, int flag 
     noInterrupts();
     polling = i2poll();
     interrupts();
  }

  myGLCD.clrScr();
  myGLCD.print("Numbers:", LEFT, ROW0);
  myGLCD.setFont(MediumNumbers);
  myGLCD.print("0123456789", CENTER, ROW2);
  myGLCD.setFont(SmallFont);
  delay (1000);  Serial.print(polling);
  myGLCD.clrRow(5);
  myGLCD.print("Lower case: p1r0", LEFT, ROW4);  // write to hidden page, not hidden on 128x64
  myGLCD.print("abcdefghijklm", CENTER, ROW6);
  myGLCD.print("nopqrstuvwxyz", CENTER, ROW7);
  delay ( 1000 );  Serial.print(polling);

  if( polling ){            // may need to finish a transfer if the buffer was at max, int flag 
     noInterrupts();
     polling = i2poll();
     interrupts();
  }

  
  myGLCD.clrScr();
  myGLCD.print("Big Numbers:", LEFT, ROW0);
  myGLCD.setFont(BigNumbers);
  myGLCD.print("012345678", LEFT, ROW2);
  myGLCD.setFont(SmallFont);
  delay (1000);  Serial.print(polling);
  
  myGLCD.clrScr();
  myGLCD.print("Special:", LEFT, ROW0);
  myGLCD.print("!\"#$%&'()*+,-.End", RIGHT, ROW2);
  myGLCD.print("/:;<=>?@[\\]^_`End", RIGHT, ROW3);
  delay (1000);  Serial.print(polling);

    if( polling ){            // may need to finish a transfer if the buffer was at max, int flag 
     noInterrupts();
     polling = i2poll();
     interrupts();
  }
  Serial.print(polling);
  
  display_dirty = 6;
  freq_display( 3582000 );
  delay( 1000 );  Serial.print(polling);
  freq_display( 7185000 );
  delay( 1000 );  Serial.print(polling);
  freq_display( 10106800 );
  delay( 1000 );  Serial.print(polling);
  freq_display( 14285000 );
  delay( 1000 );  Serial.print(polling);
  freq_display( 21010234 );
  delay( 1000 );  Serial.print(polling);
  freq_display( 28345020 );
  delay( 1000 );  Serial.print(polling);
  freq_display( 5000130 );
  delay( 1000 );  Serial.print(polling);

   if( polling ){            // may need to finish a transfer if the buffer was at max, int flag 
     noInterrupts();
     polling = i2poll();
     interrupts();
  }
 
  // try the new functions
  
  myGLCD.clrScr();
  myGLCD.gotoRowCol(1,5);
  
  for( i = 1; i <= 0xff; ){
     myGLCD.write(i,2);  // bars 
     myGLCD.write(0);    // with spaces
     i <<= 1;
     i |= 1;
  }  
  myGLCD.putch('S');
  
  myGLCD.gotoRowCol(3,4);
char buf[12]; 
String st = "Hello";
  st.toCharArray( buf,6 );
  myGLCD.puts(buf);
 // myGLCD.puts(st.c_str());  // pointer seems to be in program rom instead of ram ??
  delay(1000);  Serial.print(polling);

  myGLCD.clrScr();
  freq_display( 14285000 );
  
  myGLCD.gotoRowCol(1,0);
  myGLCD.putch('S');
  myGLCD.write(0);
  for( i = 0x80; i < 0xff; ){
     myGLCD.write(i,2);  // bars 
     myGLCD.write(0);    // with spaces
     i >>= 1;
     i |= 0x80;
  }  
  myGLCD.print("USB",32,ROW0);
  myGLCD.print("QST DE W1AW - TEXT IS",0,ROW5);
  delay( 1000 );  Serial.print(polling);
 

 myGLCD.printNumI(ints,0,ROW7);
 myGLCD.printNumI(waits,CENTER,ROW7);
 //i2flush();
 //delay(1000);

 Serial.println();
 Serial.print("Waits..");  Serial.print(waits);       // too much in the buffer at once
 Serial.print(" Ints..");  Serial.print(ints);        // is it working with ints
 Serial.print(" Kicks.");  Serial.print(extra);     // extra kick needed in start
 // Serial.print(" Total.");  Serial.print( waits + ints + extra );
 Serial.println();
 delay(3000);
}


void freq_display( unsigned long val ){
int x;

      //myGLCD.clrRow(1,0,127);
      //myGLCD.clrRow(2,0,127);
 
     if( display_dirty & 6 ){       // clean up after alternate use of the display
       myGLCD.clrRow(3,0,127);
       myGLCD.clrRow(2,0,127);
       display_dirty &= ( 0x39 );
     }

      myGLCD.setFont(MediumNumbers);
    //  x= (val >= 10000000 ) ? 0 : 12 ;
       x= 4 *12;
      myGLCD.printNumI(val/1000,x,ROW2,5,'/');    // space character not correct ??
      myGLCD.setFont(SmallFont);
      myGLCD.printNumI((val)%1000,9*12,ROW2,3,'0');

     // LCD.setFont(SmallFont);  redundant

}

/*****  Non-blocking  I2C  functions   ******/

void i2init(){
  TWSR = 0;
  TWBR = 12;   //8 500k,  12 400k, 72 100k   for 16 meg clock. ((F_CPU/freq)-16)/2
  TWDR = 0xFF;    // ?? why
  // PRR = 0;        // bit 7 should be cleared  &= 7F
  PRR &= 0x7F;
  //TWSR = 0;
  TWSR = 1<<TWEN;
  i2done = 1;
}
// use some upper bits in the buffer for control
#define ISTART 0x100
#define ISTOP  0x200


// wait for previous transfer to finish if any, queue a start condition
void i2start( unsigned char adr ){
unsigned int dat;
uint8_t t;

  //Serial.print("Start1 ");  Serial.println(gi2state);
  while( i2done == 0 ){             // wait for finish of previous transfer
     noInterrupts();
     t = i2out;                     // !!! what about reads on i2c, needs work here i2rin changing if reading
     interrupts();
     delayMicroseconds( 40 );       // i2out should be changing if have interrupts happening 400k 
     noInterrupts();
     if( t == i2out ) ++extra, i2poll();    // kick if stuck
     interrupts();
  }
  dat = ( adr << 1 ) | ISTART;      // shift the address over and add the start flag
  i2send( dat );
  //Serial.println("Start2");
}


void i2send( unsigned int data ){   // just save stuff in the buffer
uint8_t  next;
uint8_t  t;

  // check for buffer full
  next = (i2in + 1) & (I2TBUFSIZE-1);
  noInterrupts();
  t = i2out;
  interrupts();
  if( t == next ) ++waits, polling = 1;  // may need to finish this transfer via polling as
  while( t == next ){                    // this transfer is bigger than our buffer.  
         noInterrupts(); 
         i2poll();                  // wait for i2out to move
         t = i2out;
         interrupts();
  }
     //Serial.print("Buf Full");  Serial.write(' ');
     //Serial.println( ++waits ); 
  i2buf[i2in++] = data;
  i2in &= (I2TBUFSIZE - 1);
}

void i2stop( ){
   //Serial.print("Stop1 Ints Waits Extra ");  Serial.print(ints); Serial.write(' '); Serial.print(waits);
   //Serial.write(' '); Serial.println( extra );
   i2send( ISTOP );   // que a stop condition
   i2done = 0;
   noInterrupts();    // kick off sending this buffer
   i2poll();
   interrupts();
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
static unsigned int read_qty;
static uint8_t first_read;

   
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
              //delay_counter = 5;   // delay for transmit active to come true
              state = 2;
           }
        }
        else TWCR = (1<<TWEN);      // stop interrupts, keep enabled
      // break;                     // no break, check for stop completion
      case 3:  // wait for stop to clear. TWINT does not return to high and no interrupts happen on stop.
         if( state != 3 ) break;
         while( 1 ){
            if( (TWCR & (1<<TWSTO)) == 0 ){
               state = 0;
               i2done = 1;
               break;
            }
         }
      break;
      
      case 1:  // wait for start to clear, send saved data which has the device address
         if( (TWCR & (1<<TWINT)) ){
            state = ( data & 1 ) ? 4 : 2;    // read or write pending?
            first_read = 1;
            TWDR = data;
            TWCR = (1<<TWINT) | (1<<TWEN) | (I2INT_ENABLED);
            //delay_counter = 5;
         }
      break;
      case 2:  // wait for done
         if( (TWCR & (1<<TWINT)) ){  
            state = 0;
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
            //delay_counter = 5;
         }
      break;    
   }
   
  // if( state == 3 ) digitalWrite(13,HIGH);   //!!! testing
  // else digitalWrite(13,LOW);
   
   gi2state = state;
   if( i2in != i2out ) return (state + 8);
   else return state;
}
