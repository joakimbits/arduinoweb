/* Name: weather_station.ino
Based on: lacrosse.pde:
 * Version: 1.1
 * Author: Kelsey Jordahl
 * Copyright: Kelsey Jordahl 2010
   (portions copyright Marc Alexander, Jonathan Oxer 2009;
   Interactive Matter 2009 [licensed under GPL with permission])
 * License: GPLv3
 * Time-stamp: <Mon Jun 27 12:02:27 EDT 2011> 
Editors: Joakim Pettersson (JP)
History:
2012-06-28 JP deleted code for non-wireless sensors.

Receive weather sensor data with Arduino and send to serial (USB) port. 
Assumes the 433 MHz data pin is connected to Digital Pin 8 (PB0). 

Based on idea, and some code, from Practical Arduino
 http://www.practicalarduino.com/projects/weather-station-receiver
 http://github.com/practicalarduino/WeatherStationReceiver

Also useful was the detailed data protocol description at
 http://www.f6fbb.org/domo/sensors/tx3_th.php

433.92 MHz RF receiver:
 http://www.sparkfun.com/commerce/product_info.php?products_id=8950

    This program is free software: you can redistribute it and/or
    modify it under the terms of the GNU General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.  A copy of the GPL
    version 3 license can be found in the file COPYING or at
    <http://www.gnu.org/licenses/>.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*/

#include <math.h>
#include <Wire.h>

// Comment out for a normal build
// Uncomment for a debug build
//#define DEBUG

#define INPUT_CAPTURE_IS_RISING_EDGE()    ((TCCR1B & _BV(ICES1)) != 0)
#define INPUT_CAPTURE_IS_FALLING_EDGE()   ((TCCR1B & _BV(ICES1)) == 0)
#define SET_INPUT_CAPTURE_RISING_EDGE()   (TCCR1B |=  _BV(ICES1))
#define SET_INPUT_CAPTURE_FALLING_EDGE()  (TCCR1B &= ~_BV(ICES1))
#define GREEN_TESTLED_ON()          ((PORTD &= ~(1<<PORTD6)))
#define GREEN_TESTLED_OFF()         ((PORTD |=  (1<<PORTD6)))
// I reversed the red - did I flip the LED from the schematic?
#define RED_TESTLED_OFF()            ((PORTD &= ~(1<<PORTD7)))
#define RED_TESTLED_ON()           ((PORTD |=  (1<<PORTD7)))

/* serial port communication (via USB) */
#define BAUD_RATE 9600

#define PACKET_SIZE 9   /* number of nibbles in packet (after inital byte) */
#define PACKET_START 0x0A	/* byte to match for start of packet */

// 0.5 ms high is a one
#define MIN_ONE 135		// minimum length of '1'
#define MAX_ONE 155		// maximum length of '1'
// 1.3 ms high is a zero
#define MIN_ZERO 335		// minimum length of '0'
#define MAX_ZERO 370		// maximum length of '0'
// 1 ms between bits
#define MIN_WAIT 225		// minimum interval since end of last bit
#define MAX_WAIT 275		// maximum interval since end of last bit

unsigned int CapturedTime;
unsigned int PreviousCapturedTime;
unsigned int CapturedPeriod;
unsigned int PreviousCapturedPeriod;
unsigned int SinceLastBit;
unsigned int LastBitTime;
unsigned int BitCount;
byte j;
float tempC;			/* temperature in deg C */
float tempF;			/* temperature in deg F */
float dp;			/* dewpoint (deg C) */
byte h;				/* relative humidity */
byte DataPacket[PACKET_SIZE];	  /* actively loading packet */
byte FinishedPacket[PACKET_SIZE]; /* fully read packet */
byte PacketBitCounter;
boolean ReadingPacket;
boolean PacketDone;

byte CapturedPeriodWasHigh;
byte PreviousCapturedPeriodWasHigh;
byte mask;		    /* temporary mask byte */
byte CompByte;		    /* byte containing the last 8 bits read */

volatile unsigned int tick = 0;			/* count ticks of the clock */
unsigned int starttime;
unsigned int interval;
boolean timerflag = false;	/* flag to set when timer goes off */

ISR( TIMER1_CAPT_vect )
{
  // Immediately grab the current capture time in case it triggers again and
  // overwrites ICR1 with an unexpected new value
  CapturedTime = ICR1;

  // GREEN test led on (flicker for debug)
  GREEN_TESTLED_ON();
  if( INPUT_CAPTURE_IS_RISING_EDGE() )
  {
    SET_INPUT_CAPTURE_FALLING_EDGE();      //previous period was low and just transitioned high
    CapturedPeriodWasHigh = false;    //uiICP_CapturedPeriod about to be stored will be a low period
  } else {
    SET_INPUT_CAPTURE_RISING_EDGE();       //previous period was high and transitioned low
    CapturedPeriodWasHigh = true;     //uiICP_CapturedPeriod about to be stored will be a high period
  }

  CapturedPeriod = (CapturedTime - PreviousCapturedTime);

  if ((CapturedPeriod > MIN_ONE) && (CapturedPeriodWasHigh == true)) { // possible bit
    /* time from end of last bit to beginning of this one */
    SinceLastBit = (PreviousCapturedTime - LastBitTime);
    
    if ((CapturedPeriod < MAX_ONE) && (SinceLastBit > MIN_WAIT)) {
      if (SinceLastBit > MAX_WAIT) { // too long since last bit read
	if ((SinceLastBit > (2*MIN_WAIT+MIN_ONE)) && (SinceLastBit < (2*MAX_WAIT+MAX_ONE))) { /* missed a one */
          #ifdef DEBUG
	  Serial.println("missed one");
	  #endif
	} else {
	  if ((SinceLastBit > (2*MIN_WAIT+MIN_ZERO)) && (SinceLastBit < (2*MAX_WAIT+MAX_ZERO))) { /* missed a zero */
            #ifdef DEBUG
	    Serial.println("missed zero");
	    #endif
	  }
	}
	RED_TESTLED_OFF();
	if (ReadingPacket) {
          #ifdef DEBUG
	  Serial.print("dropped packet. bits read: ");
	  Serial.println(PacketBitCounter,DEC);
	  #endif
	  ReadingPacket=0;
	  PacketBitCounter=0;
	}
	CompByte=0xFF;			  /* reset comparison byte */
      } else { /* call it a one */
	if (ReadingPacket) {	/* record the bit as a one */
	  //	  Serial.print("1");
	  mask = (1 << (3 - (PacketBitCounter & 0x03)));
	  DataPacket[(PacketBitCounter >> 2)] |= mask;
	  PacketBitCounter++;
	} else {		  /* still looking for valid packet data */
	  if (CompByte != 0xFF) {	/* don't bother recording if no zeros recently */
	    CompByte = ((CompByte << 1) | 0x01); /* push one on the end */
	  }
	}
	LastBitTime = CapturedTime;
      }
    } else {			/* Check whether it's a zero */
      if ((CapturedPeriod > MIN_ZERO) && (CapturedPeriod < MAX_ZERO)) {
	if (ReadingPacket) {	/* record the bit as a zero */
	  //	  Serial.print("0");
	  mask = (1 << (3 - (PacketBitCounter & 0x03)));
	  DataPacket[(PacketBitCounter >> 2)] &= ~mask;
	  PacketBitCounter++;
	} else {		      /* still looking for valid packet data */
	  CompByte = (CompByte << 1); /* push zero on the end */
/* 	  if ((CompByte & 0xF0) != 0xf0) { */
/* 	    Serial.println(CompByte,HEX); */
/* 	  } */
	}
	LastBitTime = CapturedTime;
      }
    }
  }

  if (ReadingPacket) {
    if (PacketBitCounter == (4*PACKET_SIZE)) { /* done reading packet */
      memcpy(&FinishedPacket,&DataPacket,PACKET_SIZE);
      RED_TESTLED_OFF();
      PacketDone = 1;
      ReadingPacket = 0;
      PacketBitCounter = 0;
    }
  } else {
    /* Check whether we have the start of a data packet */
    if (CompByte == PACKET_START) {
      //      Serial.println("Got packet start!");
      CompByte=0xFF;		/* reset comparison byte */
      RED_TESTLED_ON();
      /* set a flag and start recording data */
      ReadingPacket = 1;
    }
  }

  //save the current capture data as previous so it can be used for period calculation again next time around
  PreviousCapturedTime           = CapturedTime;
  PreviousCapturedPeriod         = CapturedPeriod;
  PreviousCapturedPeriodWasHigh   = CapturedPeriodWasHigh;
  
  //GREEN test led off (flicker for debug)
  GREEN_TESTLED_OFF();
}

float dewpoint(float T, float h) {
  float td;
  // Simplified dewpoint formula from Lawrence (2005), doi:10.1175/BAMS-86-2-225
  td = T - (100-h)*pow(((T+273.15)/300),2)/5 - 0.00135*pow(h-84,2) + 0.35;
  return td;
}

void setup() {
  Serial.begin( BAUD_RATE );   //using the USB serial port for debugging and logging
  Serial.println( "La Crosse weather station capture begin" );
  Wire.begin();

  cli();
  DDRB = 0x2F;   // B00101111
  DDRB  &= ~(1<<DDB0);    //PBO(ICP1) input
  PORTB &= ~(1<<PORTB0);  //ensure pullup resistor is also disabled

  //PORTD6 and PORTD7, GREEN and RED test LED setup
  DDRD  |=  B11000000;      //(1<<PORTD6);   //DDRD  |=  (1<<PORTD7); (example of B prefix)
  GREEN_TESTLED_OFF();      //GREEN test led off
//  RED_TESTLED_ON();         //RED test led on
  // Set up timer1 for RF signal detection
  TCCR1A = B00000000;   //Normal mode of operation, TOP = 0xFFFF, TOV1 Flag Set on MAX
  TCCR1B = ( _BV(ICNC1) | _BV(CS11) | _BV(CS10) );
  SET_INPUT_CAPTURE_RISING_EDGE();
  //Timer1 Input Capture Interrupt Enable, Overflow Interrupt Enable  
  TIMSK1 = ( _BV(ICIE1) | _BV(TOIE1) );
  //  Set up timer2 for countdown timer
  TCCR2A = (1<<WGM21);				/* CTC mode */
  TCCR2B = ((1<<CS22) | (1<<CS21) | (1<<CS20)); /* clock/1024 prescaler */
  TIMSK2 = (1<<OCIE2A);	  /* enable interupts */
  ASSR &= ~(1<<AS2);	  /* make sure we're running on internal clock */
  OCR2A = 155;	       /* interrupt f=100.16 Hz, just under 10 ms period */
  sei();
  interrupts();   // Enable interrupts (NOTE: is this necessary? Should be enabled by default)

}

// in the main loop, just hang around waiting to see whether the interrupt routine has gathered a full packet yet
void loop() {

  delay(2);                  // wait for a short time
  if (PacketDone) {	     // have a bit string that's ended
    ParsePacket(FinishedPacket);
    PacketDone=0;
  }
}


// parse a raw data string
void ParsePacket(byte *Packet) {

  byte chksum;

  #ifdef DEBUG
  Serial.print("RAW: ");
  for (j=0; j<PACKET_SIZE; j++) {
    Serial.print(Packet[j], HEX);
  }	
  Serial.println("");
  #endif

  chksum = 0x0A;
  for (j=0; j<(PACKET_SIZE-1); j++) {
    chksum += Packet[j];
  }
  
  if ((chksum & 0x0F) == Packet[PACKET_SIZE-1]) { /* checksum pass */
    /* check for bad digits and make sure that most significant digits repeat */
    if ((Packet[3]==Packet[6]) && (Packet[4]==Packet[7]) && (Packet[3]<10) && (Packet[4]<10) && (Packet[5]<10)) {
      if (Packet[0]==0) {		/* temperature packet */
	Serial.print("DATA: T= ");
	tempC=(Packet[3]*10-50 + Packet[4] + ( (float) Packet[5])/10);
	tempF=tempC*9/5 + 32;
	Serial.print(tempC,1);	/* print to 0.1 deg precision */
	Serial.print(" degC, ");
	Serial.print(tempF,1);	/* print to 0.1 deg precision */
	Serial.println(" degF");
	/* PrintIndoor(); // moved to time interval sampling with pressure */
	dp=dewpoint(tempC,h);
	Serial.print("DEWPOINT: ");
	Serial.print(dp,1);
	Serial.print(" degC, ");
	Serial.print(dp*9/5 + 32,1);
	Serial.println(" degF");
      } else {
	if (Packet[0]==0x0E) {		/* humidity packet */
	  Serial.print("DATA: H= ");
	  h=(Packet[3]*10 + Packet[4]);
	  Serial.print(h,DEC);
	  Serial.println(" %");
	} else  {
	  if (Packet[0]==0x0B) {		/* custom packet */
	    Serial.print("CUSTOM: T= ");
	    tempC=(Packet[3]*10-50 + Packet[4] + ( (float) Packet[5])/10);
	    tempF=tempC*9/5 + 32;
	    Serial.print(tempC,1);	/* print to 0.1 deg precision */
	    Serial.print(" degC, ");
	    Serial.print(tempF,1);	/* print to 0.1 deg precision */
	    Serial.println(" degF");
	  }
	}
      }
    } else {
      #ifdef DEBUG
      Serial.println("Fail secondary data check.");
      #endif
    }
  } else {			/* checksum fail */
    #ifdef DEBUG
    Serial.print("chksum = 0x");
    Serial.print(chksum,HEX);
    Serial.print(" data chksum = 0x");
    Serial.println(Packet[PACKET_SIZE-1],HEX);
    #endif
  }
}
