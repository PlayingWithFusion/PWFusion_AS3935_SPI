/***************************************************************************
* File Name: as3935_lightning_snsr_nocal.ino
* Processor/Platform: Arduino Uno R3 (tested)
* Development Environment: Arduino 1.0.5
*
* Designed for use with with Playing With Fusion AS3935 Lightning Sensor
* Breakout: SEN-39001. Demo shows how this lightning sensor can be brought 
* into an Arduino project without a bunch of calibration needed. This is
* because each board is tested calibrated prior to being shipped, and the 
* cal value is written on the packaging.
*
*   SEN-39001 (universal applications)
*   ---> http://www.playingwithfusion.com/productview.php?pdid=22
*
* Copyright © 2014 Playing With Fusion, Inc.
* SOFTWARE LICENSE AGREEMENT: This code is released under the MIT License.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
* DEALINGS IN THE SOFTWARE.
* **************************************************************************
* REVISION HISTORY:
* Author		Date		Comments
* J. Steinlage		2014Aug20	Original version
* 
* Playing With Fusion, Inc. invests time and resources developing open-source
* code. Please support Playing With Fusion and continued open-source 
* development by buying products from Playing With Fusion!
*
* **************************************************************************
* ADDITIONAL NOTES:
* This file configures then runs a program on an Arduino to interface with
* an AS3935 Franklin Lightning Sensor manufactured by AMS.
*    - Configure Arduino
*    - Perform setup for AS3935 chip
*      --- capacitance registers for tuning (based on cal value provided)
*      --- configurations for your application specifics (indoor/outdoor, etc)
*    - read status/info from sensor
*    - Write formatted information to serial port
* 
* Circuit:
*    Arduino Uno   Arduino Mega  -->  AS3935 Breakout
*    SCK:  pin 13  SCK:  pin 52  -->  SCLK (must not be changed for hardware SPI)
*    MISO: pin 12  MISO: pin 50  -->  MISO (must not be changed for hardware SPI)
*    MOSI: pin 11  MOSI: pin 51  -->  MOSI (must not be changed for hardware SPI)
*    CS:   pin 10        pin 10  -->  CS0
*    SI:   pin  9        pin 9   -->  SI (select interface; GND=SPI, VDD=I2C
*    IRQ:  pin  2        pin 2   -->  IRQ
*    GND:   GND          ''      -->  GND
*    5V:     5V          ''      -->  Arduino I/O is at 5V, so power board from 5V
**************************************************************************/
// the sensor communicates via SPI or I2C. This example uses the SPI interface
#include "SPI.h"
// include Playing With Fusion AXS3935 libraries
#include "PWFusion_AS3935.h"

// setup CS pins used for the connection with the sensor
// other connections are controlled by the SPI library)
int8_t CS_PIN  = 10;
int8_t SI_PIN  = 9;
int8_t IRQ_PIN = 2;                       // digital pins 2 and 3 are available for interrupt capability
volatile int8_t AS3935_ISR_Trig = 0;

// #defines
#define AS3935_INDOORS       0
#define AS3935_OUTDOORS      1
#define AS3935_DIST_DIS      0
#define AS3935_DIST_EN       1
#define AS3935_CAPACITANCE   104      // <-- SET THIS VALUE TO THE NUMBER LISTED ON YOUR BOARD 
// prototypes
void AS3935_ISR();

PWF_AS3935  lightning0(CS_PIN, IRQ_PIN, SI_PIN);

void setup()
{
  
  Serial.begin(115200);
  Serial.println("Playing With Fusion: AS3935 Lightning Sensor, SEN-39001");
  Serial.println("beginning boot procedure....");
  
  // setup for the the SPI library:
  SPI.begin();                            // begin SPI
  SPI.setClockDivider(SPI_CLOCK_DIV16);   // SPI speed to SPI_CLOCK_DIV16/1MHz (max 2MHz, NEVER 500kHz!)
  SPI.setDataMode(SPI_MODE1);             // MAX31855 is a Mode 1 device
                                          //    --> clock starts low, read on rising edge
  SPI.setBitOrder(MSBFIRST);              // data sent to chip MSb first 
  
  lightning0.AS3935_DefInit();                        // set registers to default  
  // now update sensor cal for your application and power up chip
  lightning0.AS3935_ManualCal(AS3935_CAPACITANCE, AS3935_OUTDOORS, AS3935_DIST_EN);
                  // AS3935_ManualCal Parameters:
                  //   --> capacitance, in pF (marked on package)
                  //   --> indoors/outdoors (AS3935_INDOORS:0 / AS3935_OUTDOORS:1)
                  //   --> disturbers (AS3935_DIST_EN:1 / AS3935_DIST_DIS:2)
                  // function also powers up the chip
                  
  // enable interrupt (hook IRQ pin to Arduino Uno/Mega interrupt input: 0 -> pin 2, 1 -> pin 3 )
  attachInterrupt(0, AS3935_ISR, RISING);

}

void loop()
{
  // This program only handles an AS3935 lightning sensor. It does nothing until 
  // an interrupt is detected on the IRQ pin.
  while(0 == AS3935_ISR_Trig){}

 
  // reset interrupt flag
  AS3935_ISR_Trig = 0;
  
  // now get interrupt source
  uint8_t int_src = lightning0.AS3935_GetInterruptSrc();
  if(0 == int_src)
  {
    Serial.println("interrupt source result not expected");
  }
  else if(1 == int_src)
  {
    uint8_t lightning_dist_km = lightning0.AS3935_GetLightningDistKm();
    Serial.print("Lightning detected! Distance to strike: ");
    Serial.print(lightning_dist_km);
    Serial.println(" kilometers");
  }
  else if(2 == int_src)
  {
    Serial.println("Disturber detected");
  }
  else if(3 == int_src)
  {
    Serial.println("Noise level too high");
  }
//    lightning0.AS3935_PrintAllRegs(); // for debug...
}

// this is irq handler for AS3935 interrupts, has to return void and take no arguments
// always make code in interrupt handlers fast and short
void AS3935_ISR()
{
  AS3935_ISR_Trig = 1;
}

