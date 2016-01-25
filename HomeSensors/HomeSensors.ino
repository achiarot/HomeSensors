/* This is the sketch for the home sensor network.
 * V1
 * Created by Anthony Chiarot
 */

/*************Configuration*******************
 ********************************************/
/*************NRF24L setup********************
 *Define the CE and CS pin to be used by the RF24 device
 *you will use the default SPI pins from the arduino plus the two below
 *SCK 13    MOSI  11   MISO  12   
 *********************************************
 */
#define NRFRADIO
#define radioCE 7
#define radioCS 8

/***************Sleep Settings*****************/
/*Set the amount of time to go to sleep (in seconds) also the time between gathering
 *data from attached devices. Must be a multiple of 8
 *If the device is powered by battery we will enter a lower power sleep 
 */
#define SLEEPTIME 600
//#define ONBATTERY

/************Setup attached devices************/
/***********Interupt enabled devices***********/
/*These devices must use pins 2 or 3 for arduino atmel 328 based boards
 *for other boards refer to https://www.arduino.cc/en/Reference/AttachInterrupt
 *for a list of pins.
 *Anything that is defined here will create an interupt which will wake the 
 *arduino and send all reference data attached to that device.  Any alarming
 *will be done on the receiver side.  If the interrupt pins are not used for 
 *waking they can be used in the sections below for regular devices 
 *
 *If you will be using outputs, you will need to setup interrupts for data
 * reception.  You will need to attach the IRQ line from the nrf24 to one of
 * the Interrupt pins on the arduino.
 */
//#define RADIOISR 2

/////////This may be outdated information and no need for using LOW only
/*All ISR's will be set using a LOW pin state. It may be useful to break it out 
 * to trigger on other changes but for now it will only use LOW, therefor all switches
 * will trigger when pulled to ground so make sure to take that into account. 
 * (See arduino documentation on ISR's)
 * If you are going to use a switch triggering on a door, you will need to use both
 * pins. I will include some documentation for a circuit that will use one reed switch
 * to trigger on LOW for both.  This is because we want to be able to use the lowest 
 * power sleep mode.
 * 

 /* If you use an entry switch you cannot use those two pins for any other interupt.
 */
//#define ENTRYSWITCH {2,3} //The second pin will need to have a pulldown resistor
/////////This may be outdated information and no need for using LOW only

//#define ISRPINS {2,3} //example
#define ISRPINS {3}

/**********************************************/

/****************Input devices*****************/
/*There will be a number of possible devices that can be defined.  Each type 
 * will be held in an array of numbers.
 */
 
/*More than one DS18B20 can be hooked up to the same pin but for now is not 
 *supported*/
//#define DS18B20 4
//#define Celcius
//#define Fahrenheit

/**********************************************/

/***************Output devices*****************/
/*There will be a defined array of pins used for output.  Some will be simple
 * on/offs but as development continues I will develop LED outputs, for example,
 * utilizing PWM, among other things as I find them useful.
 */
 
/*SIMPLEOUTHI are initialized as high upon starting*/
//#define SIMPLEOUTHI {12}

/*SIMPLEOUTLO are initialized as low upon starting*/
//#define SIMPLEOUTLO {13}

/**************Configuration end***************/

//all incuded libraries
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <SPI.h>
#include "RF24.h"
#include <OneWire.h>

/*Macros*/
#define ArrayLength(x)       (sizeof(x) / sizeof(x[0]))

//ISR Flag bits
#define RADIOFLAG 0x01
#define ISRFLAG 0x02

//global variables
uint8_t inputDevices=0;
uint8_t outputDevices=0;

#ifdef NRFRADIO
  //create the RF instance
  RF24 radio(radioCE, radioCS);
#endif

//This is a flag used by the ISR for the nrf IRQ line
volatile byte flags = 0x00;
volatile int wdtOverrun = 0;

//Create the arrays of attached periferals
#ifdef ENTRYSWITCH
  int entrySwitch[] = ENTRYSWITCH;
#endif

#ifdef ISRPINS
  int isrPins[] = ISRPINS;
#endif
#ifdef DS18B20
  OneWire temp(DS18B20);
#endif
#ifdef SIMPLEOUTHI
  int outputsHi[] = SIMPLEOUTHI;
#endif
#ifdef SIMPLEOUTLO
  int outputsLo[] = SIMPLEOUTLO;
#endif



void setup() {
  //Turn off interupts for the initialization
  cli();


  #ifdef ENTRYSWITCH
    //initialize the entry switch to trigger interupts
    pinMode(entrySwitch[0], INPUT);
    //now set the internal pullup
    digitalWrite(entrySwitch[0], HIGH);
    //Do the opposite for the other entry pin
    pinMode(entrySwitch[1], INPUT);
    //There is no internal pulldown, so this must be on the circuit
  #endif

  
  #ifdef RADIOISR
    //initialize the radio isr pin
    pinMode(RADIOISR, INPUT);
    //now set the internal pullup
    digitalWrite(RADIOISR, HIGH);
  #endif
  
  #ifdef ISRPINS
    int ISRState[ArrayLength(isrPins)];
    //Initialize all ISR pins.  All will trigger low so we will turn on the internal pullup resistor
    for(int i=0;i<ArrayLength(isrPins);i++){
      pinMode(isrPins[i],INPUT);
      digitalWrite(isrPins[i], HIGH);
    }
    inputDevices += ArrayLength(isrPins);
  #endif
  
  #ifdef DS18B20
    inputDevices += 1;
  #endif
  
  #ifdef SIMPLEOUTHI
    int outputsHiState[ArrayLength(outputsHi)];
    //Intialize all simple outputs as output pins
    //Set the initial state to high
    for(int i=0;i<ArrayLength(outputsHi);i++){
      pinMode(outputsHi[i],OUTPUT);
      digitalWrite(outputsHi[i], HIGH);
    }
    outputDevices += ArrayLength(outputsHi);
  #endif
  
  #ifdef SIMPLEOUTLO
    int outputsLoState[ArrayLength(outputsLo)];
    //Intialize all simple outputs as output pins
    //Set the initial state to low
    for(int i=0;i<ArrayLength(outputsLo);i++){
      pinMode(outputsLo[i],OUTPUT);
      digitalWrite(outputsLo[i], LOW);
    }
    outputDevices += ArrayLength(outputsLo);
  #endif
  sei();
}

void loop() {
  //Check if a flag has been raised
  while(flags>0){
    //First we check the radio
    if((flags|RADIOFLAG)>0){
      //Now we read data from the radio to set outputs

      //we will clear the radio flag since we have read the data and processed its request
      flags &= ~RADIOFLAG;
    }
    if((flags|ISRFLAG)>0){
      //We have had an interrupt request and will read all inputs and send back the data
      statusUpdate();
      
      //clear the isrflag since we are done
      flags &= ~ISRFLAG;
    }
  }
  sleep();
}

void sleep(){
  //initialize the sleep and interrupts
  cli();//turn off interrupts
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_bod_disable();
  #ifdef RADIOISR
    attachInterrupt(digitalPinToInterrupt(RADIOISR),radioISR, LOW);
  #endif
  #ifdef ISRPINS
    for(int i=0;i<ArrayLength(isrPins);i++){
      attachInterrupt(digitalPinToInterrupt(isrPins[i]),wakeUp, LOW);
    }
  #endif
  //enable the watchdog interrupt before going to sleep
  watchdogEnable();
  sei(); //turn on interrupts
  sleep_cpu();
}

void watchdogEnable(){
  MCUSR = 0;                          // reset status register flags
                                     // Put timer in interrupt-only mode:                                        
  WDTCSR |= 0b00011000;               // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
                                     // using bitwise OR assignment (leaves other bits unchanged).
  WDTCSR =  0b01000000 | 0b100001;    // set WDIE (interrupt enable...7th from left, on left side of bar)
                                     // clr WDE (reset enable...4th from left)
                                     // and set delay interval (right side of bar) to 8 seconds,
                                     // using bitwise OR operator.
  wdt_reset();
}

/**************Interrupt handlers******************/
//ISR triggered by radio
void radioISR(void){
  bool tx, fail, rx;
  //disable sleep mode
  sleep_disable();
  wdt_disable();
  detachInterrupt(digitalPin(RADIOISR));

  radio.whatHappened(tx,fail,rx);
  if(tx){
       //sent data arrived successfully
       //Do nothing
  }
  else if(rx){
    //Received data, raise the radio flag
    flags |= RADIOFLAG;
  }
  else{
    //send failed, raise the resend flag, for now we will just ignore it
    //Do nothing
  }
}

//ISR Triggered by switch
void wakeUp(){
  //disable sleep mode and the watchdog
  sleep_disable();
  wdt_disable();
  
  for(int i=0;i<ArrayLength(isrPins);i++){
    detachInterrupt(digitalPinToInterrupt(isrPins[i]));
  }
  //Set the ISR Flag to trigger reading all devices
  flags |= ISRFLAG;
}

ISR(WDT_vect){
  //disable sleep mode and watchdogtimer interrupt
  sleep_disable();
  
  if(wdtOverrun < (SLEEPTIME/8-1)){
    //Incrementing the wdtOverrun, do not raise the flag, the MCU will go back to sleep
    wdtOverrun++;
  }
  else{
    //disable the wdt
    wdt_disable();
    wdtOverrun = 0; //reset timer counter
    //set the flag for reading all devices
    flags |= ISRFLAG;
  }
}
/************End of interrupt handlers*************/




/*************Defining all the structs**************/
struct switchDef{
  uint8_t totalSwitches;
  byte switchStatus;
};

/*************Reading status functions**************/
/*This function will read all current Status data, and then send that to the base station*/
void statusUpdate(){
  #ifdef DS18B20
    float temp = getTemp();
  #endif
  #ifdef ISRPINS
    struct switchDef switchStatus = readSwitches();
  #endif

}


struct switchDef readSwitches(){
  //initialize the switchDef
  struct switchDef val;
  val.switchStatus = 0;
  val.totalSwitches = 0;
  
  #ifdef ISRPINS
    for(int i = 0; i<ArrayLength(isrPins);i++){
      val.switchStatus |= digitalRead(isrPins[i])<<val.totalSwitches;
      val.totalSwitches++;
    }
  #endif  
  return val;
}

float getTemp(OneWire temp){
  byte data[12];
  byte i;
  byte present = 0;
  int16_t rawTemp;
  
  temp.reset();
  //there is only one one-wire device connected
  temp.skip();
  temp.write(0x44, 1);

  //wait for the conversion to take place
  delay(770);
  present = temp.reset();
  temp.skip();
  temp.write(0xBE);

  //only need the first two bytes to get the temperature data
  for(i=0;i<2;i++){
    data[i] = temp.read();
  }
  rawTemp = ((data[1]<<8) | data[0]);
  return (float)rawTemp/16;
}









