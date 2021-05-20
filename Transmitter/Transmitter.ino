

/*
  Transmitter for Drone
  By Brendan Aguiar
  
  Current Version: 0.2

  Version History:
  Version 0.1 Set up Button to enable/disable system.
  Version 0.2 Set up Baud Rate and delay
  Version 0.3 Set
  
  Bugs/Issues:
  
 */
//Lbraries Included
#include <RH_ASK.h>
#include <SPI.h>
#include <TinyGPS++.h>

//Button Assignments
volatile unsigned char* port_k = (unsigned char*)0x108;
volatile unsigned char* ddr_k = (unsigned char*)0x107;
volatile unsigned char* pin_k = (unsigned char*)0x106;
unsigned char enable = 0x00;

//Transmission Assignments
volatile unsigned char* myUCSR0A = (unsigned char*)0x00C0;
volatile unsigned char* myUCSR0B = (unsigned char*)0x00C1;
volatile unsigned char* myUCSR0C = (unsigned char*)0x00C2;
volatile unsigned int* myUBRR0 = (unsigned int*)0x00C4;
RH_ASK driver;
const char *msg = "Hello World!";

//GPS Assignments
TinyGPSPlus gps;

//Main Setup
void setup() {
  set_button();
  set_baud_rate(9600);
  if (!driver.init())
    Serial.println("init failed");
}

//Main Loop
void loop() {
  button_check();
  if (enable & 0x01){
    //Serial.println("System On.");
    driver.send((uint8_t *)msg, strlen(msg));
    driver.waitPacketSent();
    delay(1000);
  }
  else{
    Serial.println("System Off.");
  }
}

//Loop Functions
void button_check()//system_enable
{
  if (!(*pin_k & 0x80))//if button pressed
  {
    for (volatile unsigned int i = 0; i < 1000; i++);//wait
    if (!(*pin_k & 0x80))//if button still pressed
    {
      enable ^= 0x01;//flips enable
      while (!(*pin_k & 0x80));
    }
  }
}

//Setup Functions
void set_button(){
  *ddr_k &= 0b01111111; //k7 port set to input (Analog 15 for elegoo MEGA 2560)
  *port_k |= 0b10000000; //Enable pullup resistor
}
void set_baud_rate(unsigned long U0baud)
{
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0 = tbaud;
}
