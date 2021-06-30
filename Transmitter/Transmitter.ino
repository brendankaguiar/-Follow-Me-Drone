/********************START OF TRANSMITTER.INO****************************/

/*************************************************************************
***********************PROGRAM INFORMATION********************************
**************************************************************************

  Transmitter for Drone
  By Brendan Aguiar
  
  Current Version: 1.1

  Version History:
  Version 0.1 Set up Button to enable/disable system
  Version 0.2 Set up USART0 and delay
  Version 0.3 Set up Transmitter
  Version 0.4 Set up GPS
  Version 0.5 Set up Message for Transmission
  Version 0.6 Set up LED on indicator
  Version 0.7 Finished Initial Solder Build
  Version 0.8 Modified LED indicator
  Version 0.9 Set up NAV-SOL structure for GPS data
  Version 1.0 Cleaned up code and began tests. LED/Button confirmed
  Version 1.1 Switched from transmitter to transceiver
  
  Bugs/Issues         Status:                       Version:
  GPS Module          No data on initial test.      0.9
  RF Transmitter      Not sending or receiving      1.0
  
  Board in Use:
  Elegoo Mega 2560
  
  Pins in Use:      Description:
  A15               Button Check
  D12               Transmitter Data Pin
  D10               LED Indicator
  RX0               GPS Receiving Pin
  TX0               GPS Transmitting Pin
***************************************************************************
************************REGISTER ASSIGNMENTS*******************************
**************************************************************************/



//Port K Assignments (Button)
volatile unsigned char* port_k = (unsigned char*)0x108;
volatile unsigned char* ddr_k = (unsigned char*)0x107;
volatile unsigned char* pin_k = (unsigned char*)0x106;
unsigned char enable = 0x00;

//Timer1 Assignments (Delay)
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

//UART0 Assignments (GPS Reception)
volatile unsigned char* myUCSR0A = (unsigned char*)0x00C0; //Control & Status Register A
volatile unsigned char* myUCSR0B = (unsigned char*)0x00C1; //Control & Status Register B
volatile unsigned char* myUCSR0C = (unsigned char*)0x00C2; //Control & Status Register C
volatile unsigned int* myUBRR0 = (unsigned int*)0x00C4; //Baud Rate Register
volatile unsigned char* myUDR0 = (unsigned char*)0x00C6; // Data Register

//Transciever Assignments
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "ALPHA";//Communication line 1
//const byte address[6] = "BRAVO";//Communication line 1

//Port B Assignments (LED Indicator)
volatile unsigned char* port_b = (unsigned char*)0x25;
volatile unsigned char* ddr_b = (unsigned char*)0x24;
volatile unsigned char* pin_b = (unsigned char*)0x23;

/********************************GPS Structure***************************************/
const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };
struct NAV_SOL {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  long fTOW;
  signed short week;
  unsigned char gpsFix;
  boolean flags;
  signed long ecefX; //cm
  signed long ecefY; //cm
  signed long ecefZ; //cm
  unsigned long pAcc; //3D Pos Accuracy Estimate in cm
  signed long ecefVX; //cm/s
  signed long ecefVY; //cm/s
  signed long ecefVZ; //cm/s
  unsigned long sAcc; //Speed Accuracy Estimate in cm
  unsigned short pDOP; //Position DOP
  unsigned char reserved1; //Reserved
  unsigned char numSV; //number of SVs used in Nav Soln.
  unsigned long reserved2; //Reserved
};
NAV_SOL sol; 

//GPS Methods
void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_SOL); i++) {
    CK[0] += ((unsigned char*)(&sol))[i];
    CK[1] += CK[0];
  }
}

bool processGPS() {
  static int fpos = 0; //field position
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_SOL); //52 bytes

  while (U0kbhit() == 0) {
    byte c = U0getchar();
    if (fpos < 2) {
      if (c == UBX_HEADER[fpos])
        fpos++;
      else
        fpos = 0;
    }
    else {
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&sol))[fpos-2] = c;

      fpos++;

      if ( fpos == (payloadSize+2) ) {
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize+3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize+4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          return true;
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        fpos = 0;
      }
    }
  }
  return false;
}

//Messenger Assignments
String posX;
String posY; 
String posZ;
String VofX;
String VofY;
String VofZ;
char pDOP;//Accuracy of GPS 9 and higher is considered weak
#define BUFFER_LENGTH 36 //in Characters
const char msg[BUFFER_LENGTH]; // Complete message
bool sendData = true; // Set to true to continuously transmit data
#define BUFFER_SIZE 288 //in Bytes


/*******************************************************************************
************************************Setup and Loop******************************
*******************************************************************************/
void setup()
{
  set_LED();
  USART0init(9600); //Enable USART0 in asynchronous mode for GPS Communication
  set_button();
  set_transceiver();
  set_message();
}

void loop()
{
  button_check();
  if (enable & 0x01)
  {
    *port_b |= 0b00010000;//Turn on LED
    set_data();
    transmit();
    my_delay(1000);
  }
  else
  {
    *port_b &= !0b00010000;//Turn off LED
  }
}

/*****************************Loop Functions****************************************/
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

void set_data()//Max message size should be no more than 67 bytes
{
  if (processGPS())//updates message for pos. and velocity
  {    
    posX = String(sol.ecefX);
    posY = String(sol.ecefY);
    posZ = String(sol.ecefZ);
    VofX = String(sol.ecefVX);
    VofY = String(sol.ecefVY);
    VofZ = String(sol.ecefVZ);
    msg.concat(posX); //appending complete message
    msg.concat(posY);
    msg.concat(posZ);
    msg.concat(VofX); //appending complete message
    msg.concat(VofY);
    msg.concat(VofZ);
  }
  if (msg.length() != BUFFER_LENGTH || sizeof(msg) != BUFFER_SIZE)
    indicate(2); //Concatenated Messages are of incorrect length
}

void transmit()
{
  radio.write(&msg, sizeof(msg));
}

void my_delay(unsigned int freq)
{
  double period = 1.0/double(freq); //calc period
  double half_period = period/ 2.0f; //50% duty cycle  
  double clk_period = 0.0000000625; //clock period def
  unsigned int ticks = half_period / clk_period; //calc ticks
  *myTCCR1B &= 0xF8; //stop the timer
  *myTCNT1 = (unsigned int) (65536 - ticks); //set the counts
  * myTCCR1B |= 0b00000001; //start the timer
  while((*myTIFR1 & 0x01)==0); //wait for overflow
  *myTCCR1B &= 0xF8; //stop the timer
  *myTIFR1 |= 0x01; //reset TOV          
}

unsigned char U0kbhit() // Read USART0 RDA status bit and return non-zero true if set
{
  return *myUCSR0A & 0x80;
}

unsigned char U0getchar()
{
  return *myUDR0;
}


/***************************Setup Functions****************************************/
void set_button()
{
  *ddr_k &= 0b01111111; //k7 port set to input (Analog 15 for elegoo MEGA 2560)
  *port_k |= 0b10000000; //Enable pullup resistor
}

void set_LED()
{
  *ddr_b |= 0b00010000; //b4 port set to Output (Digital 10 for elegoo MEGA 2560)
}

void USART0init(unsigned long U0baud)
{
  unsigned long FCPU = 16000000; //set frequency olscillation
  unsigned int tbaud;
  tbaud = (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20; //
  *myUCSR0B = 0x18; //Enable Receive and Transmit
  *myUCSR0C = 0x06; //Set to asynchronous mode
  *myUBRR0 = tbaud; //Set Baud Rate
}

void set_transceiver()
{
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void set_message()
{//reserve buffer space for message
  posX.reserve(64);
  posY.reserve(64);
  posZ.reserve(64);
  VofX.reserve(32);
  VofY.reserve(32);
  VofZ.reserve(32);
  msg.reserve(BUFFER_SIZE);
}

/*****************************Error Function****************************************/
void indicate(unsigned int err_num)//Flashes LED n times to indicate error number
{
  *port_b &= !0b00010000;//Turn off LED
  my_delay(2000);
  for (unsigned int i = 0; i < err_num; i++)
  {
    *port_b |= 0b00010000;//Turn on LED
    my_delay(1000);
    *port_b &= !0b00010000;//Turn off LED
    my_delay(1000);
  }
  enable ^= 0x01;//flips enable to turn off system
}
/* Error #      Description:
 * 1            Driver failed to initialize
 * 2            Concatenated Messages are of incorrect length
 *
 */

 /***********************END OF TRANSMITTER.INO***********************************/
