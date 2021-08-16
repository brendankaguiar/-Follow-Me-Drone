 /********************START OF BEACON.INO****************************/

/*************************************************************************
***********************PROGRAM INFORMATION********************************
**************************************************************************
  Beacon Transmitter for drone
  By Brendan Aguiar

When enabled, the system will be on standby and sending touchdown instructions to the drone. 
When takeoff is enabled, the becaon will send the longitude (LON), latitude (LAT), altitude (ALT) and compass heading (alpha) 
to the drone via a transceiver. This will only happen if the GPS signal has been acquired by checking the fix (fixStatus).
If the system is put into standby or shut off, the beacon will send a touchdown command to the drone once again.
  
  Current Version: 1.3
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
  Version 1.2 Implemented takeoff and touchdown functionality
  Version 1.3 changed GPS sentence to NMEA based GPGGA and restructured syntax
  Version 1.4 Added Magnetometer for compass heading
  
  Bugs/Issues         Status:                       Version:
  GPS Module          No data on initial test.      0.9
  RF Transmitter      Not sending or receiving      1.0
  
  Board in Use:
  Elegoo Mega 2560
  
  Pins in Use:      Description:
  A15               Button Check
  *                 Transmitter Data Pin
  D10               LED Indicator
  RX0               GPS Receiving Pin
  TX0               GPS Transmitting Pin
***************************************************************************
************************ASSIGNMENTS*******************************
**************************************************************************/
//Transceiver Assignments
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
RF24 radio(9, 10); // CE, CSN         
const byte address[6] = "00001";
#define BUFF_LENGTH 12  
unsigned char msg[BUFF_LENGTH]; // Complete message

//GGA Assignments
String inByte;
char Byte;
char buff[13][12];
float UTC = 0;//Time
float LAT = 0; //Latitude
char LATDir = 'I'; //Direction
float LON = 0; //Longitude
char LONDir = 'I'; //Direction
unsigned int fixStatus; //Quality Indicator
int NoSVs = 0; //Number of Satelites in use
float HDop = 0; //Height Dilution of Precision
float ALT = 0; //Altitude
char ALTUnit = 'I'; //Altitude unit in meters
float Sep = 0; //Geoid Separation
char uSep = 'I'; //Units of Separation
float diffAge = 0; //Age of Differential Connections

//Compass Assignments
#include <Wire.h>
int x, y, z; //magnetic field strength
int alpha; //orientation of device

//Port B Assignments LED Indicators
//D10               RED LED (System)
//D11               GREEN LED (Takeoff)
volatile unsigned char* port_b = (unsigned char*)0x25;
volatile unsigned char* ddr_b = (unsigned char*)0x24;
volatile unsigned char* pin_b = (unsigned char*)0x23;

//Port K Assignments Buttons
//A14                (system)
//A15                (takeoff)
volatile unsigned char* port_k = (unsigned char*)0x108;
volatile unsigned char* ddr_k = (unsigned char*)0x107;
volatile unsigned char* pin_k = (unsigned char*)0x106;
unsigned char system_enable = 0x00;
unsigned char takeoff_enable = 0x00;
unsigned char sys = 0x80;//system port
unsigned char takeoff = 0x40;//takeoff port

void setup() {
  *ddr_b |= 0b00110000; //b3, b4 port set to Output (Digital 11, 10 for elegoo MEGA 2560)
  setup_buttons();
  //setup_compass();
  setup_transceiver();
  Serial1.begin(9600);//Setup GPS
}
void loop() {
  button_check(sys);
  if (system_enable & 0x01){
    set_LED("RED");
    button_check(takeoff);
    if (takeoff_enable & 0x01){
      set_LED("GREEN");
      //get_alpha();
      get_GPS();
      if (fixStatus == 1)
        send_takeoff();
    }
    else if (port_b == 0b00100000){//if green light is on
      send_touchdown();
    } 
  }
  else {
     set_LED("OFF");
     if (takeoff_enable & 0x01){
        send_touchdown();
        takeoff_enable ^= 0x01;
     }
  }
}
void send_takeoff() {
  msg[0] = (int)(((signed long)LON >> 24) & 0xFF);
  msg[1] = (int)(((signed long)LON >> 16) & 0xFF);
  msg[2] = (int)(((signed long)LON >> 8) & 0xFF);
  msg[3] = (int)(((signed long)LON & 0xFF));
  msg[4] = (int)(((signed long)LAT >> 24) & 0xFF);
  msg[5] = (int)(((signed long)LAT >> 16) & 0xFF);
  msg[6] = (int)(((signed long)LAT >> 8) & 0xFF);
  msg[7] = (int)(((signed long)LAT & 0xFF));  
  msg[8] = (int)(((signed long)ALT >> 24) & 0xFF);
  msg[9] = (int)(((signed long)ALT >> 16) & 0xFF);
  msg[10] = (int)(((signed long)ALT >> 8) & 0xFF);
  msg[11] = (int)(((signed long)ALT & 0xFF));/*
  msg[12] = (int)(((signed long)alpha >> 24) & 0xFF);
  msg[13] = (int)(((signed long)alpha >> 16) & 0xFF);
  msg[14] = (int)(((signed long)alpha >> 8) & 0xFF);
  msg[15] = (int)(((signed long)alpha & 0xFF));/*
  */
  transmit();
}
void send_touchdown() //Send touchdown message
{
  msg[0] = 0xFF;
  transmit();
}

void transmit() {
  radio.write(&msg, sizeof(msg));
  //delay(500);
}
void get_GPS() {
  if (Serial1.available()) { // read the incoming byte:
    Byte = Serial1.read();
    if (Byte == '$') { //start of new data
      inByte = Serial1.readStringUntil('\n');//read until return
      if (inByte[3] == 'G')
        parseData();//parse data into buff
    }
  }
  clearBuff();
}

void clearBuff()
{
  for (int i= 0; i < 13; i++)
    for (int j = 0; j < 12; j++)
      buff[i][j] = 0;
}
void parseData() {
  int j = 0;
  int k = 0;;
  for (int i = 6; i < inByte.length(); i++)
  {
    
    if (inByte[i] == ',')
    {
      j++;
      k = 0;
    }
    else
    {
      buff[j][k] = inByte[i];
      k++;
    }
  } 
  loadData();
}
void loadData() {
  UTC = atof(buff[0]);
  LAT = atof(buff[1]);
  LATDir = buff[2][0];
  LON = atof(buff[3]);
  LONDir = buff[4][0];
  fixStatus = atoi(buff[5]);
  NoSVs = atoi(buff[6]);
  HDop = atof(buff[7]);
  ALT = atof(buff[8]);
  ALTUnit = buff[9][0];
  Sep = atof(buff[10]);
  uSep = buff[11][0];
  diffAge = atof(buff[12]);
}
void get_alpha() {
  Wire.beginTransmission(0x1E);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  Wire.requestFrom(0x1E, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  alpha = atan2(x, y) / 0.0174532925;
  if(alpha < 0){
    alpha += 360;
    alpha = 360 - alpha;
  }
}
void setup_transceiver(){
  radio.begin();                  //Starting the Wireless communication
  radio.openWritingPipe(address); //Setting the address where we will send the data
  radio.setPALevel(RF24_PA_MIN);  //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.stopListening();          //This sets the module as transmitter
}
void setup_compass(){
  Wire.begin();
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(0x1E); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
}
void setup_buttons() {
  *ddr_k &= 0b00111111; //k7 {system}, k6 {takeoff} pins set to input (Analog 15, 14 for elegoo MEGA 2560) 
  *port_k |= 0b11000000; //Enable pullup resistors
}
void button_check(unsigned char button) {
  if (!(*pin_k & button)) { //if button pressed
    for (volatile unsigned int i = 0; i < 1000; i++);//wait
    if (!(*pin_k & button)){ //if button still pressed
      if(button == sys)
        system_enable ^= 0x01;//flips enable
      else
        takeoff_enable ^= 0x01;//flips takeoff
    }
    while (!(*pin_k & button));
  }
}
void set_LED(String color) {
  if (color == "RED"){
    *port_b |= 0b00010000;//Turn on RED
    *port_b &= !0b00100000;//Turn off GREEN 
  }
  else if (color == "GREEN"){
    *port_b |= 0b00100000;//Turn on GREEN
    *port_b &= !0b00010000;//Turn off RED
  }
  else if (color == "OFF")
     *port_b &= !0b00110000;//Turn off RED and GREEN
}
 /***********************END OF BEACON.INO***********************************/
