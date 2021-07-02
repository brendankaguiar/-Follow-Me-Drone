/*************************************************************************
***********************RECEIVER CODE***********************************
**************************************************************************

  Receiver for Drone
  By Brendan Aguiar
  
  Current Version: 0.4
  
  Version History:
  Version 0.1 Set up Set up buttons, LED, receiver, timer, and Baud Rate
  Version 0.2 Added PIOB for LED
  Version 0.3 Added Transceiver functionality
  Version 0.4 Implemented Artificial Potential Field
  Version 0.5 Started ESC and Motor implementation
  
  Bugs/Issues:
  LED, and push button set to ELEGOO Mega 2560 specs. Convert to DUE specs in upcoming version.

  Board in Use:
  Arduino DUE
  
  Pins in Use:      Description:
  D12               IR Data Pin
  D11               Receiver Data Pin
  D13               LED Indicator
  D3                GPS Receiving Pin (RX)
  D4                GPS Transmitting Pin (TX)
 
***************************************************************************
***************************************************************************
**************************************************************************/

//Libraries Included
#include <SPI.h> //for Transceiver
#include <nRF24L01.h> //for Transceiver
#include <RF24.h> //for Transceiver
#include <Wire.h> //for gyroscope
#include <Servo.h> //for ESC and Motor
//PIOB Assignments for LED
volatile unsigned long* PIO_WPMR = (unsigned long*)0x400E10E4;//PIO Write Protect Mode
volatile unsigned long* PIO_OER = (unsigned long*)0x400E1010;//Output Enable
volatile unsigned long* PIO_SODR = (unsigned long*)0x400E1030;//Set Output Data
volatile unsigned long* PIO_CODR = (unsigned long*)0x400E1034;//Clear Output Data
volatile unsigned long* PIO_ODSR = (unsigned long*)0x400E1038;//Output Status

//Messenger Assignments
#define BUFFER_LENGTH 24 //in unsigned chars
#define BUFFER_SIZE 192 //in Bytes
unsigned char msg[BUFFER_LENGTH]; // Complete message

//Artificial Potential Field Assignments
signed long qdb[3]; //relative states
signed long pdb[3]; //relative velocities
signed long phi;
long gamma_2 = 0;
long velocity = 0;
long theta_b = 0;
long theta_d = 0;
long lambda = 1;
long rHoz;
#define VMAX 550 //cm/s
//Gyroscope Assignments

//ESC and Motor assignments
Servo ESC_CCW1;//Front right
Servo ESC_CCW2; //Back Left
Servo ESC_CW1; //Front left
Servo ESC_CW2; //Back right

//PID Assignments
float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

//UART Register Assignments
volatile unsigned long* UART_CR = (unsigned long*)0x400E0800; //Control Register (Write-Only)
volatile unsigned long* UART_MR = (unsigned long*)0x400E0804; //Mode Register (Read-Write)
volatile unsigned long* UART_SR = (unsigned long*)0x400E0814; //Status Register (Read-Only)
volatile unsigned long* UART_RHR = (unsigned long*)0x400E0818; //Receiver Holding Register (Read-Only)
volatile unsigned long* UART_THR = (unsigned long*)0x400E081C; //Transmitter Holding Register (Write-Only)
volatile unsigned long* UART_BRGR = (unsigned long*)0x400E0820; //Baud Rate Generator Register (Read-Write)
unsigned long MCK = 84000000; //Clock speed in Hz

//Receiver Assignments
RF24 radio(7,8);// CNS, CE
const byte address[6] = "ALPHA";//Communication line 1
signed long pos[3]; //geocentric coordinates of beacon [0] = X, [1] = Y, [2] = Z (cm)
signed long vof[3];  //velocity of beacon [0] = X, [1] = Y, [2] = Z (m/s)

//Real-Time timer Register Assignments
volatile unsigned long* RTT_MR = (unsigned long*)0x400E1A30; // mode register
volatile unsigned long* RTT_AR = (unsigned long*)0x400E1A34;// alarm register
volatile unsigned long* RTT_VR = (unsigned long*)0x400E1A38; //value register
volatile unsigned long* RTT_SR = (unsigned long*)0x400E1A3C;//read/write , set if match alarm


/********************************GPS Structure (Drone)**********************************/
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

  while ((*UART_SR & 0x00000001) == 0) { //wait for byte to be read
    byte c =  *UART_RHR; //read byte from receiver holding register
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

/*******************************************************************************
************************************Setup and Loop******************************
*******************************************************************************/

void setup()
{
  set_LED();
  UART_init(9600); //initialize UART
  set_receiver();
}

void loop()
{
  receive();
  if (msg[0] == 0xF0)//touchdown
  {
    *PIO_CODR = 0x08000000;//Turn off LED
    /*power off motors if on*/
  }
  else if (msg[0] == 0xFF)//Takeoff
  {
    *PIO_SODR = 0x08000000;//Turn on LED
    /*Start Motors and PID*/
  }
  else //Follow Me
  {
    if (processGPS())
    {
      APF();//Artificial Potential Field 
    }
  }
}

//Loop Functions
void APF()//The artificial Potential Field updates the desired velocity and heading
{
  set_relatives();
  set_headings();
  update_velocity();
  update_heading();    
}
void receive()
{
  if (radio.available())
  {
    radio.read(&msg, sizeof(msg));
  }
  if (sizeof(msg) == 8)
  {
    if (msg[0] == 0xF0)//touchdown
    {
      /*Turn off all motors*/
    }
    else //takeoff
    {
      /*Start motors and enable PID*/
    }
  }
  else //Set coordinates and speed of beacon
  {
    for (int i = 0; i < 3; i++)
    {
      pos[i] = ( ((signed long)msg[i * 4] << 24) 
                     + ((signed long)msg[i * 4 + 1] << 16) 
                     + ((signed long)msg[i * 4 + 2] << 8) 
                     + ((signed long)msg[i * 4 + 3])); //Pos of Beacon
      vof[i] = ( ((signed long)msg[i * 4 + 12] << 24) 
                     + ((signed long)msg[i * 4 + 13] << 16) 
                     + ((signed long)msg[i * 4 + 14] << 8) 
                     + ((signed long)msg[i * 4 + 15])); //Speed of Beacon                
    }
  }
}

void set_relatives()//relative position and velocity between beacon and robot
{
  qdb[0] = pos[0] - sol.ecefX;
  qdb[1] = pos[1] - sol.ecefY;
  qdb[2] = pos[2] - sol.ecefZ; //Initial states between robot and beacon
  pdb[0] = vof[0] - sol.ecefVX;
  pdb[1] = vof[1] - sol.ecefVY;
  pdb[2] = vof[2] - sol.ecefVZ;
}

void set_headings()
{
  phi = atan2(qdb[1],qdb[0]); //Horizontal heading difference
  rHoz = sqrt(pow(qdb[1],2) + pow(qdb[0],2)); //resolved horizontal distance
  gamma_2 = atan2(qdb[2], rHoz); //Vertical heading difference
}

void update_velocity()
{
  long term1 = norm(vof);
  long term2 = 2 * lambda * norm(qdb) * term1 * abs(cos(theta_b - phi));
  long term3 = pow(norm(qdb) * lambda, 2);
  velocity = sqrt(pow(term1,2) + term2 + term3);
  if (velocity > VMAX || velocity > term1)
    velocity = term1;
}

void update_heading()
{
  long nested = pow(norm(vof),2) * sin(theta_b - phi) / velocity;
  theta_d = phi + asin(nested);
}

signed long norm(signed long val[])
{
  return sqrt(pow(val[0],2) + pow(val[1],2) + pow(val[2],2));
}
//Setup Functions
void set_receiver()
{
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void set_LED()
{
  *PIO_WPMR = 0x50494F00; // Disable Write Protection
  *PIO_OER = 0x08000000; //PB27 set to Output (Digital 13 for Arduino Due)
}

void UART_init(unsigned long clk_div)
{
  unsigned int baud_rate = MCK / (16 * clk_div); //( master clk / (16 * clk divisions));
  *UART_CR = 0x00000050; //enable receive and transmit
  *UART_MR = 0x00000800; //no parity, normal mode
  *UART_BRGR = baud_rate;//set baud rate
}

