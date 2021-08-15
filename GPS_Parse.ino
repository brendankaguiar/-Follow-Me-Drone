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
//VTG Assignments
float cogt = 0; //Course over ground (true)
char T = 'I'; //Fixed Field: Truth
float cogm = 0; //Course over ground (magnetic)
char M = 'I'; // Fixed Field: Magnetic
float sog = 0; //Speed over ground (knots)
char N = 'I'; // Fided Field: knots
float kph = 0; //Speed over ground (kph)
char K = 'I'; //Fixed Field: Kilometers per Hour
char mode = 'N'; //Mode Indicator

void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);
}

void loop() { // send data only when you receive data:
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
void parseData()
{
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
  printData(); 
}
void loadData(){
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
void printData(){
  Serial.print("Status indicator :");
  Serial.println(fixStatus);
  Serial.print("Latitude :");
  Serial.println(LAT);
  Serial.print("Longitude :");
  Serial.println(LON);
  Serial.print("Altitude :");  
  Serial.println(ALT);
}
