//GGA Assignments
String inByte;
char Byte;
char buff[12][12];
float UTC = 0;//Time
float LAT = 0; //Latitude
char LATDir = 'I'; //Direction
float LON = 0; //Longitude
char LONDir = 'I'; //Direction
int fixStatus = 0; //Quality Indicator
int NoSVs = 0; //Number of Satelites in use
float HDop = 0; //Height Dilution of Precision
float ALT = 0; //Altitude
char ALTUnit = 'I'; //Altitude unit in meters
float Altref = 0; //Geoid Separation
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

void loop() {

  // send data only when you receive data:
  if (Serial1.available()) {
    // read the incoming byte:
    Byte = Serial1.read();
    if (Byte == '$')//start of new data
    {
      inByte = Serial1.readStringUntil('\n');//read until return
      parseData();//parse data into buff
    }
  }
  //Serial.println(buff[0]);
  if (inByte[3] == 'G')//Handle GPGGA
  {
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
    Altref = atof(buff[10]);
    uSep = buff[11][0];
    diffAge = atof(buff[12]);
  }
  else if (inByte[3] == 'T')//Handle GPVTG
  {
    cogt = atof(buff[0]); //Course over ground (true)
    T = buff[1][0]; //Fixed Field: Truth
    cogm = atof(buff[2]);; //Course over ground (magnetic)
    M = buff[3][0]; // Fixed Field: Magnetic
    sog = atof(buff[4]);; //Speed over ground (knots)
    N = buff[5][0]; // Fided Field: knots
    kph = atof(buff[6]);; //Speed over ground (kph)
    K = buff[7][0]; //Fixed Field: Kilometers per Hour
    mode = buff[8][0]; //Mode Indicator
  }
  Serial.println(LAT);
  Serial.println(LON);
  Serial.println(ALT);
  Serial.println(kph);
  Serial.println(cogt);
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
}
