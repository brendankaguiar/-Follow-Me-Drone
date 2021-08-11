//GGA Assignments
String inByte;
char buff1[12][12];
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
char buff2[8][12];
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
      inByte = Serial1.readStringUntil(13);//read until return
      if (inByte.startsWith("$GPGGA")
          parseData(buff1);//parse data into buff1 
      else if (inByte.startsWith("$GPVTG"){
        parseData(buff2);
    }
  }
  UTC = atof(buff1[0]);
  LAT = atof(buff1[1]);
  LATDir = buff1[2][0];
  LON = atof(buff1[3]);
  LONDir = buff1[4][0];
  fixStatus = atoi(buff1[5]);
  NoSVs = atoi(buff1[6]);
  HDop = atof(buff1[7]);
  ALT = atof(buff1[8]);
  ALTUnit = buff1[9][0];
  Altref = atof(buff1[10]);
  uSep = buff1[11][0];
  diffAge = atof(buff1[12]); 
  cogt =  atof(buff2[0]);; //Course over ground (true)
  T = buff2[1][0];; //Fixed Field: Truth
  cogm = atof(buff2[2]); //Course over ground (magnetic)
  M = buff2[3][0]; // Fixed Field: Magnetic
  sog = atof(buff2[4]; //Speed over ground (knots)
  N = buff2[5][0]; // Fided Field: knots
  kph = atof(buff2[6]); //Speed over ground (kph)
  K = buff2[7][0]; //Fixed Field: Kilometers per Hour
  mode = buff2[8][0]; //Mode Indicator
  Serial.println(LAT);
  Serial.println(LON);
  Serial.println(ALT);
  Serial.println(kph);
  Serial.println(cogt);
}
void parseData(char buff[][12])
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
