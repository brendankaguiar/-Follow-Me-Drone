String inByte;
char Byte;
char buff[12][12];
float UTC = 0;//Time
float LAT = 0; //Latitude
char LATDir = 'I'; //Direction
float LON = 0; //Longitude
char LONDir = 'I'; //Direction
int fixStatus = 0; //Position fix status
int NoSVs = 0; //Number of Satelites in use
float HDop = 0; //Height Dilution of Precision
float ALT = 0; //Altitude
char ALTUnit = 'I'; //Altitude unit in meters
float Altref = 0; //Geoid Separation
char uSep = 'I'; //Units of Separation
float diffAge = 0;

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
  Serial.println(LAT);
  Serial.println(LON);
  Serial.println(ALT);
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
