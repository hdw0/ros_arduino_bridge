//  Arduino Pro Mini 3.3V example code
//  for   https://github.com/p-h-a-i-l/hoverboard-firmware-hack
//  visit https://pionierland.de/hoverhack/ to compile your firmware online :-)
// Edit by Andrey Kishchenko hdw0@yandex.ru https://github.com/hdw0/ros_arduino_bridge.git
#ifdef HOVER_SERIAL
#include <SoftwareSerial.h>
SoftwareSerial oSerial(2,3); // RX, TX

typedef struct{
   int16_t steer;
   int16_t speed;
   uint32_t crc;
} Serialcommand;
Serialcommand oCmd;

typedef struct{
   int16_t iSpeedL; // 100* km/h
   int16_t iSpeedR; // 100* km/h
   uint16_t iHallSkippedL;
   uint16_t iHallSkippedR;
   uint16_t iTemp;  // °C
   uint16_t iVolt;  // 100* V
   int16_t iAmpL;  // 100* A
   int16_t iAmpR;  // 100* A
   uint32_t crc;
} SerialFeedback;
SerialFeedback oFeedback;

void hover_setup() 
{
  //Serial.begin(115200);
  //Serial.println("Hoverhack Test v1.0");
    
  oSerial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

uint32_t crc32_for_byte(uint32_t r) 
{
  for(int j = 0; j < 8; ++j)
    r = (r & 1? 0: (uint32_t)0xEDB88320L) ^ r >> 1;
  return r ^ (uint32_t)0xFF000000L;
}

void crc32(const void *data, size_t n_bytes, uint32_t* crc) {
  static uint32_t table[0x100];
  if(!*table)
    for(size_t i = 0; i < 0x100; ++i)
      table[i] = crc32_for_byte(i);
  for(size_t i = 0; i < n_bytes; ++i)
    *crc = table[(uint8_t)*crc ^ ((uint8_t*)data)[i]] ^ *crc >> 8;
}


void Hover_Send(int leftSpeed, int rightSpeed)
{
    int steer=0;
    float tmp=0;
    tmp=leftSpeed-rightSpeed;
    tmp=(float)tmp/(float)(rightSpeed+leftSpeed);
    tmp*=(float)1000.0;
    oCmd.steer = (int)tmp; ////////////
    tmp=leftSpeed+rightSpeed;
    tmp/=(float)2.0;
    
    oCmd.speed = (int)tmp; ////////////  

  uint32_t crc = 0;
  crc32((const void *)&oCmd, sizeof(Serialcommand)-4,   &crc);
  oCmd.crc = crc;
  
  oSerial.write((uint8_t *) &oCmd, sizeof(oCmd)); 
}

int iFailedRec = 0;
boolean Hover_Receive()
{ 
  if (oSerial.available()<  sizeof(SerialFeedback))
    return false;

  SerialFeedback oNew;
  byte* p = (byte*)&oNew;
  for (unsigned int i=0; i < sizeof(SerialFeedback); i++)
    *p++ = oSerial.read();;

  uint32_t crc = 0;
  crc32((const void *)&oNew, sizeof(SerialFeedback)-4,   &crc);



  if (oNew.crc == crc)
  {
    memcpy(&oFeedback,&oNew,sizeof(SerialFeedback));
    return true;    
  }


  while (oSerial.available()) oSerial.read();   // empty garbage
  //Serial.print("X");
  iFailedRec++;

  return false;
}

#define TIME_SEND 30
int iTest = 10;
unsigned long iTimeSend = 0;
/*
void loop(void)
{ 
  unsigned long iNow = millis();
  if (Receive())
  {
    if (iFailedRec)
      Serial.println();
    iFailedRec = 0;
    Serial.print("speedL: ");Serial.print(-0.01*(float)oFeedback.iSpeedL);
    Serial.print("\tspeedR: ");Serial.print(-0.01*(float)oFeedback.iSpeedR);
    Serial.print("\tskippedL: ");Serial.print(oFeedback.iHallSkippedL);
    Serial.print("\tskippedR: ");Serial.print(oFeedback.iHallSkippedR);
    Serial.print("\t°C: ");Serial.print(oFeedback.iTemp);
    Serial.print("\tU: ");Serial.print(0.01 * (float)oFeedback.iVolt);
    Serial.print("\tlA: ");Serial.print(0.01 * (float)oFeedback.iAmpL);
    Serial.print("\trA: ");Serial.println(0.01 * (float)oFeedback.iAmpR);
  //}
    if (iTimeSend > iNow) return;

  iTimeSend = iNow + TIME_SEND;
  Send(abs(iTest)-200);
 // Send(0);
  Serial.print(" ");Serial.println(abs(iTest)-200);
  iTest+= 2;
  if (iTest>400) iTest=-400;

  digitalWrite(LED_BUILTIN, (iNow%2000)<1000);
  }
  
}
*/
#endif
