#include <SoftwareSerial.h>

#define SOP '<'
#define EOP '>'
#define TXpin 9
#define RXpin 8

bool started = false;
bool ended = false;

SoftwareSerial SXbee(RXpin, TXpin); //RX, TX

char inData[80];
char in;
byte index;

void setup() {
  SXbee.begin(9600);
  Serial.begin(9600);
  Serial.println("Configurando serial do XBee...");
  Serial.println("Vamos esperar...");
}

void loop() {
  while (SXBee.available() > 0)
  {
    in = SXbee.read();
    Serial.print(in);
  }
/*  while(Xbee.available() > 0)
  {
    char inChar = Xbee.read();
    if(inChar == SOP)
    {
       index = 0;
       inData[index] = '\0';
       started = true;
       ended = false;
    }
    else if(inChar == EOP)
    {
       ended = true;
       break;
    }
    else
    {
      if(index < 79)
      {
        inData[index] = inChar;
        index++;
        inData[index] = '\0';
      }
    }
  }

  // We are here either because all pending serial
  // data has been read OR because an end of
  // packet marker arrived. Which is it?

  if(started && ended)
  {
    Serial.println (inData);

    // Reset for the next packet
    started = false;
    ended = false;
    index = 0;
    inData[index] = '\0';
  }*/
  
}
