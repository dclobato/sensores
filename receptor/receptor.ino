#include <SoftwareSerial.h>

#define SOP '<'
#define EOP '>'
#define TXpin 8
#define RXpin 9

bool started = false;
bool ended = false;

SoftwareSerial Console(RXpin, TXpin); //RX, TX

char inData[80];
char in;
byte index;

void setup() {
  Console.begin(9600);
  Serial.begin(9600);
  Console.println("Configurando serial do XBee...");
  Console.println("Vamos esperar...");
}

void loop() {
  while (Serial.available() > 0)
  {
    in = Serial.read();
    Console.print(in);
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
