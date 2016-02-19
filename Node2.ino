#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_BMP085_U.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <XBee.h>
#include <FuncoesSensores.h>
#include <avr/sleep.h>
#include <pgmspace.h>

#define DEBUG

#define NODEiD "CTD002"               //"SP1234"

///////////////////////////////////////////////////
//                                               //
// Hardware: Arduino Mega2560                    //
// Sensores: Luminosidade e infravermelho        //
//                                               //
///////////////////////////////////////////////////
   
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

XBee radio = XBee();

Adafruit_GPS GPS(&Serial3);

// Endereco do radio XBee configurado como coordenador da rede (etiqueta 1)
XBeeAddress64 destino = XBeeAddress64(0x0013a200, 0x40c6740d);

char luxS[8];
char latS[9];
char lonS[9];
char altS[7];
char toSend[50];
char timeStamp[13];
int med=0;

void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("----------------------------");
  Serial.println("DADOS DO SENSOR");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  //Serial.println("------------------------------------");
  Serial.println("");
  delay(2000);
}

void configureSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */  
  //Serial.println("------------------------------------");
  Serial.println("CONFIGURACOES DO SENSOR");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("----------------------------\n");
  delay(2000);
}

/////////////////////////////////////////////////////////////////////// Funcoes do GPS

boolean usingInterrupt = false;

void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
//////////////////////////////////////////////////////////////////////// Funcoes do GPS

void setup(void) 
{
  ZBTxStatusResponse txStatus;  
  Serial.begin(115200);   //Serial.println("Light Sensor Test"); Serial.println("");
  radio.setSerial(Serial2);
  Serial2.begin(9600);
#ifdef DEBUG
  Serial.println("::::::::::::::::::::::[ Inicializando No2 ]:::::::::::::::::::::::::\n\n");
#endif
  createMessage (toSend, 50, "ON", NODEiD, 0);
  txStatus = sendToNode(radio, destino, toSend);
  Serial.println("Mensagem de ON enviada com identificacao do no ligado");
  Serial.println("-------------------------------------------------  ON\n");
  delay(1000);
  /* Initialise the sensor */
  if(!tsl.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.print(" Ooops, TSL2561 não detectado ... Checar conexao!");
    while(1);
  }
  displaySensorDetails();
  configureSensor();
  
  /* We're ready to go! 
  Serial.println("");
  Serial.println("Passou pelo display Sensor Details e configure Sensor");
  delay(3000);*/

  // 1 1/2. Liga o GPS
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
#ifdef DEBUG
  Serial.print("Ligando GPS... ");  
#endif
  GPS.begin(9600);
#ifdef DEBUG
  Serial.println("Baudrate ok... ");
  Serial.println("Aguardando 30 segundos de Warm/Start could do GPS...");
  delay(10000);
#endif
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
#ifdef DEBUG
  Serial.print("Comandos ok... ");
#endif
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
#ifdef DEBUG
  Serial.println("NMEA update rate ok...\n");
#endif
  // Request updates on antenna status, comment out to keep quiet
  // GPS.sendCommand(PGCMD_ANTENNA);
  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  
  //GPS.wakeup();
  useInterrupt(true);
  Serial.print("Iniciando...");
  Serial.print("Aguardando fix do GPS...");

 /* // 2. Espera pelo fix do GPS
#ifdef DEBUG
              Serial.print("LastNMEA do GPS:");
              Serial.println(GPS.lastNMEA());
              Serial.print("NMEAreceived do GPS:");
              Serial.println(GPS.newNMEAreceived());
              Serial.println("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^  Primeiro print do GPS");
              delay (3000);
              Serial.println("Iniciando...");
              Serial.print("Aguardando fix do GPS...");
#endif
*/
 int letsWaitMore = 60;
  while ((int)GPS.fix == 0)
  {
#ifdef DEBUG
    Serial.println("."); delay(1000);
    Serial.println(":"); delay(1000);
/*    
     for(i=0;i<letsWaitMore;i++)
      {
        Serial.println (i);
        //Serial.println(".");
        //Serial.print("letsWaitMore:");
        //Serial.println (letsWaitMore);
        //Serial.println(".\n");
        delay(1000);
      }
*/
#endif
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA()))
        continue;  // we can fail to parse a sentence in which case we should just wait for another
    } else Serial.println("GPS nao esta mandando nada...");
    
    getTimeStamp (GPS, timeStamp);

              Serial.println("Passou pelo if de parse do GPS\n");
              Serial.print("TimeStamp do GPS:");
              Serial.println(timeStamp);
              Serial.print("LastNMEA do GPS:");
              Serial.println(GPS.lastNMEA());
              Serial.print("NMEAreceived do GPS:");
              Serial.println(GPS.newNMEAreceived());
              Serial.println("----------------------------------------------------------  GPS.parse\n");
              delay(3000);
              
    createMessage (toSend, 50, "NF", NODEiD, 2, timeStamp, letsWaitMore);
    txStatus = sendToNode(radio, destino, toSend);
    
    Serial.println("-----------------------------------------------------------  NOT FIX!\n");
/*        Serial.print("TimeStamp:");
          Serial.println (timeStamp);
          Serial.print("letsWaitMore:");
          Serial.println (letsWaitMore);
          delay(2000);                  */

    if (letsWaitMore == 0)
    {
      createMessage (toSend, 50, "HF", NODEiD, 2, timeStamp, letsWaitMore);
      txStatus = sendToNode(radio, destino, toSend);
#ifdef DEBUG
      Serial.println(" Nao foi possivel conseguir fix do GPS. Halt!");
#endif
      cli();
      sleep_enable();
      sleep_cpu();
      while(1);
    }
    letsWaitMore--;
              Serial.println("Decrementou 1 no letsWaitMore agora:");
              Serial.println(".");
              Serial.print("TimeStamp:");
              Serial.println (timeStamp);
              Serial.print("letsWaitMore:");
              Serial.println (letsWaitMore);
              Serial.print("LastNMEA do GPS:");
              Serial.println(GPS.lastNMEA());
              Serial.print("NMEAreceived do GPS:");
              Serial.println(GPS.newNMEAreceived());
              Serial.println("---------------------------------------------------------  HF\n");
              delay(3000);
  }
  // 3. Envia, pelo Xbee, um "Alo, estou vivo" com coordenadas, timestamp e lista de sensores disponiveis
  if (GPS.fix) {
    getTimeStamp (GPS, timeStamp);
    dtostrf(GPS.latitudeDegrees, 6, 4, latS);
    dtostrf(GPS.longitudeDegrees, 6, 4, lonS);
    dtostrf(GPS.altitude, 3, 1, altS);
    
    //////////teste
              Serial.println("Aqui entrou no if do GPS FIX");
              Serial.println("Latitude, Longitude e Altitude obtidos agora!\n\n");

      Serial.print("-----------------------------------------------------------------------------------");       
        Serial.print("LastNMEA do GPS:");
        Serial.println(GPS.lastNMEA());
        Serial.print("NMEAreceived do GPS:");
        Serial.println(GPS.newNMEAreceived());
        Serial.print("TimeStamp:");
        Serial.println (timeStamp);
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("-----------------------------------------------------------------------------------");
      delay(13000);


    createMessage (toSend, 50, "UP", NODEiD, 4, timeStamp, latS, lonS, altS);
    txStatus = sendToNode(radio, destino, toSend);
    Serial.println("UP\n");
                                                  Serial.println(".\n");
                                                  Serial.print("letsWaitMore:");
                                                  Serial.println (letsWaitMore);
                                                  Serial.println(".\n");
                                                  Serial.print("LastNMEA do GPS:");
                                                  Serial.println(GPS.lastNMEA());
                                                  Serial.print("NMEAreceived do GPS:");
                                                  Serial.println(GPS.newNMEAreceived());
                                                  Serial.println(".\n");
                                                  Serial.println(".\n");
                                                  Serial.println(".\n");

                                                  
          Serial.println("-------------------------------------------  UP com coordenadas, timestamp e lista de sensores disponiveis\n");
          Serial.println("A partir do SETUP:");
          Serial.println(".");
          Serial.print("letsWaitMore:");
          Serial.println (letsWaitMore);
          Serial.print("TimeStamp:");
          Serial.println (timeStamp);
          Serial.print("Latitude:");
          Serial.println(latS);
          Serial.print("Longitude:");
          Serial.println(lonS);
          Serial.print("Altitude:");
          Serial.println(altS);
    Serial.println("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^  UP\n");



    createMessage (toSend, 50, "SL", NODEiD, 2, timeStamp, "L");
    txStatus = sendToNode(radio, destino, toSend);
    Serial.println("-------------------------------------  SL\n");
    Serial.println("Enviada mensagem com lista de sensores disponiveis...");
  }

// Envia, pelo Xbee, um "Alo, estou pronto para coletar" com timestamp

#ifdef DEBUG
  Serial.println("Inicializando sensores...");
#endif

  Serial.println("Terminou SETUP\n\n\n");
  delay(3000);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void) 
{
   ZBTxStatusResponse txStatus;  
  /* Get a new sensor event */ 
  sensors_event_t event;
  tsl.getEvent(&event);

////////////////////
  // Atualiza o timestamp a partir do GPS
  // Liga o GPS
  useInterrupt(true);
  while (!GPS.newNMEAreceived());
  GPS.parse(GPS.lastNMEA());
  getTimeStamp (GPS, timeStamp);
    // Desliga o GPS
    // GPS.standby();
    // useInterrupt(false);
                                  
  /* Display the results (light is measured in lux) */
  if (event.light)
  {
    dtostrf(event.light,5,1,luxS);
    //Serial.print(event.light); Serial.println(" lux");
  }
  else
  {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.println("Sensor overload");
  }

                                  Serial.println(".........................................................................Inicio do LOOP:\n");
                                  Serial.print("TimeStamp:");
                                  Serial.println (timeStamp);
                                  Serial.print("Latitude:");
                                  Serial.println(latS);
                                  Serial.print("Longitude:");
                                  Serial.println(lonS);
                                  Serial.print("Altitude:");
                                  Serial.println(altS);
                                  Serial.print("Luminosidade:");
                                  Serial.println(luxS);
                                  Serial.println("---------------------------------------------------------\n\n");


  createMessage (toSend, 50, "DT", NODEiD, 2, timeStamp, luxS);
  txStatus = sendToNode(radio, destino, toSend);
  Serial.println("---------------------------------------------------------------------------------------  DT");
  Serial.println("Envianda mensagem completa (DT - Identificacao do no, timeStamp, coordenadas e luminosidade)\n\n");
  med = med+1;
  Serial.print("Medicao numero:");
  Serial.println(med);
  Serial.println("^^^^^^^^^^^^^^^\n");
  Serial.println("::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: Terminou o LOOP\n\n\n\n");
  
  delay(7654);
}
