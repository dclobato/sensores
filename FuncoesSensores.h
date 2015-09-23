#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <XBee.h>
#include <stdarg.h>
#include <stdio.h>

#define SOP '['
#define EOP ']'
#define SEP ':'

void getTimeStamp (Adafruit_GPS, char *);
void emptyToSend();
ZBTxStatusResponse sendToNode(XBee, XBeeAddress64, char *);
void createMessage (char *, int, const char *, const char *, int, ...);

void getTimeStamp (Adafruit_GPS GPS, char *t)
{
  int i;
  for (i=0; i<12; i++)
    t[i]='\0';
  sprintf(t,"%02d%02d%02d%02d%02d%02d", GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);
  return;
}

void emptyToSend(char *s, int n)
{
  int i;
  for (i=0; i<n; i++)
    s[i]='\0';
  return;
}

ZBTxStatusResponse sendToNode(XBee radio, XBeeAddress64 to, char *s)
{
  ZBTxRequest zbTx;
  ZBTxStatusResponse txStatus;

  zbTx = ZBTxRequest(to, (uint8_t *) s, strlen(s));
  radio.send(zbTx);
  txStatus = ZBTxStatusResponse();

  return (txStatus);
}

void createMessage (char *buffer, int maxsize, const char *type, const char *node, int num, ...)
{
    va_list argumentos;

    emptyToSend (buffer, maxsize);
    if (strcmp(type, "ON") == 0)                // Turned on
    {
        sprintf(buffer, "%cON%c%s%c", SOP, SEP,
                                      node,
                                      EOP);
    }
    if (strcmp(type, "PG") == 0)                // Ping
    {
        va_start (argumentos, num);
        char *timeStamp = va_arg (argumentos, char *);  // Timestamp
        va_end (argumentos);
        sprintf(buffer, "%cPG%c%s%c%s%c", SOP, SEP,
                                          node, SEP,
                                          timeStamp,
                                          EOP);
    }
    if (strcmp(type, "NF") == 0)                // No fix
    {
        va_start (argumentos, num);
        char *timeStamp = va_arg (argumentos, char *);  // Timestamp
        int n           = va_arg (argumentos, int);     // Quantas tentativas
        va_end (argumentos);
        sprintf(buffer, "%cNF%c%s%c%s%c%02d%c", SOP, SEP,
                                                node, SEP,
                                                timeStamp, SEP,
                                                n,
                                                EOP);
    }
    if (strcmp(type, "HF") == 0)                // Halt fix
    {
        va_start (argumentos, num);
        char *timeStamp = va_arg (argumentos, char *);  // Timestamp
        int n           = va_arg (argumentos, int);     // Quantas tentativas
        va_end (argumentos);
        sprintf(buffer, "%cHF%c%s%c%s%c%02d%c", SOP, SEP,
                                                node, SEP,
                                                timeStamp, SEP,
                                                n,
                                                EOP);
    }
    if (strcmp(type, "UP") == 0)                // Up and running
    {
        va_start (argumentos, num);
        char *timeStamp = va_arg (argumentos, char *);  // Timestamp
        char *lat       = va_arg (argumentos, char *);  // Latitude
        char *lon       = va_arg (argumentos, char *);  // Longitude
        char *alt       = va_arg (argumentos, char *);  // Altitude
        va_end (argumentos);
        sprintf(buffer, "%cUP%c%s%c%s%c%s%c%s%c%s%c", SOP, SEP,
                                                      node, SEP,
                                                      timeStamp, SEP,
                                                      lat, SEP,
                                                      lon, SEP,
                                                      alt,
                                                      EOP);
    }
    if (strcmp(type, "SL") == 0)                // List of sensors
    {
        va_start (argumentos, num);
        char *timeStamp = va_arg (argumentos, char *);  // Timestamp
        char *lista     = va_arg (argumentos, char *);  // Lista de sensores
        va_end (argumentos);
        sprintf(buffer, "%cLS%c%s%c%s%c%s%c", SOP, SEP,
                                              node, SEP,
                                              timeStamp, SEP,
                                              lista,
                                              EOP);
    }
    if (strcmp(type, "DT") == 0)                // Data message
    {
        int i;
        String saida = String (String(SOP) + "DT" + String(SEP));

        va_start (argumentos, num);
        char *timeStamp = va_arg (argumentos, char *);  // Timestamp

        saida += String (String(node) + String(SEP));
        saida += String (timeStamp);    // Timestamp
        for (i = 0; i < num-1; i++)
        {
            char *leitura = va_arg (argumentos, char *);  // Leitura do sensor i
            saida += String(String(SEP) + leitura);          // Leitura do sensor i
        }
        va_end (argumentos);
        saida += String(EOP);
        saida.toCharArray(buffer, maxsize-1);
    }
    if (strcmp(type, "HS") == 0)                // Halt on sensor error
    {
        va_start (argumentos, num);
        char *timeStamp = va_arg (argumentos, char *);  // Timestamp
        char *lista     = va_arg (argumentos, char *);  // Lista de sensores com falha
        va_end (argumentos);
        sprintf(buffer, "%cHS%c%s%c%s%c%s%c", SOP, SEP,
                                              node, SEP,
                                              timeStamp, SEP,
                                              lista,
                                              EOP);
    }

    return;
}
