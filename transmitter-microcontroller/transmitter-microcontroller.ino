#include <stdio.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

/** Основные константы */
#define RED_LED         8
#define GREEN_LED       9
#define TRANSMIT_DELAY  1500
#define UUID            1

/** Параметры соединений */
static const int RX_GPS = 5, TX_GPS = 3;
static const int RX_RADIO = 7, TX_RADIO = 6;
static const uint32_t GPS_BAUD = 9600;
static const uint32_t RADIO_BAUD = 1200;

/** Настройка serial соединений */
/** Радиомодем */
SoftwareSerial radio(RX_RADIO, TX_RADIO); // RX(not used), TX
/** GPS устройство */
SoftwareSerial gpsSerial(RX_GPS, TX_GPS); // RX, TX(not used)

/** Объект для работы с gps */
TinyGPSPlus gps;

/** Объявление вспомогательных функций */
void reverse(char *str, int len);
int intToStr(int x, char str[], int d);
void ftoa(float n, char *res, int afterpoint);
static void smartDelay(unsigned long ms);

void setup() {
  /** Задаем начальные значения для светодиодов индикации состояния */
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);

  /** Открываем serial соединения */
  radio.begin(RADIO_BAUD);
  gpsSerial.begin(GPS_BAUD);

  /** DEBUG */
  Serial.begin(9600);
}
void loop() {
  /** Если не улавливается ни одного спутника ИЛИ приходит невалидный пакет 
   *  с GPS устройства, то индицируем ошибку зажиганием красного светодиода.
   *  Иначе происходит мигание зеленого светодиода при отправке пакета по radio соединению. */

  /** DEBUG */
  Serial.print("Видимые спутники: ");
  Serial.println(gps.satellites.value());
  Serial.print("Состояние пакетов(false-поврежденные): ");
  Serial.println(gps.location.isValid());
  /** DEBUG */
  
  if(gps.satellites.value() == 0 || !gps.location.isValid()) digitalWrite(RED_LED, HIGH);
  else {
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    char package[40];
    char coord[10];
    char uuid[6];
    strcpy(package, "SU");
    itoa(UUID, uuid, 6);
    strcat(package, uuid);
    ftoa(gps.location.lat(), coord, 4);
    strcat(package, "LA");
    strcat(package, coord);
    ftoa(gps.location.lng(), coord, 4);
    strcat(package, "LN");
    strcat(package, coord);
    strcat(package, "E");
    Serial.println(package);
    radio.print(package);
    digitalWrite(GREEN_LED, LOW);
    gpsSerial.flush();
    //smartDelay(TRANSMIT_DELAY);
  }
  smartDelay(TRANSMIT_DELAY);
}

void reverse(char *str, int len) { 
  int i=0, j=len-1, temp; 
  while (i<j){ 
    temp = str[i]; 
    str[i] = str[j]; 
    str[j] = temp; 
    i++; j--; 
  } 
} 

int intToStr(int x, char str[], int d) 
{ 
  int i = 0; 
  if(x < 0){
    str[i++] = '-';
  }
  while (x) { 
    str[i++] = (x%10) + '0'; 
    x = x/10; 
  } 
  while (i < d) 
    str[i++] = '0'; 
  reverse(str, i); 
  str[i] = '\0'; 
  return i; 
} 

void ftoa(float n, char *res, int afterpoint) { 
  int ipart = (int)n; 
  float fpart = n - (float)ipart; 
  int i = intToStr(ipart, res, 0); 
  if (afterpoint != 0) { 
    res[i] = '.';
    fpart = fpart * pow(10, afterpoint); 
    intToStr((int)fpart, res + i + 1, afterpoint); 
  } 
}

static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}
