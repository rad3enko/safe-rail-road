#include <LiquidCrystal_PCF8574.h>
//#include <SoftwareSerial.h>
#include <TinyGPS++.h>

/** @edit 1.02.2019 Отлажены пины приема/передачи: GPS, RADIO.
 *  Индикация временно реализуется на дисплее LCD 16x2 I2C (PCF8574 converter)
 *  @bugfix 1.02.2019 Не получалось считать данные с GPS. Решение: пересадил GPS на аппаратный Serial3
 *  
 *  @edit 2.02.2019 Зашиты реальные данные с хасанской ЖД, обрезанные втрое.
 */

/** Контстанты */
#define UUID 2
/** Количество узлов в маршруте */
#define SIZE 74

/** Массивы расстояний, широт и долгот узлов маршрута */
const PROGMEM float ARR_DST[SIZE] = {0.0,28.058807924430873,56.81563212585857,90.95986403868889,116.6736872376382,151.4537974993714,184.5282643959589,219.17447578992324,249.217382584965,280.25381331488575,310.2698608402168,342.87738220673197,376.1629076557915,407.12302078917116,433.7452313452894,462.68974567148973,493.05504348232176,524.4792816635858,558.8364783048576,592.6220095908801,625.2579948099884,657.3210476836033,695.4708964699379,726.1335279703844,748.4362387906335,772.954566792885,797.915987938651,819.8597967043336,842.1446231002893,868.5810643442054,892.6662412404629,916.8247526131087,942.6339681927794,970.3667820878215,999.0625434983285,1025.2979132218684,1054.6656171477975,1081.6488232265312,1107.4564032224073,1133.263938050773,1158.1157738574107,1180.7941043959017,1204.234513228908,1227.4629185159251,1248.6614586592634,1269.3495122867014,1291.841083337352,1316.6823524372403,1341.0172942706672,1365.839506307297,1385.8073465802788,1407.4365253087751,1430.9331526795224,1454.9180449820215,1481.9585964148653,1509.913376528727,1535.252983487374,1567.0884173468933,1593.5395497405664,1617.3245763699445,1646.6190788093477,1677.826393854571,1708.1201876109992,1735.5203886091797,1761.7306553749902,1788.1049446604404,1816.4021270071546,1852.2839606872928,1876.4576533925067,1906.685636507026,1938.69555584463,1969.4808737605817,1997.6868041837051,2024.274350174979};
const PROGMEM float ARR_LAT[SIZE] = {48.494525,48.494607,48.494689,48.494785,48.494856,48.49496,48.495045,48.495135,48.495209,48.495302,48.495388,48.495484,48.495587,48.495687,48.495762,48.495858,48.495944,48.496047,48.496158,48.496265,48.496379,48.496479,48.496604,48.49676,48.496871,48.496996,48.497113,48.497217,48.497317,48.497441,48.497559,48.497673,48.497787,48.497916,48.498055,48.498187,48.498333,48.498461,48.498586,48.498711,48.498825,48.498942,48.499049,48.499099,48.499039,48.498974,48.4989,48.498728,48.498611,48.498486,48.498386,48.498276,48.498151,48.49804,48.497901,48.497762,48.497645,48.497481,48.497299,48.497149,48.496985,48.49691,48.4968,48.496696,48.496596,48.496486,48.496297,48.496218,48.496151,48.496069,48.49598,48.495894,48.495808,48.495734};
const PROGMEM float ARR_LNG[SIZE] = {135.06317,135.06281,135.06244,135.062,135.061668,135.061223,135.060793,135.060343,135.059951,135.059554,135.059168,135.05875,135.058326,135.057934,135.057591,135.057226,135.056835,135.056438,135.056003,135.055574,135.055166,135.054758,135.054276,135.054619,135.054871,135.055145,135.055434,135.055687,135.055949,135.056255,135.056529,135.056808,135.057113,135.057435,135.057763,135.058058,135.05839,135.058701,135.058996,135.059291,135.059581,135.059833,135.060107,135.060413,135.060686,135.060949,135.061233,135.061448,135.061169,135.06089,135.060665,135.060423,135.060166,135.059887,135.059586,135.05927,135.058975,135.058621,135.058852,135.059082,135.059393,135.059801,135.060177,135.060514,135.060836,135.061153,135.06141,135.061882,135.062194,135.062585,135.062998,135.063395,135.063755,135.064098};

/** Настройка пинов коммуникаций */
/** GPS */
//static const int RX_GPS = 2, TX_GPS = 3;
static const uint32_t GPS_BAUD = 9600;
/** Радио */
//static const int RX_RADIO = 4, TX_RADIO = 5;
static const uint32_t RADIO_BAUD = 1200;
/** Модуль управления индикаторами */
//static const int RX_SEGMENTS = 6, TX_SEGMENTS = 7;
//static const uint32_t SEGMENTS_BAUD = 9600;

/** Буфер хранения сообщения от передатчика */
char message[32] = ""; 
int messageIterator = 0;

unsigned long distBetw;
unsigned long rawDist;

int messageNode = 0;
int myNode = 0;

/** Буфер сообщения, передаваемого на модуль управления семисегментными индикаторами */
//char segmentsMessage[7] = "";

/** Открываем serial соединения */
//SoftwareSerial radio(RX_RADIO, TX_RADIO); // TX не используется
//SoftwareSerial gpsSerial(RX_GPS, TX_GPS); // TX не используется
//SoftwareSerial segments(RX_SEGMENTS, TX_SEGMENTS); // RX не используется

/** Объект для работы с модулем GPS */
TinyGPSPlus gps;

/** Объект для работы с LCD */
LiquidCrystal_PCF8574 lcd(0x27);

boolean isGpsOk = false;
boolean isRadioOk = false;

/** Вспомогательные функции */
int findNearestNode(float lat1, float lng1);
String getUuid(char *msg);
float getLat(char *msg);
float getLng(char *msg);
unsigned long getDistanceBetweenNodes(int index1, int index2);
static void smartListenGps(unsigned long ms);
static void smartListenRadio(unsigned long ms);
static void lcdPrint();

void setup() {
  /** Открываем соединения по радио, GPS */
//  segments.begin(SEGMENTS_BAUD);

  /** Serial1 = radio */
  Serial1.begin(RADIO_BAUD);

  /** Serial3 = GPS */
  Serial3.begin(GPS_BAUD);

  /** DEBUG */
  Serial.begin(9600);

  /** Инициализация дисплея */
  lcd.begin(16, 2);
  lcd.home();
  lcd.setBacklight(255);
}

void loop() {
  while(1) {
    smartListenGps(1000);
    smartListenRadio(1500);
    lcdPrint();
  }
}



/** Возвращает ближайший узел с переданным широте и долготе.
 *  Использует массивы координат маршрута.
 *  @param lat1 широта
 *  @param lng1 долгота
 *  @return ближайший узел */
int findNearestNode(float lat1, float lng1){
  int minIndex = 0;
  float minDiff = 9999999;
  for(int i=0; i<SIZE; i++){
    float diff = fabs(lat1 - pgm_read_float_near(ARR_LAT + i)) + 
                 fabs(lng1 - pgm_read_float_near(ARR_LNG + i));
                 
    if(diff < minDiff){
      minIndex = i;
      minDiff = diff;
    }
  }
  return minIndex;
}

/** @param index1 индекс первого узла
 *  @param index2 индекс второго узла
 *  @return дистанция между узлами */
unsigned long getDistanceBetweenNodes(int index1, int index2){
  return abs(pgm_read_float_near(ARR_DST+index2) - pgm_read_float_near(ARR_DST+index1));
}

/** Достает UUID из сообщения */
String getUuid(char *msg){
  return msg[2];
}

/** Достает широту из сообщения */
float getLat(char *msg){
  // 5-12
  float latf = 0;
  char lat1[8];
  for(int i=0; i<8; i++){ lat1[i] = msg[5+i]; }

  latf += 10      * (lat1[0]-'0');
  latf += 1       * (lat1[1]-'0');
  latf += 0.1     * (lat1[3]-'0');
  latf += 0.01    * (lat1[4]-'0');
  latf += 0.001   * (lat1[5]-'0');
  latf += 0.0001  * (lat1[6]-'0');
  latf += 0.00001 * (lat1[7]-'0');
  
  return latf;
}

/** Достает долготу из сообщения */
float getLng(char *msg){
  // 15-22
  char lng1[8];
  float lngf = 0;

  for(int i=0; i<8; i++){ lng1[i] = msg[14+i];}
  
  lngf += 100    * (lng1[0]-'0');
  lngf += 10     * (lng1[1]-'0');
  lngf += 1      * (lng1[2]-'0');
  lngf += 0.1    * (lng1[4]-'0');
  lngf += 0.01   * (lng1[5]-'0');
  lngf += 0.001  * (lng1[6]-'0');
  lngf += 0.0001 * (lng1[7]-'0');
  return lngf;
}

static void smartListenGps(unsigned long ms) {
    unsigned long start = millis();
    isGpsOk = false;
    
    do {
      while (Serial3.available()) {
        gps.encode(Serial3.read());
        
        if(gps.satellites.value() == 0 || !gps.location.isValid()) {
           isGpsOk = false;
        } else {
           isGpsOk = true;
        }
      }
    } while (millis() - start < ms);
}

static void smartListenRadio(unsigned long ms) {
  unsigned long start = millis();
  isRadioOk = false;
  
  do {
    while(Serial1.available()){
      
    char letter = Serial1.read();

    /** Начало пакета */
    if(letter=='S') messageIterator = 0;

    /** Вносим в буфер пришедший символ, инкреминируем итератор */
    message[messageIterator++] = letter;

    /** Конец пакета, проводим расшифровку */
    if(letter=='E'){ 
      String msg_uuid = getUuid(message);
      float  msg_lat  = getLat(message);
      float  msg_lng  = getLng(message);

      messageNode = findNearestNode(msg_lat, msg_lng);
      myNode = findNearestNode(gps.location.lat(), gps.location.lng());

      /** DEBUG */
      /** Выводит в Serial монитор координаты из принятого пакета и свои текущие. */
      Serial.print("Message coordinates: ");
      Serial.print(msg_lat, 5);
      Serial.print(", ");
      Serial.println(msg_lng, 5);

      Serial.print("Receiver coordinates: ");
      Serial.print(gps.location.lat(), 5);
      Serial.print(", ");
      Serial.println(gps.location.lng(), 5);
      /** DEBUG endregion */


      /** Получаем дистанцию между вычисленными узлами */
      distBetw = getDistanceBetweenNodes(messageNode, myNode);

      /** Расчет дистанции напрямую */
      rawDist =
        (unsigned long)TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          msg_lat, 
          msg_lng);
          
      isRadioOk = true;
    }
  }
  } while (millis() - start < ms);
}

static void lcdPrint() {
  lcd.clear();
  lcd.home();
  /** Отображаем состояние на экран */
  if (!(isGpsOk && isRadioOk)) {
    
    /** Что-то не так */
    lcd.print("GPS:");
    isGpsOk ? lcd.print("+") : lcd.print("-");

    lcd.setCursor(0, 1);
    lcd.print("RADIO:");
    isRadioOk ? lcd.print("+") : lcd.print("-");
  } else {
    
    /** Все ок, можно светить дистанцию */
    lcd.print("Line: ");
    lcd.print(rawDist);
    lcd.print("m");

    lcd.setCursor(0, 1);
    lcd.print("Path: ");
    lcd.print(distBetw);
    lcd.print("m");

    /** В первой строке выведем ноду передатчика */
    lcd.setCursor(13, 0);
    lcd.print(messageNode);

    /** Во второй строке выведем ноду приемника */
    lcd.setCursor(13, 1);
    lcd.print(myNode);
  }
}
