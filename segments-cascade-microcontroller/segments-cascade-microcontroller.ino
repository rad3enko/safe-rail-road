#include <SoftwareSerial.h>

/** Задать программное serial соединение мастера(откуда принимаем управляющие пакеты) */
SoftwareSerial master(5, 6); // RX, TX

/** Настройка пинов управления регистром 74HC595 */
#define DS 2
#define ST_CP 3
#define SH_CP 4

/** Число индикаторов */
byte NUMBER_OF_INDIATORS = 5;

/** Пины, к которым подключены земли семисегментных индикаторов */
byte SEG_GROUNDS[] = {9, 10, 11, 12, 8};

/** Переменная, задающая текущее отображение индикаторов. По умолчанию: индикаторы ничего не показывают */
byte stat[5] = {10, 10, 10, 10, 10};

/** Буфер принимаемого управляющего сигнала */
char masterBuffer[7] = "";
/** Итератор обработки буфера */
byte bufferIterator = 0;

/** Определяет комбинации зажигаемых светодиодов */
byte digits[12] ={0b11101111, // 0
                  0b10001100, // 1
                  0b11011011, // 2
                  0b11011110, // 3
                  0b10111100, // 4
                  0b01111110, // 5
                  0b01111111, // 6
                  0b11001100, // 7
                  0b11111111, // 8
                  0b11111110, // 9
                  0b00001000, // Пусто (точка, если подсоединена)
                  0b00011000};// -
                  
void setup() {
  pinMode(DS, OUTPUT);
  pinMode(ST_CP, OUTPUT);
  pinMode(SH_CP, OUTPUT);

  // Default: all indicators off
  for(int i=0; i<NUMBER_OF_INDIATORS; i++) {
    pinMode(SEG_GROUNDS[i], OUTPUT);
    digitalWrite(SEG_GROUNDS[i], HIGH);
  }

  master.begin(9600);
}

void loop() {
  while(true) {

    // Пока поступают пакеты управления
    while(master.available() > 0) {
      char sym = master.read();
  
      // Начало управляющего пакета, инициализируем итератор буфера нулем
      if(sym=='!') bufferIterator = 0;
  
      // Записываем принятый символ в буфер
      masterBuffer[bufferIterator++] = sym;
  
      // Конец управляющего пакета, задаем новое состояние каскаду индикаторов
      if(sym == '#'){
        for(int i=0; i<NUMBER_OF_INDIATORS; i++) {
          if(masterBuffer[i+1] == ' ') stat[i] = 10;
          else if (masterBuffer[i+1] == '-') stat[i] = 11;
          else stat[i] = masterBuffer[i+1]-'0';
        }
      }
    }

    // Производим динамическую индикацию в соответствии с текущим состоянием
    for(int i=0; i<NUMBER_OF_INDIATORS; i++) {
      printDigit(digits[stat[i]], i);
    }
  }
}

/** Реализация динамической индикации
 *  Зажигает на индикаторе с порядковым номером segment отображение цифры digit(представление байтовое см. digits[])
 *  @param digit отображаемая цифра
 *  @param segment порядковый номер индикатора */
void printDigit(byte digit, int segment) {

  // Включить землю на выбранном индикаторе
  digitalWrite(SEG_GROUNDS[segment], LOW);
  
  for(int i=0; i<=8; i++){
    byte comparingByte = 0b00000001 << 7-i;
    byte compare = digit & comparingByte;

    boolean bitStatus = compare != 0;

    // Записать число
    digitalWrite(ST_CP, HIGH); // Пишем в сдвиг
     
    digitalWrite(DS, bitStatus);
    digitalWrite(SH_CP, HIGH);
    digitalWrite(DS, LOW);
    digitalWrite(SH_CP, LOW); // Записывает в регистр сдвига
        
    digitalWrite(ST_CP, HIGH);
    digitalWrite(ST_CP, LOW);
  }
  
    delay(2);
    // Выключить землю на выбранном индикаторе
    digitalWrite(SEG_GROUNDS[segment], HIGH);
}
