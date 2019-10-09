/*
     File: WatchDog_Envilog.ino
     Description: WatchDog Envilog DLM-1470
     Created on 06.09.2018
     Copyright: CC Attribution
     Author: Telegin V.S. Polyarnie Zori KOLNPP
     Email: web.skorpio@gmail.com
*/
#include <Wire.h>                           // Библиотека для работы с I2C
#include <stdio.h> 
#include <Eeprom24C32_64.h>                 // EEPROM 24C64

//  ------------------------------------------Define----------------------------------------------

#define EEPROM_ADDRESS  0x50                // Адресс EEPROM 24C64         
#define EEPROM_ADDRESS_COUNT_OK_RESET  0x03 // Адресс памяти EEPROM с количеством удачных перезагрузок поста
#define EEPROM_ADDRESS_COUNT_RESET  0x07    // Адресс памяти EEPROM с количеством попыток перезапуска поста               
#define INTERVAL_LED_BLINK 500              // Интнрвал мигания светодиода
#define LED_GREEN 6                         // PD7 - ATmega8A - Led Green 
#define LED_RED 7                           // PD6 - ATmega8A - Led Red
#define IMPULSE 3                           // PD3 - ATmega8A - выход на GPIO Envilog
                                            // Pull-UP(res 1.5k) сигнал по pull-down.
#define RELAY 5                             // PD5 - ATmega8A - управление реле.

//  ----------------------------------------Переменные---------------------------------------------

//static Eeprom24C32_64 eeprom(EEPROM_ADDRESS);
const int tryReset = 5;                     // Количество попыток перезагрузить пост
const uint32_t timeWatchdog = 90000;        // Таймер Watchdog (при достижении максиума reset)
const uint32_t timeWaitReset = 90000;       // Время отведенное на перезагрузку поста
const uint16_t resetTime = 2000;            // Время задержки реле при reset
uint32_t currentTime, loopTime;
uint32_t blinkTime, startTime, resTim;
int cnt = 0;                                // Счетчик попыток перезапуска
int cntErr = 0;
int flagImpulse = 0;
int blinkTimeFlag = 0;
bool errorReset = false;
bool startSys = false;
bool firstStartSys = false;
uint32_t countOkReset ;                     // Общее количество перезагрузок поста
uint32_t countReset;                        // Общее количество попыток перезагрузок поста
//byte inputDate[4];
//byte outputBytes[15];

//  -------------------------------------Setup при старте-----------------------------------------

void setup() {
  //Serial.begin(9600);                     // Инициализация Serial Port
  //eeprom.initialize();                    // Инициализация EEPROM 24C64

  DDRD &= ~(1 << IMPULSE);                  // Уствновка настроек порта Status
  DDRD |= 1 << RELAY;                       // Установка прота Relay
  PORTD &= ~(1 << RELAY);                   // отключаем реле после инициализации(на всякий случай).
  DDRD |= 1 << LED_GREEN;                   // Зеленый светодиод - индикатор работы
  PORTD &= ~(1 << LED_GREEN);
  DDRD |= 1 << LED_RED;                     // Красный светодиод - индикатор ошибки
  PORTD &= ~(1 << LED_RED);
  currentTime = millis();
  startTime = millis();





  /* Что будем писать в EEPROM?
     0. Дата паследнего снятия лога          - 0x01-0x03
     1. Общее количество перезапусков поста  - 0x04-0x07       // countOkReset
     2. Количество попыток перезапуска поста - 0x08-0x0B       // countReset
  */

  //  eeprom.writeBytes(0X01, 0x03, inputDate);
  //  eeprom.writeBytes(0x04, 0x07, countOkReset);
  //  eeprom.writeBytes(0x08, 0x0B, countReset);

}

//  ----------------------------------Старт основной программы------------------------------------

void loop() {
  // Ждем загрузки Envilog заданное время.
  if (firstStartSys != true && cnt < tryReset) {
    // Если это не первая попытка запуска дергаем реле
    if (cnt > 0) {
      
      PORTD |= 1 << RELAY;
      delay(resetTime);
      PORTD &= ~(1 << RELAY);
      startTime = millis();
    }
    while (millis() <= startTime + timeWaitReset) {
      // Если система загрузилась ставим флаг
      if (!(PIND & (1 << IMPULSE))) {
        flagImpulse = 1;
      }
      if (PIND & (1 << IMPULSE) && flagImpulse == 1) {
        if (PIND & (1 << IMPULSE)) {
          flagImpulse = 0;
        }
        startSys = true;
        firstStartSys = true;
        cnt = 0;
        resTim = timeWatchdog + millis();
        PORTD &= ~(1 << LED_GREEN);
        break;
      }

      if (firstStartSys != true) {
        if (blinkTimeFlag == 0) {
          blinkTime = millis();
          blinkTimeFlag = 1;
        }

        if (millis() >= blinkTime + INTERVAL_LED_BLINK) {
          PORTD |= 1 << LED_GREEN;
          if (millis() >= blinkTime + INTERVAL_LED_BLINK * 2) {
            PORTD &= ~(1 << LED_GREEN); blinkTimeFlag = 0;
          }
        }
      }
    }
    if (firstStartSys != true) {
      cnt++;
    }
  }

  //  -------------------------------------------||-----------------------------------------------

  // Если система запустилась нормально, работаем с тайМером WatchDog.
  if (startSys == true && errorReset == false) {
    PORTD &= ~(1 << LED_RED);
    PORTD |= 1 << LED_GREEN;

    if (millis() >= resTim && cnt < tryReset) {
      reset();
      cntErr = 0;
    } else if (cnt >= tryReset) {
      errorReset = true;
      PORTD &= ~(1 << LED_GREEN);
      PORTD |= 1 << LED_RED;
      delay(10000);
    }
    if (!(PIND & (1 << IMPULSE))) {
      flagImpulse = 1;
    }
    if (PIND & (1 << IMPULSE) && flagImpulse == 1) {
      if (PIND & (1 << IMPULSE)) {
        flagImpulse = 0;
      }
      resTim = timeWatchdog + millis();
    }
  }
  else if (firstStartSys == false && cnt == tryReset && errorReset != true) {
    errorReset = true;
    cntErr = 0;
    PORTD &= ~(1 << LED_GREEN);
    PORTD |= 1 << LED_RED;

  }

  // Решить нужно ли это решение?
  // ------------------!!!!!!!!!!!!!!!!!!!----------------------
  // Убрать перезагрузку поста при появлении импульсов. Ибо пост работает.
  if (errorReset == true) {
    if (!(PIND & (1 << IMPULSE))) {
      flagImpulse = 1;
    }
    if (PIND & (1 << IMPULSE) && flagImpulse == 1) {
      if (PIND & (1 << IMPULSE)) {
        flagImpulse = 0;
      }
      cntErr++;
      delay(50);
    }
    if (cntErr == 3) {
      cnt = tryReset - 1;
      startSys = true;
      errorReset = false;
      PORTD &= ~(1 << LED_RED);
    }
  }
}


//  --------------------------------------Процедуры и функции--------------------------------------

void reset() {
  if (cnt < tryReset && errorReset == false) {

    PORTD |= 1 << RELAY;
    delay(resetTime);
    PORTD &= ~(1 << RELAY);
    startTime = millis();

    while (millis() <= startTime + timeWaitReset) {
      // Если система загрузилась ставим флаг
      if (!(PIND & (1 << IMPULSE))) {
        flagImpulse = 1;
      }
      if (PIND & (1 << IMPULSE) && flagImpulse == 1) {
        if (PIND & (1 << IMPULSE)) {
          flagImpulse = 0;
        }
        startSys = true;
        resTim = timeWatchdog + millis();
        cnt = 0;
        PORTD &= ~(1 << LED_GREEN);
        break;
      }

      if (blinkTimeFlag == 0) {
        blinkTime = millis();
        blinkTimeFlag = 1;
      }

      if (millis() >= blinkTime + INTERVAL_LED_BLINK) {
        PORTD |= 1 << LED_GREEN;
        if (millis() >= blinkTime + INTERVAL_LED_BLINK * 2) {
          PORTD &= ~(1 << LED_GREEN); blinkTimeFlag = 0;
        }
      }
    }
    cnt++;
  }

  if (cnt == tryReset && errorReset != true) {
    errorReset = true;
    cntErr = 0;
    PORTD &= ~(1 << LED_GREEN);
    PORTD |= 1 << LED_RED;

  }
}
