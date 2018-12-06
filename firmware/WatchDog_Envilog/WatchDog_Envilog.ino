/*
 * File:   WatchDog Envilog DLM-1470
 * Author: Telegin V.S. Polyarnie Zori KOLNPP
 * Created on 06.09.2018
 */
#define StatusPin 3                         // PD3 - ATmega8A - вывод на оптопару.
#define Relay 5                             // PD5 - ATmega8A - управление реле.

const int tryReset = 5;
const unsigned long timeWtachdog = 10000;  //60секунд
const unsigned long timeWaitReset = 10000; //60секунд
const unsigned long ResetTime = 10000;     //60секунд
unsigned long currentTime, loopTime;

int cnt = 0;
bool errorReset = false;

//Установки при запуске платы
void setup() {
  //Уствновка настроек порта Status
  pinMode(StatusPin, INPUT);
  //digitalWrite(StatusPin, HIGH);
  //Установка прота Relay
  pinMode(Relay, OUTPUT);
    pinMode(6, OUTPUT);
      pinMode(7, OUTPUT);
  digitalWrite(Relay, LOW);
 
  currentTime = millis();
  }

//Старт основной программы
void loop() {
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW);
    delay(2000);
    digitalWrite(7, HIGH);
     digitalWrite(6, LOW);
delay(2000);
  /*
  //Сбрасываем счётчик если есть сигнал от Envilog
  if(digitalRead(StatusPin) == HIGH){
    currentTime = millis();
    cnt = 0;
    }
  //Проверяем количество попыток перезапуска Envilog. Если превысило лимит выставляем флаг ошибки reset
  if(cnt > tryReset){
    errorReset = true;
  }
  //Если счётчик вышел и нет ошибки количества попыток перезапуска то перезагружаем Envilog
  if(millis() > currentTime + timeWtachdog && errorReset != true){
    reset();
    cnt++;
  }
  
}

// Функция перезапуска Envilog
void reset(){
    
    digitalWrite(Relay, HIGH);
    delay(5000);
    digitalWrite(Relay, LOW);
    currentTime = millis() + ResetTime;
*/
}
