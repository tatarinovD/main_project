#include <EEPROM.h>  
#include <Wire.h>
#include <OneWire.h>
#include "RTClib.h"
#include "HTU21D.h"
#include "Adafruit_VEML6070.h"
RTC_DS1307 RTC; //работат с реалтайм часами 
Adafruit_VEML6070 uv = Adafruit_VEML6070(); //обьявляем раобту с датчиком ультрафиолета 
HTU21D myHumidity; //обьект работы с датчиком темпиратуры и влажности 

long previousMillis = 10000; // частота опроса датчиков 
long interval = 10000; // интервал опроса часов
byte ManualMod;
byte PreManualMod;
byte PinDL;
byte PinNL;
byte PinUH;
byte PinDH;
byte PinH;
byte PinDT;
byte DMinTemp;
byte DMaxTemp;
byte NMinTemp;
byte NMaxTemp;
byte HMinTemp;
byte HMaxTemp;
byte DDHMinTemp;
byte DDHMaxTemp;
byte NDHMinTemp;
byte NDHMaxTemp;
byte UWMinLevel;
byte BeginDay;
byte EndDay;
byte NUWMinLevel;
byte NUWMaxLevel;
byte DLState;
byte NLState;
byte UHState;
byte DHState;
byte HState ;  

OneWire ds(PinDT); // на пине 6 (нужен резистор 4.7 КОм)

void setup() {
  EEPROM.begin(40);//текущее значени 30 макс колличество переменных 512
  // блок опредиления заданных параметров контроллера 
   ManualMod = EEPROM.read(0); // режим работы контроллера 0- авто 1- ручной 
   PinDL = EEPROM.read(1); // пин подключения реле лампы дневного света 
   PinNL = EEPROM.read(2); // пин подключения реле лампы ночного света  
   PinUH = EEPROM.read(3); // пин подключения реле верхнего подогрева   
   PinDH = EEPROM.read(4); // пин подключения реле нижнего подогрева 
   PinH  = EEPROM.read(5); // пин подключения реле увлажнителья воздуха 
   PinDT = EEPROM.read(6); // пин подключения цифрового датчика темпиратуры DS18B20 
   DMinTemp = EEPROM.read(7);  //дневной минимум темпиратуры
   DMaxTemp = EEPROM.read(8);  //дневной максимум темпиратуры
   NMinTemp = EEPROM.read(9);  //ночной минимум темпиратуры
   NMaxTemp = EEPROM.read(10);  //ночной максимум темпиратуры
   HMinTemp = EEPROM.read(11);  //минимальное значение влажности 
   HMaxTemp = EEPROM.read(12);  //максимальное значение влажности 
   DDHMinTemp = EEPROM.read(13); //дневной минимум температуры нижнего подогрева
   DDHMaxTemp = EEPROM.read(14); //дневной максимум температуры нижнего подогрева
   NDHMinTemp = EEPROM.read(15); //ночной минимум температуры нижнего подогрева
   NDHMaxTemp = EEPROM.read(16); //ночной максимум температуры нижнего подогрева
   UWMinLevel = EEPROM.read(17); //минимальное значение УФ излучения от лампы дневного света 
   BeginDay = EEPROM.read(18); //начало светового дня 
   EndDay = EEPROM.read(19); //окончание светового дня 
   NUWMinLevel = EEPROM.read(20); //минимальное значение УФ для включения лампы ночного света 
   NUWMaxLevel = EEPROM.read(21); //минимальное значение УФ для вкылючения лампы ночного света 
  //
   pinMode(PinDL,OUTPUT);
   pinMode(PinNL,OUTPUT);
   pinMode(PinUH,OUTPUT);
   pinMode(PinDH,OUTPUT);
   pinMode(PinH,OUTPUT);
  
  if (ManualMod) {
    EEPROM.begin(512);
    DLState = EEPROM.read(30); // состояние реле лампы дневного света 
    NLState = EEPROM.read(31); // состояние реле лампы ночного света  
    UHState = EEPROM.read(32); // состояние реле верхнего подогрева   
    DHState = EEPROM.read(33); // состояние реле нижнего подогрева 
    HState  = EEPROM.read(34);  // состояние реле увлажнителья воздуха 
    digitalWrite(PinDL,DLState);
    digitalWrite(PinNL,NLState);
    digitalWrite(PinUH,UHState);  
    digitalWrite(PinDH,DHState); 
    digitalWrite(PinH,HState); 
  }
 Serial.begin(115200);
 Wire.begin(4, 5); // определяем пины L2C (D2,D1)
 myHumidity.begin();
 RTC.begin();//  Активация часов 
 
}
void loop() {
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    auto_mod();
  }
}

void auto_mod() {
  
    if (!ManualMod){
      float humd = myHumidity.readHumidity();
      float temp = myHumidity.readTemperature();
      float tempDH = currentDigTemp(); 
      float UW = uv.readUV();
      DLState = digitalRead(PinDL);
      NLState = digitalRead(PinNL);
      UHState = digitalRead(PinUH);
      DHState = digitalRead(PinDH);
      HState  = digitalRead(PinH);
      if (humd < HMinTemp and digitalRead(PinH) == 1) digitalWrite(PinH,0); // включаем увлажнитель 
      if (humd >= HMaxTemp and digitalRead(PinH) == 0) digitalWrite(PinH,1); // выключаем увлажнитель 
      DateTime now = RTC.now(); //считываение параметров часов 
      if(now.hour() >= BeginDay and  now.hour() < EndDay) { // --------------------------день а свет не включен
        if (DLState == 1) {
          digitalWrite(PinDL,0);//включаем дневной свет 
          if (!NLState) digitalWrite(PinNL,1); //включаем ночной свет если он включен
        }
        if (temp <= DMinTemp and UHState == 1) digitalWrite(PinUH,0); //включаем подогрев на дневном минимуме
        if (temp >= DMaxTemp and UHState ==0) digitalWrite(PinUH,1); //отключаем дневной подогрев по максимуму
        if (tempDH < DDHMinTemp and DHState ==1) digitalWrite(PinDH,0); //включаем коврик по минимуму
        if (tempDH >= DDHMaxTemp and DHState ==0) digitalWrite(PinDH,1); //отключаем коврик по достижению максимума 
      }
      else {
        if (!DLState) digitalWrite(PinDL,1);
        if (NLState == 1 and UW < NUWMinLevel) digitalWrite(PinNL,0);//включаем ночной свет по УФ датчику
        if (NLState == 0 and UW > NUWMaxLevel) digitalWrite(PinNL,1);//выключаем ночной свет по УФ датчику
        if (temp <= NMinTemp and UHState == 1) digitalWrite(PinUH,0); //включаем подогрев на дневном минимуме
        if (temp >= NMaxTemp and UHState ==0) digitalWrite(PinUH,1); //отключаем дневной подогрев по максимуму
        if (tempDH < DDHMinTemp and DHState ==1)digitalWrite(PinDH,0); //включаем коврик по минимуму
        if (tempDH >= DDHMaxTemp and DHState ==0)digitalWrite(PinDH,1); //отключаем коврик по достижению максимума 
      }
   }
}    
float currentDigTemp(){
  byte i;
  byte present = 0;
  byte type_s = 0;
  byte data[12];
  byte addr[8];
  ds.search(addr);
    if ( !ds.search(addr)) {
      ds.reset_search();
    }
   
   // the first ROM byte indicates which chip
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    }
  return  ((float)raw / 16.0);
}

