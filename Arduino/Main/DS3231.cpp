#include "DS3231.h"

uint8_t DS3231::address = 0;
bool DS3231::boolhasbegun = false;
Stream* DS3231::_serial = nullptr;

void DS3231::begin(const uint8_t addr, const bool wireBegan, Stream* serial) {
  address = addr;
  _serial = serial;
  _serial->println("librairie DS3231");
  if(!wireBegan){
    Wire.begin();
  }
  boolhasbegun = true;
}

bool DS3231::hasbegun() {
  return boolhasbegun;
}

//----------------------------------------------------------Lecture

void DS3231::readDisplayTime(uint8_t* buffer) {
  Wire.beginTransmission(address);
  Wire.write(0x01);
  Wire.endTransmission();
  uint8_t nbRead = 2;
  Wire.requestFrom(address, nbRead);
  int i = 0;
  while (Wire.available() && i < nbRead) {
    buffer[i++] = Wire.read();
  }
}

void DS3231::readAlarmCheck(uint8_t* buffer) {
  Wire.beginTransmission(address);
  Wire.write(0x01);
  Wire.endTransmission();
  uint8_t nbRead = 6;
  Wire.requestFrom(address, nbRead);
  int i = 0;
  while (Wire.available() && i < 4) {
    buffer[i++] = Wire.read();
  }
  if(Wire.available()){
    Wire.read();
  }
  while (Wire.available() && i < nbRead) {
    buffer[i++] = Wire.read();
  }
}

void DS3231::readFullDate(uint8_t* buffer) {
  Wire.beginTransmission(address);
  Wire.write(0x01);
  Wire.endTransmission();
  uint8_t nbRead = 6;
  Wire.requestFrom(address, nbRead);
  int i = 0;
  while (Wire.available() && i < nbRead) {
    buffer[i++] = Wire.read();
  }
}

void DS3231::setupFullDateRead(uint8_t* buffer) {
  Wire.beginTransmission(address);
  Wire.write(0x00);
  Wire.endTransmission();
  uint8_t nbRead = 7;
  Wire.requestFrom(address, nbRead);
  int i = 0;
  while (Wire.available() && i < nbRead) {
    buffer[i++] = Wire.read();
  }
}


//----------------------------------------------------------Ecriture

void DS3231::write1byte(const uint8_t Register, const uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(Register);
  Wire.write(data);
  Wire.endTransmission();
}

void DS3231::writeMbyte(const uint8_t FirstRegister, const uint8_t* data, const uint8_t nbBytes) {
  Wire.beginTransmission(address);
  Wire.write(FirstRegister);
  for(uint8_t i = 0; i < nbBytes; i++){
    Wire.write(data[i]);
  }
  Wire.endTransmission();
}

void DS3231::writeSec(const uint8_t second){
  write1byte(0x00, second);
}

void DS3231::writeMin(const uint8_t minute){
  write1byte(0x01, minute);
}

void DS3231::writeHou(const uint8_t hour){
  write1byte(0x02, hour);
}

void DS3231::writeDay(const uint8_t day){
  write1byte(0x03, day);
}

void DS3231::writeDat(const uint8_t date){
  write1byte(0x04, date);
}

void DS3231::writeMon(const uint8_t month){
  write1byte(0x05, month);
}

void DS3231::writeYea(const uint8_t year){
  write1byte(0x06, year);
}