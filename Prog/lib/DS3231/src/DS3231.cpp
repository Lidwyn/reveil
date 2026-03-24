#include "DS3231.h"

uint8_t DS3231::address = 0;
bool DS3231::boolhasbegun = false;
Stream* DS3231::_serial = nullptr;

void DS3231::begin(const uint8_t addr, const bool wireBegan, Stream* serial) {
  _serial->println("DS3231 library");
  address = addr;
  _serial = serial;
  write1byte(0x0E, 0); // This register might reset when corrupted, so I set it every initialisation
  if(!wireBegan){
    Wire.begin();
  }
  boolhasbegun = true;
}

bool DS3231::hasbegun() {
  return boolhasbegun;
}

//----------------------------------------------------------Reading

void DS3231::readDisplayTime(uint8_t* buffer) {
  // Read minute (reg01) and hour (reg02) and write them in the pointer buffer 
  Wire.beginTransmission(address);          // ╗
  Wire.write(0x01);                         // ║
  Wire.endTransmission();                   // ║
  uint8_t nbRead = 2;                       // ║
  Wire.requestFrom(address, nbRead);        // ║> Classic I2C reading
  int i = 0;                                // ║
  while (Wire.available() && i < nbRead) {  // ║
    buffer[i++] = Wire.read();              // ║
  }                                         // ╝
}

void DS3231::readFullDate(uint8_t* buffer) {
  // Read full date (except second) and write them in the pointer buffer
  Wire.beginTransmission(address);
  Wire.write(0x01);
  Wire.endTransmission();
  uint8_t nbRead = 6; // It should work with only 3 for minutes, hour and day
  Wire.requestFrom(address, nbRead);
  int i = 0;
  while (Wire.available() && i < nbRead) {
    buffer[i++] = Wire.read();
  }
}

void DS3231::setupFullDateRead(uint8_t* buffer) {
  // Read full date (with second) and write them in the pointer buffer for the setup mode
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


//----------------------------------------------------------Writing

void DS3231::write1byte(const uint8_t Register, const uint8_t data) {
  // Generic 1 register writing function
  Wire.beginTransmission(address);
  Wire.write(Register);
  Wire.write(data);
  Wire.endTransmission();
}

void DS3231::writeMbyte(const uint8_t FirstRegister, const uint8_t* data, const uint8_t nbBytes) {
  // Generic multiple registers writing function
  Wire.beginTransmission(address);
  Wire.write(FirstRegister);
  for(uint8_t i = 0; i < nbBytes; i++){
    Wire.write(data[i]);
  }
  Wire.endTransmission();
}

// Generic specific register writing, not used but good to have
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