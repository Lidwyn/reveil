#include "AT24C32.h"

uint8_t AT24C32::address = 0;
bool AT24C32::boolhasbegun = false;
const uint8_t AT24C32::RegBytes = 8;
const uint8_t AT24C32::NUBytes = 0x08; //debut de la zone NU 0x08 = 8; fin en 0x34 = 52
const uint8_t AT24C32::UBytes = 0x36; //debut de la zone U 0x63 = 54; fin en 0x53 = 83
Stream* AT24C32::_serial = nullptr;

void AT24C32::begin(const uint8_t addr, const bool wireBegan, Stream* serial) {
  address = addr;
  _serial = serial;
  _serial->println("librairie AT24C32");
  if(!wireBegan){
    Wire.begin();
  }
  boolhasbegun = true;
}

bool AT24C32::hasbegun() {
  return boolhasbegun;
}

//----------------------------------------------------------Lecture

uint8_t AT24C32::readAll(uint8_t* bufferNU, uint8_t* bufferU){
  uint8_t ret = 0; // 0bxxxx yyyy x-> nbNU; y-> nbU

  uint8_t nbRead = RegBytes;
  uint8_t readReg[nbRead];
  Wire.beginTransmission(address);
  Wire.write(0x00); //adresse haute
  Wire.write(0x00); //adresse basse
  Wire.endTransmission();
  Wire.requestFrom(address, nbRead);
  uint8_t i = 0;

  while (Wire.available() && i < nbRead) {
    readReg[i++] = Wire.read();
  }

  uint8_t pos[15];
  nbRead = 0;

  for(i = 0; i < RegBytes/2; i++){ // For each line in the first half of the register (NU alarms)
    uint8_t b = (readReg[i] & 0b01010101); // b takes the value the i line, only the part that concerns if it exist
    while (b) { // While there is still some 1s in b
      uint8_t bit = __builtin_ctz(b) / 2; // We look for the number of 0 behind the first bit at 1, and divide by 2 to know the pos in this register line
      pos[nbRead++] = i * 4 + bit; // pos[] take the positions of all the alarm in the register
      b &= b - 1; // Remove the last 1 in b and start over until b is empty
    }
  }

  for(uint8_t j = 0; j < nbRead; j++){
    Wire.beginTransmission(address);
    Wire.write(0x00); // Upper address
    Wire.write(RegBytes+pos[j]*3); // Lower address : the exact position of the alarm data = Regbytes + pos(number computed before) * 3 (NUalarm data take 3 lines)
    Wire.endTransmission();
    Wire.requestFrom(address, (uint8_t)3); // Looking for 3 lines of data : 1 alarm data
    uint8_t k = 0;
    while (Wire.available() && k < 3) {
      bufferNU[(3*j)+k] = Wire.read(); // We inject the read data to bufferNU
      k++;
    }
  }
  ret = nbRead<<4; // Number of NU is saved in the first 4 bits of ret (later nbAlarm)
  
  // We do the same for U alarms, alarm data is saved on only 2 lanes this time
  nbRead = 0;
  for(; i < RegBytes; i++){
    uint8_t b = (readReg[i] & 0b01010101);
    while (b) {
      uint8_t bit = __builtin_ctz(b) / 2;
      pos[nbRead++] = (i - 4) * 4 + bit;
      b &= b - 1;
    }
  }
  
  for(uint8_t j = 0; j < nbRead; j++){
    Wire.beginTransmission(address);
    Wire.write(0x00); //adresse haute
    Wire.write(UBytes+pos[j]*2); //adresse basse
    Wire.endTransmission();
    Wire.requestFrom(address, (uint8_t)2);
    uint8_t k = 0;
    while (Wire.available() && k < 2) {
      bufferU[2*j+k] = Wire.read();
      k++;
    }
  }
  ret += nbRead;

  return ret;
}

uint8_t AT24C32::readActive(uint8_t* bufferNU, uint8_t* bufferU){
  uint8_t ret = 0; // 0bxxxx yyyy x-> nbNU; y-> nbU

  uint8_t nbRead = RegBytes;
  uint8_t readReg[nbRead];
  Wire.beginTransmission(address);
  Wire.write(0x00); //adresse haute
  Wire.write(0x00); //adresse basse
  Wire.endTransmission();
  Wire.requestFrom(address, nbRead);
  uint8_t i = 0;

  while (Wire.available() && i < nbRead) {
    readReg[i++] = Wire.read();
  }

  uint8_t pos[15];
  nbRead = 0;

  for(i = 0; i < RegBytes/2; i++){
    uint8_t b = (readReg[i] & 0b10101010)>>1;
    while (b) {
      uint8_t lsb = b & -b;
      uint8_t bit = __builtin_ctz(lsb) / 2;
      pos[nbRead++] = i * 4 + bit;
      b &= b - 1;
    }
  }

  for(uint8_t j = 0; j < nbRead; j++){
    Wire.beginTransmission(address);
    Wire.write(0x00); //adresse haute
    Wire.write(RegBytes+pos[j]*3); //adresse basse
    Wire.endTransmission();
    Wire.requestFrom(address, (uint8_t)3);
    uint8_t k = 0;
    while (Wire.available() && k < 3) {
      bufferNU[(3*j)+k] = Wire.read();
      k++;
    }
  }
  ret = nbRead<<4;
  
  nbRead = 0;
  for(; i < RegBytes; i++){
    uint8_t b = (readReg[i] & 0b10101010)>>1;
    while (b) {
      uint8_t lsb = b & -b;
      uint8_t bit = __builtin_ctz(lsb) / 2;
      pos[nbRead++] = (i - 4) * 4 + bit;
      b &= b - 1;
    }
  }
  
  for(uint8_t j = 0; j < nbRead; j++){
    Wire.beginTransmission(address);
    Wire.write(0x00); //adresse haute
    Wire.write(UBytes+pos[j]*2); //adresse basse
    Wire.endTransmission();
    Wire.requestFrom(address, (uint8_t)2);
    uint8_t k = 0;
    while (Wire.available() && k < 2) {
      bufferU[2*j+k] = Wire.read();
      k++;
    }
  }
  ret += nbRead;

  return ret;
}

//----------------------------------------------------------Ecriture

bool AT24C32::createNewNU(uint8_t* buffer){
  uint8_t pos = 16;
  //check first place
  uint8_t nbRead = RegBytes/2;
  uint8_t readReg[nbRead];
  Wire.beginTransmission(address);
  Wire.write(0x00); //adresse haute
  Wire.write(0x00); //adresse basse
  Wire.endTransmission();
  Wire.requestFrom(address, nbRead);
  uint8_t i = 0;

  while (Wire.available() && i < nbRead) {
    readReg[i++] = Wire.read();
  }

  bool notfound = true;
  i = 0;
  while(notfound && i < nbRead) {
    uint8_t reg = ~(readReg[i] | 0b10101010);
    if(reg){
      uint8_t vPos = __builtin_ctz(reg);
      pos = 4 * i + (vPos)/2;
      uint8_t tmp = readReg[i] | 0b11 << vPos;
      Wire.beginTransmission(address);
      Wire.write(0x00); //adresse haute
      Wire.write(i); //adresse basse
      Wire.write(tmp);
      Wire.endTransmission();
      notfound = false;
    }
    i++;
  }
  if(notfound || (pos > 15)){
    return false;
  }

  //ecriture
  delay(10);
  Wire.beginTransmission(address);
  Wire.write(0x00); //adresse haute
  Wire.write(NUBytes + pos * 3); //adresse basse
  Wire.write(buffer[0]); // actif + minute : 1 bit pour actif, 3 bits pour la dizaine, 4 bits pour le chiffre
  Wire.write(buffer[1]); // heure : 2 bits à 0, 2 bits pour la dizaine, 4 bits pour le chiffre
  Wire.write(buffer[2]); // jour de la semaine
  Wire.endTransmission();
  return true;
}

bool AT24C32::createNewU(uint8_t* buffer){
  uint8_t pos = 16;
  //check first place
  uint8_t nbRead = RegBytes/2;
  uint8_t readReg[nbRead];
  Wire.beginTransmission(address);
  Wire.write(0x00); //adresse haute
  Wire.write(RegBytes/2); //adresse basse
  Wire.endTransmission();
  Wire.requestFrom(address, nbRead);
  uint8_t i = 0;

  while (Wire.available() && i < nbRead) {
    readReg[i++] = Wire.read();
  }

  bool notfound = true;
  i = 0;
  while(notfound && i < nbRead) {
    uint8_t reg = ~(readReg[i] | 0b10101010);
    if(reg){
      uint8_t vPos = __builtin_ctz(reg);
      pos = 4 * i + (vPos)/2;
      uint8_t tmp = readReg[i] | 0b11 << vPos;
      Wire.beginTransmission(address);
      Wire.write(0x00); //adresse haute
      Wire.write(i + (RegBytes/2)); //adresse basse
      Wire.write(tmp);
      Wire.endTransmission();
      notfound = false;
    }
    i++;
  }
  if(notfound || (pos > 15)){
    return false;
  }

  //ecriture
  delay(10);
  Wire.beginTransmission(address);
  Wire.write(0x00); //adresse haute
  Wire.write(UBytes + pos * 2); //adresse basse
  Wire.write(buffer[0]); // actif + minute : 1 bit pour actif, 3 bits pour la dizaine, 4 bits pour le chiffre
  Wire.write(buffer[1]); // heure : 2 bits à 0, 2 bits pour la dizaine, 4 bits pour le chiffre
  Wire.endTransmission();
  return true;
}

void AT24C32::modifyNU(uint8_t nb, uint8_t* buffer){
  uint8_t pos = 16;
  //check first place
  uint8_t nbRead = RegBytes/2;
  uint8_t readReg[nbRead];
  Wire.beginTransmission(address);
  Wire.write(0x00); //adresse haute
  Wire.write(0x00); //adresse basse
  Wire.endTransmission();
  Wire.requestFrom(address, nbRead);
  uint8_t i = 0;

  while (Wire.available() && i < nbRead) {
    readReg[i++] = Wire.read();
  }

  bool notfound = true;
  i = 0;
  uint8_t j = 0;
  while(notfound && (i < nbRead)){
    uint8_t b = (readReg[i] & 0b01010101)>>1;
    while (b && notfound) {
      uint8_t lsb = b & -b;
      uint8_t bit = __builtin_ctz(lsb) / 2;
      pos = i * 4 + bit;
      b &= b - 1;
      if(j == nb){
        notfound = false;
        Wire.beginTransmission(address);
        Wire.write(0x00);
        Wire.write(i);
        if(buffer[0] & 0b10000000){ // Changing reg depending on Active state
          Wire.write(readReg[i] | 0b11<<(pos*2));
        }
        else{
          Wire.write(readReg[i] & ~(0b10<<(pos*2)));
        }
        Wire.endTransmission();
      }
      j++;
    }
    i++;
  }

  Wire.beginTransmission(address);
  Wire.write(0x00);
  Wire.write(RegBytes+pos*3);
  Wire.write(buffer[0]);
  Wire.write(buffer[1]);
  Wire.write(buffer[2]);
  Wire.endTransmission();
}

void AT24C32::modifyU(uint8_t nb, uint8_t* buffer){
  uint8_t pos = 16;
  //check first place
  uint8_t nbRead = RegBytes/2;
  uint8_t readReg[nbRead];
  Wire.beginTransmission(address);
  Wire.write(0x00); //adresse haute
  Wire.write(RegBytes/2); //adresse basse
  Wire.endTransmission();
  Wire.requestFrom(address, nbRead);
  uint8_t i = 0;

  while (Wire.available() && i < nbRead) {
    readReg[i++] = Wire.read();
  }

  bool notfound = true;
  i = 0;
  uint8_t j = 0;
  while(notfound && (i < nbRead)){
    uint8_t b = (readReg[i] & 0b01010101)>>1;
    while (b && notfound) {
      uint8_t lsb = b & -b;
      uint8_t bit = __builtin_ctz(lsb) / 2;
      pos = i * 4 + bit;
      b &= b - 1;
      if(j == nb){
        notfound = false;
        Wire.beginTransmission(address);
        Wire.write(0x00);
        Wire.write(i);
        if(buffer[0] & 0b10000000){ // Changing reg depending on Active state
          Wire.write(readReg[i] | 0b11<<(pos*2));
        }
        else{
          Wire.write(readReg[i] & ~(0b10<<(pos*2)));
        }
        Wire.endTransmission();
      }
      j++;
    }
    i++;
  }

  Wire.beginTransmission(address);
  Wire.write(0x00);
  Wire.write(UBytes + pos * 2);
  Wire.write(buffer[0]);
  Wire.write(buffer[1]);
  Wire.endTransmission();
}