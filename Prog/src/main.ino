/*
+-----------------------------------------------+
|                                               |
| Ajout du setup                                |
| Projet Reveil                                 |
| setupMode.ino                                 |
| Lidwyn Le Bars 08/08/2025                     |
+-----------------------------------------------+
*/

#include <avr/sleep.h>        // sommeil/mode eco du cpu
#include <avr/interrupt.h>    // interruptions
#include <Wire.h>             // bus I2C
#include "DS3231.h"           // DS3231
#include "AT24C32.h"          // AT24C32
#include "MAX7219.h"          // MAX7219
#include <AltSoftSerial.h>      // Necessary for DFPlayer
#include "DFPlayer.h"         // DFPlayer

#define PB6_OUT()   DDRB |= (1 << DDB6)
#define PB6_HIGH()  PORTB |= (1 << PORTB6)
#define PB6_LOW()   PORTB &= ~(1 << PORTB6)

//partie interruption
volatile bool RTCFlag = false;
volatile bool RTCLastPinState = LOW;
volatile bool refreshNextAlarmDislpay = false;

//addresse des éléments du bus I2C
const uint8_t DS3231addr = 0x68;
const uint8_t AT24C32addr = 0x57;

//variable de calcul des reveils
uint8_t timeData[6];
uint8_t AlarmNU[45];
uint8_t AlarmU[30];
uint8_t ActiveNU[45];
uint8_t ActiveU[30];
uint8_t nbAlarm = 0;
uint8_t nbActive = 0;
uint8_t nextAlarmIndex;
bool AlarmIsActive = false;

//variable de mode reveil
volatile uint8_t mode = 0; // 0: normal, 1: setup, 2: menu alarm, 3: alarm audio

//variable de setup
bool setupIsInit = false;
uint8_t newTimeData[7]; // traitement en amont des valeurs à envoyer dans le DS3231
uint8_t isSetting;
uint8_t selected = 1; // 0: heure, 1: minutes, 2: secondes, 3: jour(date), 4: mois, 5: année 
unsigned long lastBlink = 0; // variable de test pour le clignotement
bool setupIsBlinking = 0; // 0: éteint, 1: allumé
const unsigned long blinkTime = 500; // clignotement de 500ms (f = 1Hz)
unsigned long BtLastReg = 0; // Variable de dernière itération de l'utilisation du bouton (haut ou bas)
const unsigned long BtRegPeriod = 100; // Période entre chaque itération de l'utilisation du bouton
const unsigned long BtRegFirstPeriod = 500; // Période avant la première itération de l'utilisation du bouton
bool FirstRegDone = false; // Sert à savoir si la premère itération a été réalisée
bool LRHysteresis = false; // Variable permettant la mise en place d'un cycle d'hystérésis sur l'utilisation des boutons gauche et droite

//variable du menu reveil
uint8_t selectedAlarm = 1; // alarme selectionnée
bool selectedType = 0; // type d'alarme selectionnée (unique/non unique)
bool menuReveilInit = false;
bool confirmDelete = false;
bool settingAlarm = false;
bool settingAlarmInit = false;
bool alarmIsNew = false;
bool UDHysteresis = false; // Variable permettant la mise en place d'un cycle d'hystérésis sur l'utilisation des boutons haut et bas
// réutilisation de newTimeData, isSetting et selected
// réutilisation de lastBlink, setupIsBlinking et blinkTime

//timing
unsigned long lastSecond = 0;
uint8_t lastMinute = 61;
const unsigned long secondInterval = 1000;

//variable d'affichage
uint8_t displayTime[2];
bool showDoubleDot = true;
uint8_t displayNextAlarm[3];

//night mode
volatile bool nightMode = false;

//creation des éléments MAX7219 et de leur pin cs associé (SPI)
MAX7219 myMAX7219_1(12);
MAX7219 myMAX7219_2(11);
bool ddot = true; //variable d'affichage des deux points
volatile uint8_t brightness = 0x03; // Variable de luminosité des afficheurs (entre 0x00 et 0x0F)
volatile bool brightnessFlag = false;

//numeral code for MAX7219/7seg from 0 to 9
const uint8_t NumeralCode1[10] = {
  0b11110110,
  0b00110000,
  0b01011110,
  0b00111111,
  0b10111000,
  0b10101110,
  0b11101110,
  0b00110100,
  0b11111110,
  0b10111110
};

const uint8_t NumeralCode2[10] = {
  0b10110111,
  0b00010001,
  0b00111110,
  0b00111101,
  0b10011001,
  0b10101101,
  0b10101111,
  0b00010101,
  0b10111111,
  0b10111101
};

//address adaptator for MAX7219/7seg, address x != DIGx
const uint8_t MAXAddress1[5] = {
  0x08, // digit 4 = DIG7 (from right to left on the display)
  0x04, // digit 3 = DIG3
  0x03, // digit 2 = DIG2
  0x07, // digit 1 = DIG6
  0x01  // ddot = DIG0
};

const uint8_t MAXAddress2[5] = {
  0x03, // digit 4 = DIG2 (from right to left on the display)
  0x07, // digit 3 = DIG6
  0x05, // digit 2 = DIG4
  0x01, // digit 1 = DIG0
  0x04  // leds = DIG3
};

// Buttons PIN
uint8_t btPlus = 5;         // + button on D5 / PIN 11 / PCINT21
bool btPlusLastState = false; // Save the last value of the button for better fluidity in setup/alarm modes
uint8_t btMoins = 7;        // - button on D7 / PIN 13 / PCINT23
bool btMoinsLastState = false; // Save the last value of the button for better fluidity in setup/alarm modes
uint8_t btSetup = 3;        // Setup button on D3 / PIN 5 / PCINT19
uint8_t btA = 4;            // Top button on D4 / PIN 6 / PCINT20
uint8_t btGauche = 2;       // Left button on D2 / PIN 4 / PCINT18
bool btGaucheLastState = false; // Save the last value of the button for better fluidity in setup/alarm modes
uint8_t btDroite = 6;       // Right Button on D6 / PIN 12 / PCINT22
bool btDroiteLastState = false; // Save the last value of the button for better fluidity in setup/alarm modes

//Audio
const uint8_t DFPrx = 8; // to TX
const uint8_t DFPtx = 9; // to RX
// const uint8_t DFPpower = A0; // Not A0 but PB6 which doesn't have number in the standard Arduino core
AltSoftSerial DFPlayerPort; // RX = D8/PB0 TX = D9/PB1 (forced)
bool isAwake = false;
bool isReset = false;
unsigned long alarmWakeUpTimer = 0; // This timer is for the delay between the DFPlayer turning on and the first UART transmission
const unsigned long wakeUpTime = 900; // From testing, 900 ms was the smallest amount of time necessary
// réutilisation de lastBlink, setupIsBlinking et blinkTime

void setup() { //-------------------------------------------------setup
  unsigned long setupTimer = millis(); // Setup duration and for DFPlayer setup timer
  Serial.begin(9600); // Debug serial port at 9600 bauds

  //Audio
  pinMode(DFPrx, OUTPUT);
  pinMode(DFPtx, OUTPUT);
  PB6_LOW();
  PB6_OUT(); // Same as digitalWrite(DFPpower, LOW);
  // Turning the DFPlayer on for setup (low = on --> mosfet P-channel)
  // 1 sec delay necessary after this, so the rest of the setup is found at the end 
  
  for(uint8_t i = 0; i < 20; i++){ 
    Serial.println(); //serial clear
  }
  Serial.println("Hello from ATmega328P !");
  //Display init
  MAX7219::begin(13, 10, &Serial);
  myMAX7219_1.init();
  myMAX7219_2.init();

  
  Serial.println("test 3");
  delay(200);
  //Turning the display on while setup
  myMAX7219_1.send(MAXAddress1[0], 0xff);  // Turning on all the led for every 7-seg
  myMAX7219_1.send(MAXAddress1[1], 0xff);
  myMAX7219_1.send(MAXAddress1[2], 0xff);
  myMAX7219_1.send(MAXAddress1[3], 0xff);
  myMAX7219_1.send(MAXAddress1[4], 0xff);

  myMAX7219_2.send(MAXAddress2[0], 0xff);
  myMAX7219_2.send(MAXAddress2[1], 0xff);
  myMAX7219_2.send(MAXAddress2[2], 0xff);
  myMAX7219_2.send(MAXAddress2[3], 0xff);
  myMAX7219_2.send(MAXAddress2[4], 0xff);

  //I2C init
  Wire.begin(); // wire.h setup for I2C bus
  DS3231::begin(DS3231addr, true, &Serial);
  AT24C32::begin(AT24C32addr, true, &Serial);

  //Interruption clock (1hz so two interruption per second -> passage 0-1 et 1-0)
  pinMode(A3, INPUT);         // DS3131 clock input on A3 (PC3)
  PCICR |= (1 << PCIE1);      // Activing interruptions on PCINT[14:8] --> PC0 to PC5 / A0 to A5
  PCMSK1 |= (1 << PCINT11);   // Activing interruption specifically on PCINT11 --> PC3 / A3
  
  //Interruption boutons
  PCICR |= (1 << PCIE2);      // Activing interruptions on PCINT[16:23] --> PD0 to PD7 / D0 to D7

  pinMode(btPlus, INPUT);     // pinMode button
  PCMSK2 |= (1 << PCINT21);   // Activing interruption
  pinMode(btMoins, INPUT);
  PCMSK2 |= (1 << PCINT23);
  pinMode(btSetup, INPUT);
  PCMSK2 |= (1 << PCINT19);
  pinMode(btA, INPUT);
  PCMSK2 |= (1 << PCINT20);
  pinMode(btGauche, INPUT);   // No interruption on left button -> can upgrade the device through this button
  pinMode(btDroite, INPUT);
  PCMSK2 |= (1 << PCINT22);
  
  sei();  // Activing interruptions globaly

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  if(false){ // bus I2C debug
    //write eeprom
    /*
    Wire.beginTransmission(AT24C32addr);
    Wire.write(0x00); //adresse haute
    Wire.write(54); //adresse basse
    Wire.write(0b00010101);
    Wire.write(0b00010110);
    Wire.endTransmission();
    delay(50);
    */
    /*
    uint8_t k = 0;
    Wire.beginTransmission(AT24C32addr);
    Wire.write(0x00); //adresse haute
    Wire.write(0x00); //adresse basse
    //registre
    Wire.write(0b00000011); k++; // 1 alarme NU active
    Wire.write(0x00); k++; //r
    Wire.write(0x00); k++; //r
    Wire.write(0x00); k++; //r
    Wire.write(0b00000011); k++; // 1 alarme U active
    Wire.write(0x00); k++; //r
    Wire.write(0x00); k++; //r
    Wire.write(0x00); k++; //r
    //zone NU
    Wire.write(0b11000101); k++; // 45min + actif
    Wire.write(0b00001000); k++; // 8h
    Wire.write(0b00011111); k++; // lundi - vendredi
    Wire.endTransmission();
    delay(200);
    //ecriture de 0;
    while(k < 54){
      Wire.beginTransmission(AT24C32addr);
      Wire.write(0x00); //adresse haute
      Wire.write(k); //adresse basse
      Wire.write(0x00); //write 0s
      Wire.endTransmission();
      delay(50);
      k++;
    }

    Wire.beginTransmission(AT24C32addr);
    Wire.write(0x00); //adresse haute
    Wire.write(54); //adresse basse 54
    //zone U
    Wire.write(0b10010101); k++; // 15min + actif
    Wire.write(0b00010110); k++; // 16h
    Wire.endTransmission();
    delay(50);
    //ecriture de 0;
    while(k < 84){
      Wire.beginTransmission(AT24C32addr);
      Wire.write(0x00); //adresse haute
      Wire.write(k); //adresse basse
      Wire.write(0x00); //write 0s
      Wire.endTransmission();
      delay(50);
      k++;
    }
    */

    
    //read all eeprom for debug
    /*
    Serial.println("--- Read AT24C32 ---");
    uint8_t j = 0;
    while(j < 84){
      Wire.beginTransmission(AT24C32addr);
      Wire.write(0x00); //adresse haute
      Wire.write(j); //adresse basse
      Wire.endTransmission();
      Wire.requestFrom(AT24C32addr, (uint8_t)1);
      Serial.print(j);
      Serial.print(" : ");
      Serial.println(Wire.read(), BIN);
      delay(50);
      j++;
    }
    */
    

    /*
    //write DS3231
    Wire.beginTransmission(DS3231addr);
    Wire.write(0x0E);
    Wire.write(0);
    Wire.endTransmission();
    

    
    //read DS3231
    Serial.println("--- read DS3231 ---");
    uint8_t k = 0;
    Wire.beginTransmission(DS3231addr);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom((int)DS3231addr, 16);
    while (Wire.available() && k < 16) {
      Serial.println(Wire.read(),BIN);
    }
    */
  }
  
  //Time read setup
  DS3231::readFullDate(timeData);
  displayTime[0] = timeData[0];
  displayTime[1] = timeData[1];

  //Stored Alarm read setup
  nbAlarm = AT24C32::readAll(AlarmNU, AlarmU);
  nbActive = AT24C32::readActive(ActiveNU, ActiveU);

  //Next alarm setup
  nextAlarmIndex = findNextActiveAlarm(timeData, ActiveNU, (nbActive & 0xf0)>>4, ActiveU, nbActive & 0x0f);
  displayAlarm(nextAlarmIndex, ActiveNU, ActiveU, displayNextAlarm);
  
  //Audio rest of setup  
  while((millis() - setupTimer) < secondInterval){ //we check if we waited 1 sec after turning on the DFPlayer dure min 1 seconde
    delay(50);
  }
  DFPlayerPort.begin(9600);
  DFPLAYER::begin(&DFPlayerPort, DFPrx, DFPtx, &Serial);
  delay(50);
  DFPLAYER::toSleep();
  //Turning of the display
  myMAX7219_1.send(MAXAddress1[0], 0);
  myMAX7219_1.send(MAXAddress1[1], 0);
  myMAX7219_1.send(MAXAddress1[2], 0);
  myMAX7219_1.send(MAXAddress1[3], 0);
  myMAX7219_1.send(MAXAddress1[4], 0);

  myMAX7219_2.send(MAXAddress2[0], 0);
  myMAX7219_2.send(MAXAddress2[1], 0);
  myMAX7219_2.send(MAXAddress2[2], 0);
  myMAX7219_2.send(MAXAddress2[3], 0);
  myMAX7219_2.send(MAXAddress2[4], 0);
  delay(500); // Black display for .5s to signal en of setup

  //First display
  myMAX7219_1.send(MAXAddress1[3], NumeralCode1[(displayTime[1] & 0b00110000)>>4]);
  myMAX7219_1.send(MAXAddress1[2], NumeralCode1[displayTime[1] & 0b00001111]);
  myMAX7219_1.send(MAXAddress1[1], NumeralCode1[(displayTime[0] & 0b01110000)>>4]);
  myMAX7219_1.send(MAXAddress1[0], NumeralCode1[displayTime[0] & 0b00001111]);

  myMAX7219_2.send(MAXAddress2[3], NumeralCode2[(displayNextAlarm[1] & 0b00110000)>>4]);
  myMAX7219_2.send(MAXAddress2[2], NumeralCode2[displayNextAlarm[1] & 0b00001111]);
  myMAX7219_2.send(MAXAddress2[1], NumeralCode2[(displayNextAlarm[0] & 0b01110000)>>4]);
  myMAX7219_2.send(MAXAddress2[0], NumeralCode2[displayNextAlarm[0] & 0b00001111]);
  myMAX7219_2.send(MAXAddress2[4], dayLedsCompute(displayNextAlarm[2]));
  Serial.println("setup end");
  /*Serial.println("fin setup/debut nonboucle debug");
  Serial.print("setup realise en : ");
  Serial.println((millis() - setupTimer)/1000);*/
} //-------------------------------------------------------------end setup

void loop() { //-------------------------------------------------loop
  if(mode == 0){ //----------------------------------------------normal
    if(brightnessFlag){
      myMAX7219_1.Brightness(brightness);
      myMAX7219_2.Brightness(brightness);
      brightnessFlag = false;
    }
    if(RTCFlag){
      //double loopTimer = millis(); // calcul du temps
      RTCFlag = false;
      DS3231::readDisplayTime(displayTime); // getting new time to display and later detect alarm even in night mode
      if(!nightMode){
        //display time        
        myMAX7219_1.send(MAXAddress1[3], NumeralCode1[(displayTime[1] & 0b00110000)>>4]);
        myMAX7219_1.send(MAXAddress1[2], NumeralCode1[displayTime[1] & 0b00001111]);
        myMAX7219_1.send(MAXAddress1[1], NumeralCode1[(displayTime[0] & 0b01110000)>>4]);
        myMAX7219_1.send(MAXAddress1[0], NumeralCode1[displayTime[0] & 0b00001111]);
        myMAX7219_1.send(MAXAddress1[4], ddot<<3);
        ddot = !ddot;

        if(refreshNextAlarmDislpay){
          if(nbActive != 0){
            myMAX7219_2.send(MAXAddress2[3], NumeralCode2[(displayNextAlarm[1] & 0b00110000)>>4]);
            myMAX7219_2.send(MAXAddress2[2], NumeralCode2[displayNextAlarm[1] & 0b00001111]);
            myMAX7219_2.send(MAXAddress2[1], NumeralCode2[(displayNextAlarm[0] & 0b01110000)>>4]);
            myMAX7219_2.send(MAXAddress2[0], NumeralCode2[displayNextAlarm[0] & 0b00001111]);
            myMAX7219_2.send(MAXAddress2[4], dayLedsCompute(displayNextAlarm[2]));
          }
          else {
            myMAX7219_2.send(MAXAddress2[3], 0);
            myMAX7219_2.send(MAXAddress2[2], 0);
            myMAX7219_2.send(MAXAddress2[1], 0);
            myMAX7219_2.send(MAXAddress2[0], 0);
            myMAX7219_2.send(MAXAddress2[4], 0);
          }
          refreshNextAlarmDislpay = false;
        }
      }
      else{
        myMAX7219_1.send(MAXAddress1[0], 0);
        myMAX7219_1.send(MAXAddress1[1], 0);
        myMAX7219_1.send(MAXAddress1[2], 0);
        myMAX7219_1.send(MAXAddress1[3], 0);
        myMAX7219_1.send(MAXAddress1[4], 0);

        myMAX7219_2.send(MAXAddress2[0], 0);
        myMAX7219_2.send(MAXAddress2[1], 0);
        myMAX7219_2.send(MAXAddress2[2], 0);
        myMAX7219_2.send(MAXAddress2[3], 0);
        myMAX7219_2.send(MAXAddress2[4], 0);
      }

      if (lastMinute != computeValue(displayTime[0])){ // If new min we check if there is a new alarm
        /*Serial.println("new minute");*/
        lastMinute = computeValue(displayTime[0]);
        if(nbActive != 0){ // We check only if there are some alarms active
          DS3231::readFullDate(timeData);
          AlarmIsActive = checkAlarm(timeData, ActiveNU, (nbActive & 0xf0)>>4, ActiveU, nbActive & 0x0f);
          if(AlarmIsActive){
            Serial.println("!----------------------!");
            Serial.println("    Alarme detectee");
            Serial.println("!----------------------!");
            mode = 3; // Going in mode 3 : ringing mode
          }
        }
        // We could use something similar to displayNextAlarm[] to simplify things, it work like that so it's not a priority
      }
      
      delay(10);
    }
    
    // Au dodo
    sleep_enable();
    sleep_cpu();
    sleep_disable();
  }
  else if(mode == 1){ //-----------------------------------------mode setup
    DS3231::setupFullDateRead(newTimeData);
    if(!setupIsInit){
      selected = 0;
      isSetting = setupSelectNew(selected, newTimeData);
      setupIsInit = true;
      PCICR = 0; // Desactiver les interruptions pendant le setup
      // Cycle hysteresis du bouton setupEnd (/!\ stop le programme, mais ça n'est pas particulièrement important);
      bool setupSwitchStillUp = digitalRead(btSetup);
      while(setupSwitchStillUp){
        delay(10);
        setupSwitchStillUp = digitalRead(btSetup);
      }
    }
    
    unsigned long now = millis(); // Dans le mode 1 et 2 on utilise plus les interruptions du DS3231 mais millis pour le clignotement et autre

    
    // Partie bouton
    const bool setupUp = digitalRead(btPlus); // Plus grande prio
    const bool setupDown = digitalRead(btMoins);
    const bool setupRight = digitalRead(btDroite);
    const bool setupLeft = digitalRead(btGauche);
    const bool setupEnd = digitalRead(btSetup); // Plus petite prio
    if(setupUp || setupDown || btPlusLastState || btMoinsLastState){ // Détection haut ou bas
      // Gestion des boutons haut et bas avec une orientation sur la fluidité d'utilisation (asynchrone avec l'itération de la boucle loop)
      // Cela permet d'appuyer sur le bouton quand on veut et pouvoir rester appuyer sans que ça bouger trop rapidement. C'est une sorte de cycle d'hystérésis
      if(btPlusLastState){
        if(setupUp){ // Cas où on garde le bouton appuyé
          const unsigned long periodTmp = FirstRegDone ? BtRegPeriod : BtRegFirstPeriod; // Choix de la période différent si c'est la première
          if(now - periodTmp >= BtLastReg){ // Itération tous les 100 ms (valeur initiale)
            BtLastReg = now;
            isSetting = modifyIsSetting(selected, true, isSetting); // Seulement dans ce cas on modifie, sinon rien
            if(!FirstRegDone){FirstRegDone = true;}
          }
        }
        else {
          btPlusLastState = false;  // Bouton relaché
          BtLastReg = 0;  // Permet des appui rapide
          FirstRegDone = false;
        }
      }
      else if(btMoinsLastState){
        if(setupDown){ // Cas où on garde le bouton appuyé
          const unsigned long periodTmp = FirstRegDone ? BtRegPeriod : BtRegFirstPeriod; // Choix de la période différent si c'est la première
          if(now - periodTmp >= BtLastReg){
            BtLastReg = now;
            isSetting = modifyIsSetting(selected, false, isSetting);
            if(!FirstRegDone){FirstRegDone = true;}
          }
        }
        else {
          btMoinsLastState = false;
          BtLastReg = 0;
          FirstRegDone = false;
        }
      }
      else { // cas premier appui
        isSetting = modifyIsSetting(selected, setupUp, isSetting);
        BtLastReg = now;
        if(setupUp){
          btPlusLastState = true;
        }
        else{
          btMoinsLastState = true;
        }
      }
    }
    else if(setupRight || setupLeft || LRHysteresis){
      if(!LRHysteresis && (setupRight || setupLeft)){ // Detection du premier appui
        if((selected + setupRight < 7) && (selected - setupLeft >= 0)){ // on test pour ne pas sortir de [0;6]
          saveSetup(selected, isSetting); // on sauvegarde la valeur qu'on vient d'ajutster
          selected = setupRight ? selected + 1 : selected - 1; // on déplace le curseur
          isSetting = setupSelectNew(selected, newTimeData);
        }
        LRHysteresis = true; // Activation du cycle d'hystérésis
      }
      else if(LRHysteresis && !setupRight && !setupLeft){
        LRHysteresis = false; // Désactivation du cycle d'hystérésis
      }
    }
    else if(setupEnd){
      saveSetup(selected, isSetting); // on sauvegarde la valeur qui est sélectionnée
      setupIsInit = false; // reset de l'initialisation du mode setup
      mode = 0; // on passe en mode normal
      RTCFlag = true; // on force un flag d'affichage normal
      bool setupSwitchStillUp = digitalRead(btSetup); // On attend d'avoir relacher le bouton setup pour sortir, pour ne pas avoir une interruptions et revenir directement dans mode 1
      refreshNextAlarmDislpay = true;
      while(setupSwitchStillUp){
        delay(10);
        setupSwitchStillUp = digitalRead(btSetup);
      }
      PCICR = (1 << PCIE1) | (1 << PCIE2); // Réactiver les interruptions à la fin
      return; // on force le redemarrage de loop()
    }

    // test cligno
    if(now - lastBlink > blinkTime){
      lastBlink = now;
      setupIsBlinking = !setupIsBlinking;
    }
    switch(selected){
      case 0:{ // heures
        myMAX7219_1.send(MAXAddress1[3], NumeralCode1[(isSetting & 0b00110000)>>4]);
        myMAX7219_1.send(MAXAddress1[2], NumeralCode1[isSetting & 0b00001111]);
        if(setupIsBlinking){
          myMAX7219_1.send(MAXAddress1[1], NumeralCode1[(newTimeData[1] & 0b01110000)>>4]);
          myMAX7219_1.send(MAXAddress1[0], NumeralCode1[newTimeData[1] & 0b00001111]);
          myMAX7219_1.send(MAXAddress1[4], 0);
          myMAX7219_2.send(MAXAddress2[3], NumeralCode2[(newTimeData[0] & 0b01110000)>>4]);
          myMAX7219_2.send(MAXAddress2[2], NumeralCode2[newTimeData[0] & 0b00001111]);
          myMAX7219_2.send(MAXAddress2[1], 0);
          myMAX7219_2.send(MAXAddress2[0], 0);
          myMAX7219_2.send(MAXAddress2[4], 0);
        }
        else{
          myMAX7219_1.send(MAXAddress1[1], 0);
          myMAX7219_1.send(MAXAddress1[0], 0);
          myMAX7219_1.send(MAXAddress1[4], 0);
          myMAX7219_2.send(MAXAddress2[3], 0);
          myMAX7219_2.send(MAXAddress2[2], 0);
          myMAX7219_2.send(MAXAddress2[1], 0);
          myMAX7219_2.send(MAXAddress2[0], 0);
          myMAX7219_2.send(MAXAddress2[4], 0);
        }
        break;
      }
      case 1:{ // minutes
        myMAX7219_1.send(MAXAddress1[1], NumeralCode1[(isSetting & 0b01110000)>>4]);
        myMAX7219_1.send(MAXAddress1[0], NumeralCode1[isSetting & 0b00001111]);
        if(setupIsBlinking){
          myMAX7219_1.send(MAXAddress1[3], NumeralCode1[(newTimeData[2] & 0b00110000)>>4]);
          myMAX7219_1.send(MAXAddress1[2], NumeralCode1[newTimeData[2] & 0b00001111]);
          myMAX7219_1.send(MAXAddress1[4], 0);
          myMAX7219_2.send(MAXAddress2[3], NumeralCode2[(newTimeData[0] & 0b01110000)>>4]);
          myMAX7219_2.send(MAXAddress2[2], NumeralCode2[newTimeData[0] & 0b00001111]);
          myMAX7219_2.send(MAXAddress2[1], 0);
          myMAX7219_2.send(MAXAddress2[0], 0);
          myMAX7219_2.send(MAXAddress2[4], 0);
        }
        else{
            myMAX7219_1.send(MAXAddress1[3], 0);
            myMAX7219_1.send(MAXAddress1[2], 0);
            myMAX7219_1.send(MAXAddress1[4], 0);
            myMAX7219_2.send(MAXAddress2[3], 0);
            myMAX7219_2.send(MAXAddress2[2], 0);
            myMAX7219_2.send(MAXAddress2[1], 0);
            myMAX7219_2.send(MAXAddress2[0], 0);
            myMAX7219_2.send(MAXAddress2[4], 0);
        }
        break;
      }
      case 2:{ // secondes
        myMAX7219_2.send(MAXAddress2[3], NumeralCode2[(isSetting & 0b01110000)>>4]);
        myMAX7219_2.send(MAXAddress2[2], NumeralCode2[isSetting & 0b00001111]);
        if(setupIsBlinking){
          myMAX7219_1.send(MAXAddress1[3], NumeralCode1[(newTimeData[2] & 0b00110000)>>4]);
          myMAX7219_1.send(MAXAddress1[2], NumeralCode1[newTimeData[2] & 0b00001111]);
          myMAX7219_1.send(MAXAddress1[1], NumeralCode1[(newTimeData[1] & 0b01110000)>>4]);
          myMAX7219_1.send(MAXAddress1[0], NumeralCode1[newTimeData[1] & 0b00001111]);
          myMAX7219_1.send(MAXAddress1[4], 0);
          myMAX7219_2.send(MAXAddress2[1], 0);
          myMAX7219_2.send(MAXAddress2[0], 0);
          myMAX7219_2.send(MAXAddress2[4], 0);
        }
        else{
          myMAX7219_1.send(MAXAddress1[3], 0);
          myMAX7219_1.send(MAXAddress1[2], 0);
          myMAX7219_1.send(MAXAddress1[1], 0);
          myMAX7219_1.send(MAXAddress1[0], 0);
          myMAX7219_1.send(MAXAddress1[4], 0);
          myMAX7219_2.send(MAXAddress2[1], 0);
          myMAX7219_2.send(MAXAddress2[0], 0);
          myMAX7219_2.send(MAXAddress2[4], 0);
        }
        break;
      }
      case 3:{ // jour
        myMAX7219_1.send(MAXAddress1[3], NumeralCode1[(isSetting & 0b00110000)>>4]);
        myMAX7219_1.send(MAXAddress1[2], NumeralCode1[isSetting & 0b00001111]);
        if(setupIsBlinking){
          myMAX7219_1.send(MAXAddress1[1], NumeralCode1[(newTimeData[5] & 0b01110000)>>4]);
          myMAX7219_1.send(MAXAddress1[0], NumeralCode1[newTimeData[5] & 0b00001111]);
          myMAX7219_1.send(MAXAddress1[4], 0);
          myMAX7219_2.send(MAXAddress2[3], NumeralCode2[2]);
          myMAX7219_2.send(MAXAddress2[2], NumeralCode2[0]);
          myMAX7219_2.send(MAXAddress2[1], NumeralCode2[(newTimeData[6] & 0b11110000)>>4]);
          myMAX7219_2.send(MAXAddress2[0], NumeralCode2[newTimeData[6] & 0b00001111]);
          myMAX7219_2.send(MAXAddress2[4], 0);
        }
        else{
          myMAX7219_1.send(MAXAddress1[1], 0);
          myMAX7219_1.send(MAXAddress1[0], 0);
          myMAX7219_1.send(MAXAddress1[4], 0);
          myMAX7219_2.send(MAXAddress2[3], 0);
          myMAX7219_2.send(MAXAddress2[2], 0);
          myMAX7219_2.send(MAXAddress2[1], 0);
          myMAX7219_2.send(MAXAddress2[0], 0);
          myMAX7219_2.send(MAXAddress2[4], 0);
        }
        break;
      }
      case 4:{ // mois
        myMAX7219_1.send(MAXAddress1[1], NumeralCode1[(isSetting & 0b00010000)>>4]);
        myMAX7219_1.send(MAXAddress1[0], NumeralCode1[isSetting & 0b00001111]);
        if(setupIsBlinking){
          myMAX7219_1.send(MAXAddress1[3], NumeralCode1[(newTimeData[4] & 0b00110000)>>4]);
          myMAX7219_1.send(MAXAddress1[2], NumeralCode1[newTimeData[4] & 0b00001111]);
          myMAX7219_1.send(MAXAddress1[4], 0);
          myMAX7219_2.send(MAXAddress2[3], NumeralCode2[2]);
          myMAX7219_2.send(MAXAddress2[2], NumeralCode2[0]);
          myMAX7219_2.send(MAXAddress2[1], NumeralCode2[(newTimeData[6] & 0b11110000)>>4]);
          myMAX7219_2.send(MAXAddress2[0], NumeralCode2[newTimeData[6] & 0b00001111]);
          myMAX7219_2.send(MAXAddress2[4], 0);
        }
        else{
          myMAX7219_1.send(MAXAddress1[3], 0);
          myMAX7219_1.send(MAXAddress1[2], 0);
          myMAX7219_1.send(MAXAddress1[4], 0);
          myMAX7219_2.send(MAXAddress2[3], 0);
          myMAX7219_2.send(MAXAddress2[2], 0);
          myMAX7219_2.send(MAXAddress2[1], 0);
          myMAX7219_2.send(MAXAddress2[0], 0);
          myMAX7219_2.send(MAXAddress2[4], 0);
        }
        break;
      }
      case 5:{ // année
        myMAX7219_2.send(MAXAddress2[3], NumeralCode2[2]);
        myMAX7219_2.send(MAXAddress2[2], NumeralCode2[0]);
        myMAX7219_2.send(MAXAddress2[1], NumeralCode2[(isSetting & 0b11110000)>>4]);
        myMAX7219_2.send(MAXAddress2[0], NumeralCode2[isSetting & 0b00001111]);
        if(setupIsBlinking){
          myMAX7219_1.send(MAXAddress1[3], NumeralCode1[(newTimeData[4] & 0b00110000)>>4]);
          myMAX7219_1.send(MAXAddress1[2], NumeralCode1[newTimeData[4] & 0b00001111]);
          myMAX7219_1.send(MAXAddress1[1], NumeralCode1[(newTimeData[5] & 0b01110000)>>4]);
          myMAX7219_1.send(MAXAddress1[0], NumeralCode1[newTimeData[5] & 0b00001111]);
          myMAX7219_1.send(MAXAddress1[4], 0);
          myMAX7219_2.send(MAXAddress2[4], 0);
        }
        else{
          myMAX7219_1.send(MAXAddress1[3], 0);
          myMAX7219_1.send(MAXAddress1[2], 0);
          myMAX7219_1.send(MAXAddress1[1], 0);
          myMAX7219_1.send(MAXAddress1[0], 0);
          myMAX7219_1.send(MAXAddress1[4], 0);
          myMAX7219_2.send(MAXAddress2[4], 0);
        }
        break;
      }
      case 6:{ // jour de la semaine
        myMAX7219_2.send(MAXAddress2[4], dayLedsCompute(1<<(isSetting-1)));
        if(setupIsBlinking){
          myMAX7219_1.send(MAXAddress1[3], NumeralCode1[(newTimeData[4] & 0b00110000)>>4]);
          myMAX7219_1.send(MAXAddress1[2], NumeralCode1[newTimeData[4] & 0b00001111]);
          myMAX7219_1.send(MAXAddress1[1], NumeralCode1[(newTimeData[5] & 0b01110000)>>4]);
          myMAX7219_1.send(MAXAddress1[0], NumeralCode1[newTimeData[5] & 0b00001111]);
          myMAX7219_1.send(MAXAddress1[4], 0);
          myMAX7219_2.send(MAXAddress2[3], NumeralCode2[2]);
          myMAX7219_2.send(MAXAddress2[2], NumeralCode2[0]);
          myMAX7219_2.send(MAXAddress2[1], NumeralCode2[(newTimeData[6] & 0b11110000)>>4]);
          myMAX7219_2.send(MAXAddress2[0], NumeralCode2[newTimeData[6] & 0b00001111]);
        }
        else{
          myMAX7219_1.send(MAXAddress1[3], 0);
          myMAX7219_1.send(MAXAddress1[2], 0);
          myMAX7219_1.send(MAXAddress1[1], 0);
          myMAX7219_1.send(MAXAddress1[0], 0);
          myMAX7219_1.send(MAXAddress1[4], 0);
          myMAX7219_2.send(MAXAddress2[3], 0);
          myMAX7219_2.send(MAXAddress2[2], 0);
          myMAX7219_2.send(MAXAddress2[1], 0);
          myMAX7219_2.send(MAXAddress2[0], 0);
        }
        break;
      }
    }
    delay(10);
  }
  else if(mode == 2){ //-----------------------------------------menu reveil
    if(!settingAlarm){ // Test mode selection ou creation/modification
      // Mode selection
      if(!menuReveilInit){
        menuReveilInit = true;
        nbAlarm = AT24C32::readAll(AlarmNU, AlarmU);
        delay(50);
        selectedAlarm = 1; // Première alarme
        selectedType = false; // Alarme de type non unique
        confirmDelete = false;
        PCICR = 0; // Désactiver les interruptions pendant le setup
        // Cycle hystérésis du bouton droite (/!\ stop le programme, mais ça n'est pas particulièrement important);
        bool setupSwitchStillUp = digitalRead(btDroite);
        while(setupSwitchStillUp){
          delay(10);
          setupSwitchStillUp = digitalRead(btDroite);
        }
      }
      
      // partie bouton
      const bool setupUp = digitalRead(btPlus); // Plus grande prio
      const bool setupDown = digitalRead(btMoins);
      const bool setupRight = digitalRead(btDroite);
      const bool setupLeft = digitalRead(btGauche);
      const bool setupEnd = digitalRead(btSetup); // Plus petite prio
      if(setupLeft || btGaucheLastState){
        if(!btGaucheLastState){
          btGaucheLastState = true;
          selectedType = !selectedType;
          selectedAlarm = 1;
        }
        else if(btGaucheLastState && !setupLeft){
          btGaucheLastState = false;
        }
      }
      else if(setupRight || btDroiteLastState){
        if(!btDroiteLastState){
          btDroiteLastState = true;
          uint8_t maxSelectedAlarm = selectedType ? nbAlarm & 0b00001111 : nbAlarm >> 4;
          if(selectedAlarm != 0){
            if(selectedAlarm < maxSelectedAlarm + 1){
              //activer, désactiver
              if(!selectedType){
                AlarmNU[(selectedAlarm - 1) * 3] ^= 0b10000000;
                uint8_t buffer[3];
                for(uint8_t i = 0; i < 3; i++){
                  buffer[i] = AlarmNU[(selectedAlarm - 1) * 3 + i];
                }
                AT24C32::modifyNU(selectedAlarm - 1, buffer);
              }
              else{
                AlarmU[(selectedAlarm - 1) * 2] ^= 0b10000000;
                uint8_t buffer[2];
                for(uint8_t i = 0; i < 2; i++){
                  buffer[i] = AlarmU[(selectedAlarm - 1) * 2 + i];
                }
                AT24C32::modifyU(selectedAlarm - 1, buffer);
              }
              delay(20);
              //nbAlarm = AT24C32::readAll(AlarmNU, AlarmU);
            }
          }
          else{ // Escape alarm menu
            menuReveilInit = false; // reset de l'initialisation du mode menu reveil
            mode = 0; // on passe en mode normal
            RTCFlag = true; // on force un flag d'affichage normal
            nbAlarm = AT24C32::readAll(AlarmNU, AlarmU); // Refresh nbAlarm : number of stored alarm
            nbActive = AT24C32::readActive(ActiveNU, ActiveU); // Refresh nbActive : number of stored alarm which are active/enable
            if(nbActive != 0){
              nextAlarmIndex = findNextActiveAlarm(timeData, ActiveNU, (nbActive & 0xf0)>>4, ActiveU, nbActive & 0x0f); // Looking for next alarmIndex
              displayAlarm(nextAlarmIndex, ActiveNU, ActiveU, displayNextAlarm); // Formating displayNextAlarm
            }
            refreshNextAlarmDislpay = true; // Force NextAlarm display on the next mode 0 frame
            bool setupSwitchStillUp = digitalRead(btDroite); // On attend d'avoir relacher le bouton setup pour sortir, pour ne pas avoir une interruptions et revenir directement dans mode 1
            while(setupSwitchStillUp){
              delay(10);
              setupSwitchStillUp = digitalRead(btDroite);
            }
            PCICR = (1 << PCIE1) | (1 << PCIE2); // Réactiver les interruptions à la fin
            return; // on force le redemarrage de loop()
          }
        }
        else if(btDroiteLastState && !setupRight){
          btDroiteLastState = false;
        }
      }
      else if(setupUp || setupDown || UDHysteresis){
        if(!UDHysteresis){
          UDHysteresis = true;
          uint8_t maxSelectedAlarm = selectedType ? nbAlarm & 0b00001111 : nbAlarm >> 4;
          if((selectedAlarm + setupDown <= maxSelectedAlarm + 1) && (selectedAlarm - setupUp >= 0)){ // on test pour ne pas sortir; + 1 due to esc- mode and new alarm
            selectedAlarm = selectedAlarm - setupUp + setupDown; // setupUP goes down in the list and setupDown goes up
          }
        }
        else if(UDHysteresis && !setupUp && !setupDown){
          UDHysteresis = false;
        }
      }
      else if(setupEnd && (selectedAlarm != 0)){
        menuReveilInit = false; // reset de l'initialisation du mode menu reveil
        settingAlarm = true; // Going to Alarm setting mode
        bool setupSwitchStillUp = digitalRead(btSetup); // On attend d'avoir relacher le bouton setup pour sortir, pour ne pas avoir une interruptions et revenir directement dans mode 1
        while(setupSwitchStillUp){
          delay(10);
          setupSwitchStillUp = digitalRead(btSetup);
        }
      }

      //display
      if(selectedAlarm == 0){ // ecran retour
        myMAX7219_1.send(MAXAddress1[3], 0b11001110); // E
        myMAX7219_1.send(MAXAddress1[2], 0b10101110); // S
        myMAX7219_1.send(MAXAddress1[1], 0b11000110); // C
        myMAX7219_1.send(MAXAddress1[0], 0b00001000); // -
        myMAX7219_1.send(MAXAddress1[4], 0);
        myMAX7219_2.send(MAXAddress2[3], 0);
        myMAX7219_2.send(MAXAddress2[2], 0);
        myMAX7219_2.send(MAXAddress2[1], 0);
        myMAX7219_2.send(MAXAddress2[0], 0);
        myMAX7219_2.send(MAXAddress2[4], 0);
      }
      else{
        if(!selectedType){ // alarme de type non unique
          if((selectedAlarm) <= ((nbAlarm & 0xf0)>>4)){ // Normal display, else : new alarm proposal
            alarmIsNew = false;
            myMAX7219_1.send(MAXAddress1[3], NumeralCode1[(AlarmNU[(selectedAlarm - 1) * 3 + 1] & 0b00110000)>>4]);
            myMAX7219_1.send(MAXAddress1[2], NumeralCode1[AlarmNU[(selectedAlarm - 1) * 3 + 1] & 0b00001111]);
            myMAX7219_1.send(MAXAddress1[1], NumeralCode1[(AlarmNU[(selectedAlarm - 1) * 3] & 0b01110000)>>4]);
            myMAX7219_1.send(MAXAddress1[0], NumeralCode1[AlarmNU[(selectedAlarm - 1) * 3] & 0b00001111]);
            myMAX7219_1.send(MAXAddress1[4], (AlarmNU[(selectedAlarm - 1) * 3] & 0b10000000)>>4); // Alarm Activated indicator on ddot
            myMAX7219_2.send(MAXAddress2[4], dayLedsCompute(AlarmNU[(selectedAlarm - 1) * 3 + 2]));
            if(!confirmDelete){
              myMAX7219_2.send(MAXAddress2[3], NumeralCode2[selectedAlarm / 10]);
              myMAX7219_2.send(MAXAddress2[2], NumeralCode2[selectedAlarm % 10]);
              myMAX7219_2.send(MAXAddress2[1], 0b00001000); // -
              myMAX7219_2.send(MAXAddress2[0], NumeralCode2[0]); // 0 = Non unique alarm
            }
            else{
              myMAX7219_2.send(MAXAddress2[3], 0b10101101); // S
              myMAX7219_2.send(MAXAddress2[2], 0b10110011); // U
              myMAX7219_2.send(MAXAddress2[1], 0b10011110); // P
              myMAX7219_2.send(MAXAddress2[0], 0b00001000); // -
            }
          }
          else{
            alarmIsNew = true;
            myMAX7219_1.send(MAXAddress1[3], 0b00001000); // -
            myMAX7219_1.send(MAXAddress1[2], 0);
            myMAX7219_1.send(MAXAddress1[1], 0);
            myMAX7219_1.send(MAXAddress1[0], 0);
            myMAX7219_1.send(MAXAddress1[4], 0);
            myMAX7219_2.send(MAXAddress2[3], NumeralCode2[selectedAlarm / 10]);
            myMAX7219_2.send(MAXAddress2[2], NumeralCode2[selectedAlarm % 10]);
            myMAX7219_2.send(MAXAddress2[1], 0b00001000); // -
            myMAX7219_2.send(MAXAddress2[0], NumeralCode2[0]);
            myMAX7219_2.send(MAXAddress2[4], 0);
          }
        }
        else{ // alarme de type unique
          if((selectedAlarm) <= (nbAlarm & 0x0f)){ // affichage classique, sinon propo nouvelle alarme
            alarmIsNew = false;
            myMAX7219_1.send(MAXAddress1[3], NumeralCode1[(AlarmU[(selectedAlarm - 1) * 2 + 1] & 0b00110000)>>4]);
            myMAX7219_1.send(MAXAddress1[2], NumeralCode1[AlarmU[(selectedAlarm - 1) * 2 + 1] & 0b00001111]);
            myMAX7219_1.send(MAXAddress1[1], NumeralCode1[(AlarmU[(selectedAlarm - 1) * 2] & 0b01110000)>>4]);
            myMAX7219_1.send(MAXAddress1[0], NumeralCode1[AlarmU[(selectedAlarm - 1) * 2] & 0b00001111]);
            myMAX7219_1.send(MAXAddress1[4], (AlarmU[(selectedAlarm - 1) * 2] & 0b10000000)>>4); // Alarm Activated indicator on ddot
            myMAX7219_2.send(MAXAddress2[4], 0);
            if(!confirmDelete){
              myMAX7219_2.send(MAXAddress2[3], NumeralCode2[selectedAlarm / 10]);
              myMAX7219_2.send(MAXAddress2[2], NumeralCode2[selectedAlarm % 10]);
              myMAX7219_2.send(MAXAddress2[1], 0b00001000); // -
              myMAX7219_2.send(MAXAddress2[0], NumeralCode2[1]); // 1 = Unique alarm
            }
            else{
              myMAX7219_2.send(MAXAddress2[3], 0b10101101); // S
              myMAX7219_2.send(MAXAddress2[2], 0b10110011); // U
              myMAX7219_2.send(MAXAddress2[1], 0b10011110); // P
              myMAX7219_2.send(MAXAddress2[0], 0b00001000); // -
            }
          }
          else{
            alarmIsNew = true;
            myMAX7219_1.send(MAXAddress1[3], 0b00001000); // -
            myMAX7219_1.send(MAXAddress1[2], 0);
            myMAX7219_1.send(MAXAddress1[1], 0);
            myMAX7219_1.send(MAXAddress1[0], 0);
            myMAX7219_1.send(MAXAddress1[4], 0);
            myMAX7219_2.send(MAXAddress2[3], NumeralCode2[selectedAlarm / 10]);
            myMAX7219_2.send(MAXAddress2[2], NumeralCode2[selectedAlarm % 10]);
            myMAX7219_2.send(MAXAddress2[1], 0b00001000); // -
            myMAX7219_2.send(MAXAddress2[0], NumeralCode2[1]); // 1 = Unique alarm
            myMAX7219_2.send(MAXAddress2[4], 0);
          }
        }
      }
    }
    else{ // mode creation/modification d'alarme (proche du mode setup)
      if(!settingAlarmInit){
        settingAlarmInit = true;
        if(!alarmIsNew){ // detection d'une nouvelle alarme
          newTimeData[0] = selectedType ? AlarmU[(selectedAlarm - 1) * 3] : AlarmNU[(selectedAlarm - 1) * 3];
          newTimeData[1] = selectedType ? AlarmU[(selectedAlarm - 1) * 3 + 1] : AlarmNU[(selectedAlarm - 1) * 3 + 1];
          newTimeData[2] = selectedType ? 0 : AlarmNU[(selectedAlarm - 1) * 3 + 2];
        }
        else{ // New alarms set at 12h30 and armed
          newTimeData[0] = (3 << 4) & 0b10000000;
          newTimeData[1] = (1 << 4) | 2;
          newTimeData[2] = 0;
        }
        selected = 0;
        isSetting = setupAlarmSelectNew(selected, newTimeData);
        setupIsBlinking = false;
      }
      
      unsigned long now = millis();
      
      // partie bouton
      const bool setupUp = digitalRead(btPlus); // Plus grande prio
      const bool setupDown = digitalRead(btMoins);
      const bool setupRight = digitalRead(btDroite);
      const bool setupLeft = digitalRead(btGauche);
      const bool setupEnd = digitalRead(btSetup); // Plus petite prio
      if(setupUp || setupDown || btPlusLastState || btMoinsLastState){ // Détection haut ou bas
        // Gestion des boutons haut et bas avec une orientation sur la fluidité d'utilisation (asynchrone avec l'itération de la boucle loop)
        // Cela permet d'appuyer sur le bouton quand on veut et pouvoir rester appuyer sans que ça bouger trop rapidement. C'est une sorte de cycle d'hystérésis
        if(selected < 2){
          if(btPlusLastState){
            if(setupUp){ // Cas où on garde le bouton appuyé
              const unsigned long periodTmp = FirstRegDone ? BtRegPeriod : BtRegFirstPeriod; // Choix de la période différent si c'est la première
              if(now - periodTmp >= BtLastReg){ // Itération tous les 100 ms (valeur initiale)
                BtLastReg = now;
                isSetting = modifyAlarmIsSetting(selected, true, isSetting); // Seulement dans ce cas on modifie, sinon rien
                if(!FirstRegDone){FirstRegDone = true;}
              }
            }
            else{
              btPlusLastState = false;  // Bouton relaché
              BtLastReg = 0;  // Permet des appui rapide
              FirstRegDone = false;
            }
          }
          else if(btMoinsLastState){
            if(setupDown){ // Cas où on garde le bouton appuyé
              const unsigned long periodTmp = FirstRegDone ? BtRegPeriod : BtRegFirstPeriod; // Choix de la période différent si c'est la première
              if(now - periodTmp >= BtLastReg){
                BtLastReg = now;
                isSetting = modifyAlarmIsSetting(selected, false, isSetting);
                if(!FirstRegDone){FirstRegDone = true;}
              }
            }
            else{
              btMoinsLastState = false;
              BtLastReg = 0;
              FirstRegDone = false;
            }
          }
          else{ // cas premier appui
            isSetting = modifyAlarmIsSetting(selected, setupUp, isSetting);
            BtLastReg = now;
            if(setupUp){
              btPlusLastState = true;
            }
            else{
              btMoinsLastState = true;
            }
          }
        }
        else{
          if(btPlusLastState && !setupUp){ // Releasing up switch
            btPlusLastState = false;
          }
          else if(btMoinsLastState && !setupDown){
            btMoinsLastState = false;
          }
          else if(!btPlusLastState && !btMoinsLastState){
            if(setupUp){
              isSetting |= 1 << (selected - 2);
              btPlusLastState = true;
            }
            else{
              isSetting &= ~(1 << (selected - 2));
            btMoinsLastState = true;
            }
          }
        }
      }
      else if(setupRight || setupLeft || LRHysteresis){
        if(!LRHysteresis && (setupRight || setupLeft)){ // Detection du premier appui
          uint8_t maxSelected = !selectedType ? 9 : 2;
          if((selected + setupRight < maxSelected) && (selected - setupLeft >= 0)){ // on test pour ne pas sortir de [0;6]
            saveAlarm(selected, isSetting, newTimeData);
            if(!selectedType){
              AT24C32::modifyNU(selectedAlarm - 1, newTimeData);
            }
            else{
              AT24C32::modifyU(selectedAlarm - 1, newTimeData);
            }
            selected = setupRight ? selected + 1 : selected - 1; // on déplace le curseur
            isSetting = setupAlarmSelectNew(selected, newTimeData);
          }
          LRHysteresis = true; // Activation du cycle d'hystérésis
        }
        else if(LRHysteresis && !setupRight && !setupLeft){
          LRHysteresis = false; // Désactivation du cycle d'hystérésis
        }
      }
      else if(setupEnd){
        saveAlarm(selected, isSetting, newTimeData);
        if(!selectedType){
          AT24C32::modifyNU(selectedAlarm - 1, newTimeData);
        }
        else{
          AT24C32::modifyU(selectedAlarm - 1, newTimeData);
        }
        settingAlarmInit = false; // reset de l'initialisation du mode setup
        settingAlarm = false;
        bool setupSwitchStillUp = digitalRead(btSetup); // On attend d'avoir relacher le bouton setup pour sortir, pour ne pas avoir une interruptions et revenir directement dans mode 1
        while(setupSwitchStillUp){
          delay(10);
          setupSwitchStillUp = digitalRead(btSetup);
        }
        return; // on force le redemarrage de loop()
      }

      if(now - lastBlink > blinkTime){
        lastBlink = now;
        setupIsBlinking = !setupIsBlinking;
      }

      switch(selected){
        case 0:{ // heure
          myMAX7219_1.send(MAXAddress1[3], NumeralCode1[(isSetting & 0b00110000)>>4]);
          myMAX7219_1.send(MAXAddress1[2], NumeralCode1[isSetting & 0b00001111]);
          if(setupIsBlinking){
            myMAX7219_1.send(MAXAddress1[1], NumeralCode1[(newTimeData[0] & 0b01110000)>>4]);
            myMAX7219_1.send(MAXAddress1[0], NumeralCode1[newTimeData[0] & 0b00001111]);
            myMAX7219_1.send(MAXAddress1[4], 0);
            myMAX7219_2.send(MAXAddress2[3], 0);
            myMAX7219_2.send(MAXAddress2[2], 0);
            myMAX7219_2.send(MAXAddress2[1], 0);
            myMAX7219_2.send(MAXAddress2[0], 0);
            myMAX7219_2.send(MAXAddress2[4], dayLedsCompute(newTimeData[2]));
          }
          else{
            myMAX7219_1.send(MAXAddress1[1], 0);
            myMAX7219_1.send(MAXAddress1[0], 0);
            myMAX7219_1.send(MAXAddress1[4], 0);
            myMAX7219_2.send(MAXAddress2[3], 0);
            myMAX7219_2.send(MAXAddress2[2], 0);
            myMAX7219_2.send(MAXAddress2[1], 0);
            myMAX7219_2.send(MAXAddress2[0], 0);
            myMAX7219_2.send(MAXAddress2[4], 0);
          }
          break;
        }
        case 1:{ // minutes
          myMAX7219_1.send(MAXAddress1[1], NumeralCode1[(isSetting & 0b01110000)>>4]);
          myMAX7219_1.send(MAXAddress1[0], NumeralCode1[isSetting & 0b00001111]);
          if(setupIsBlinking){
            myMAX7219_1.send(MAXAddress1[3], NumeralCode1[(newTimeData[1] & 0b00110000)>>4]);
            myMAX7219_1.send(MAXAddress1[2], NumeralCode1[newTimeData[1] & 0b00001111]);
            myMAX7219_1.send(MAXAddress1[4], 0);
            myMAX7219_2.send(MAXAddress2[3], 0);
            myMAX7219_2.send(MAXAddress2[2], 0);
            myMAX7219_2.send(MAXAddress2[1], 0);
            myMAX7219_2.send(MAXAddress2[0], 0);
            myMAX7219_2.send(MAXAddress2[4], dayLedsCompute(newTimeData[2]));
          }
          else{
            myMAX7219_1.send(MAXAddress1[3], 0);
            myMAX7219_1.send(MAXAddress1[2], 0);
            myMAX7219_1.send(MAXAddress1[4], 0);
            myMAX7219_2.send(MAXAddress2[3], 0);
            myMAX7219_2.send(MAXAddress2[2], 0);
            myMAX7219_2.send(MAXAddress2[1], 0);
            myMAX7219_2.send(MAXAddress2[0], 0);
            myMAX7219_2.send(MAXAddress2[4], 0);
          }
          break;
        }
        default:{ // jour de la semaine, pour les alarmes unique on a pas besoin de ça (test réalisé en amont)
            myMAX7219_2.send(MAXAddress2[3], NumeralCode2[selected - 1]);
            myMAX7219_2.send(MAXAddress2[4], dayLedsCompute(isSetting));
          if(setupIsBlinking){
            myMAX7219_1.send(MAXAddress1[3], NumeralCode1[(newTimeData[1] & 0b00110000)>>4]);
            myMAX7219_1.send(MAXAddress1[2], NumeralCode1[newTimeData[1] & 0b00001111]);
            myMAX7219_1.send(MAXAddress1[1], NumeralCode1[(newTimeData[0] & 0b01110000)>>4]);
            myMAX7219_1.send(MAXAddress1[0], NumeralCode1[newTimeData[0] & 0b00001111]);
            myMAX7219_1.send(MAXAddress1[4], 0);
            myMAX7219_2.send(MAXAddress2[2], 0);
            myMAX7219_2.send(MAXAddress2[1], 0);
            myMAX7219_2.send(MAXAddress2[0], 0);
          }
          else{
            myMAX7219_1.send(MAXAddress1[3], 0);
            myMAX7219_1.send(MAXAddress1[2], 0);
            myMAX7219_1.send(MAXAddress1[1], 0);
            myMAX7219_1.send(MAXAddress1[0], 0);
            myMAX7219_1.send(MAXAddress1[4], 0);
            myMAX7219_2.send(MAXAddress2[2], 0);
            myMAX7219_2.send(MAXAddress2[1], 0);
            myMAX7219_2.send(MAXAddress2[0], 0);
          }
          break;
        }
      }
    }
    delay(10);
  }
  else if(mode == 3){ //-----------------------------------------Alarm mode (audio)
    unsigned long now = millis();
    if(!isReset){
      if(!isAwake){
        PCICR = 0; // Deactivate interruptions
        myMAX7219_1.send(MAXAddress1[3], 0);  // Deactivate display during DFPlayer wake up
        myMAX7219_1.send(MAXAddress1[2], 0);
        myMAX7219_1.send(MAXAddress1[1], 0);
        myMAX7219_1.send(MAXAddress1[0], 0);
        myMAX7219_1.send(MAXAddress1[4], 0);

        myMAX7219_2.send(MAXAddress2[3], 0);
        myMAX7219_2.send(MAXAddress2[2], 0);
        myMAX7219_2.send(MAXAddress2[1], 0);
        myMAX7219_2.send(MAXAddress2[0], 0);
        myMAX7219_2.send(MAXAddress2[4], 0);
        DFPLAYER::wakeUp();
        alarmWakeUpTimer = now;
        isAwake = true;
      }

      if((now - alarmWakeUpTimer) > wakeUpTime){ // Cannot send an UART msg just after awaking the DFPlayer, need to wait
        DFPLAYER::wakeUpReset(); // Reset is for volume mainly
        delay(15); // Necessary or the first msg will not be send
        DFPLAYER::play(); // Start playing the alarm
        isReset = true;
        setupIsBlinking = false; // Start blinking
      }
    }
    else{

      const bool stopButton = digitalRead(btA);
      
      if(stopButton){ // End of alarm/mode 3
        DFPLAYER::pause(); // Stop the audio
        delay(15);
        DFPLAYER::toSleep(); // Turn the DFPlayer off

        isAwake = false; // Reset the alarm mode variables
        isReset = false;
        
        AlarmIsActive = false; // Alarm isn't active anymore // may be unnecessary
        nightMode = false; // Stopping nightmode if in nightmode (no changes otherwise)
        mode = 0;

        // Check new next alarm
        DS3231::readFullDate(timeData);
        nextAlarmIndex = findNextActiveAlarm(timeData, ActiveNU, (nbActive & 0xf0)>>4, ActiveU, nbActive & 0x0f);
        displayAlarm(nextAlarmIndex, ActiveNU, ActiveU, displayNextAlarm); // Compute displayable alarm data
        refreshNextAlarmDislpay = true;  // Refresh next alarm display in the next display loop

        bool setupSwitchStillUp = digitalRead(btA); // Waiting for the button to be realised and not having an interruption directly after in mode 0
        while(setupSwitchStillUp){
          delay(10);
          setupSwitchStillUp = digitalRead(btA);
        }
        PCICR = (1 << PCIE1) | (1 << PCIE2); // Reactivate interruptions
        return; // Force restart to the loop()
      }

      if(now - lastBlink > blinkTime){ // Checking if blinking
        lastBlink = now;
        setupIsBlinking = !setupIsBlinking;
      }

      DS3231::readDisplayTime(displayTime);
      // Whole display is blinking, if setupIsBlinking is 0, the it sends 0 to every display = off
      myMAX7219_1.send(MAXAddress1[3], setupIsBlinking * NumeralCode1[(displayTime[1] & 0b00110000)>>4]);
      myMAX7219_1.send(MAXAddress1[2], setupIsBlinking * NumeralCode1[displayTime[1] & 0b00001111]);
      myMAX7219_1.send(MAXAddress1[1], setupIsBlinking * NumeralCode1[(displayTime[0] & 0b01110000)>>4]);
      myMAX7219_1.send(MAXAddress1[0], setupIsBlinking * NumeralCode1[displayTime[0] & 0b00001111]);
      myMAX7219_1.send(MAXAddress1[4], setupIsBlinking<<3);

      myMAX7219_2.send(MAXAddress2[3], setupIsBlinking * NumeralCode2[(displayNextAlarm[1] & 0b00110000)>>4]);
      myMAX7219_2.send(MAXAddress2[2], setupIsBlinking * NumeralCode2[displayNextAlarm[1] & 0b00001111]);
      myMAX7219_2.send(MAXAddress2[1], setupIsBlinking * NumeralCode2[(displayNextAlarm[0] & 0b01110000)>>4]);
      myMAX7219_2.send(MAXAddress2[0], setupIsBlinking * NumeralCode2[displayNextAlarm[0] & 0b00001111]);
      myMAX7219_2.send(MAXAddress2[4], dayLedsCompute(setupIsBlinking * displayNextAlarm[2]));
    }
    delay(10);
  }
} //-------------------------------------------------------------end loop

// Interruption sur tout changement d’état de A3 (PCINT11)
ISR(PCINT1_vect) { //--------------------------------------------ISR
  bool currentState = digitalRead(A3);

  // Détection du front montant uniquement
  if (currentState == HIGH && RTCLastPinState == LOW) {
    RTCFlag = true;
  }

  RTCLastPinState = currentState;
}


ISR(PCINT2_vect) {
  if(digitalRead(btPlus)){
    if(brightness < 0x0F){
      brightness++;
      brightnessFlag = true;
    }
  }
  else if(digitalRead(btMoins)){
    if(brightness > 0x00){
      brightness--;
      brightnessFlag = true;
    }
  }
  else if(digitalRead(btA)){
    nightMode = !nightMode; // switch de l'affichage
    refreshNextAlarmDislpay = true; // on refresh l'affichage du prochain reveil (aucun impact quand nightMode == true)
    RTCFlag = true; // on force un affichage
  }
  else if(digitalRead(btSetup)){
    mode = 1;
  }
  else if(digitalRead(btDroite)){
    mode = 2;
  }
}

uint8_t dayLedsCompute(const uint8_t inputBit){
  return 
    ((inputBit & 0b00000001) << 4) +
    ((inputBit & 0b00000010) << 1) +
    ((inputBit & 0b00000100) << 5) +
    (inputBit & 0b00001000) +
    ((inputBit & 0b00010000) >> 4) +
    (inputBit & 0b00100000) +
    ((inputBit & 0b01000000) >> 5);
}


uint8_t AlarmSetupSelectNew(const uint8_t newSelected, const uint8_t* nTD){
  const uint8_t table[8] = {
    1, // heure
    0, // minute
    2// jour de la semaine
  };
  return nTD[table[newSelected]];
}

uint8_t modifyAlarmIsSetting(const uint8_t selectedParam, const bool setupUp, uint8_t data){
  const uint8_t bounds[2][2] = {
    {23, 0}, // heure entre 0 et 23
    {59, 0} // minute entre 0 et 59
  };
  if(selectedParam < 2){ // Hours and minutes part
    data = computeValue(data);
    if(setupUp){ // If true then going up
      data = (data == bounds[selectedParam][0]) ? bounds[selectedParam][1] : data + 1; // Testing bound
    }
    else{
      data = (data == bounds[selectedParam][1]) ? bounds[selectedParam][0] : data - 1; // Testing bound
    }
    return computeValueInv(data);
  }
  else {  // Day part
    return data ^ (1 << (selectedParam - 2));
  }
}

uint8_t setupSelectNew(const uint8_t newSelected, const uint8_t* nTD){
  const uint8_t table[7] = {
    2, // heure
    1, // minute
    0, // secondes
    4, // jour
    5, // mois
    6, // année
    3 // jour de la semaine
  };
  return nTD[table[newSelected]];
}

uint8_t setupAlarmSelectNew(const uint8_t newSelected, const uint8_t* nTD){ // Return a value for isSetting as selected 0 is not AlarmU[0]
  const uint8_t table[9] = {
    1, // heure
    0, // minute
    2, 2, 2, 2, 2, 2, 2 // jour de la semaine
  };
  return nTD[table[newSelected]];
}

void saveSetup(const uint8_t selectedParam, const uint8_t data){
  const uint8_t table[7] = {
    2, // heure
    1, // minute
    0, // secondes
    4, // jour
    5, // mois
    6, // année
    3 // jour de la semaine
  };
  DS3231::write1byte(table[selectedParam], data);
}

void saveAlarm(const uint8_t selectedParam, const uint8_t NewValue, uint8_t* nTD){
  const uint8_t table[9] = {
    1, // heure
    0, // minute
    2, 2, 2, 2, 2, 2, 2 // jour de la semaine
  };
  nTD[table[selectedParam]] = NewValue;
}

uint8_t modifyIsSetting(const uint8_t selectedParam, const bool setupUp, uint8_t data){
  const uint8_t bounds[7][2] = {
    {23, 0}, // heure entre 0 et 23
    {59, 0}, // minute entre 0 et 59
    {59, 0}, // secondes entre 0 et 59
    {31, 1}, // jour du mois entre 1 et 31
    {12, 1}, // mois entre 1 et 12
    {99, 0}, // année entre 0 et 99
    {7, 1} // jour de la semaine entre 1 et 7
  };
  data = computeValue(data); // + ou - 1 en fonction de setupUp
  if(setupUp){
    data = data == bounds[selectedParam][0] ? bounds[selectedParam][1] : data + 1; // test valeurs max
  }
  else{
    data = data == bounds[selectedParam][1] ? bounds[selectedParam][0] : data - 1; // test valeurs min
  }
  return computeValueInv(data);
}

uint8_t computeValue(const uint8_t value){ // Compute value 0b zzzz zzzz = 0b 0000 xxxx + 10 * (0b yyyy 0000 >> 4)
  return (value & 0b00001111) + (((value & 0b11110000)>>4)*10);
}

uint8_t computeValueInv(const uint8_t value){ // Compute value back to normal
  return (uint8_t(value / 10)<<4) | (uint8_t(value % 10));
}

bool checkAlarm(uint8_t* time, uint8_t* bufferNU, uint8_t nbNU, uint8_t* bufferU, uint8_t nbU){ // This function is a bit overkill
// but it separate findNextActiveAlarm from the alarm detection
  uint8_t i = 0;
  //si il y a plusieur alarme unique a une même heure, seulement la premiere sera desarme.
  while(i < nbU){ // Check for each alarm
    if(
      ((bufferU[i*2] & 0b01111111) == time[0]) &&
      (bufferU[i*2+1] == (time[1] & 0b00111111)) // Check time
    ){
      const uint8_t tmp[2] = {bufferU[i] & 0b011111111, bufferU[i+1]};
      //AT24C32::modifyU(i, tmp); // Deactivate U alarm after one use
      return true; // No need to go further
    }
    i++;
  }

  i = 0;
  while(i < nbNU){
    if(
      ((bufferNU[i*3] & 0b01111111) == time[0]) &&
      (bufferNU[i*3+1] == (time[1] & 0b00111111)) &&
      (bufferNU[i*3+2] & (0b1<< (time[2]-1))) // Check time and day of the week
    ){
      return true;
    }
    i++;
  }
  return false; // No alarm matching detected
}

void displayAlarm(uint8_t index, const uint8_t * bufferNU, const uint8_t* bufferU, uint8_t* display){
  if(index & 0b10000000){
    index &= 0b00001111;
    display[0] = bufferNU[(index - 1) * 3];
    display[1] = bufferNU[(index - 1) * 3 + 1];
    display[2] = bufferNU[(index - 1) * 3 + 2];
  }
  else{
    display[0] = bufferU[(index - 1) * 2];
    display[1] = bufferU[(index - 1) * 2 + 1];
    display[2] = 0x00;
  }
}

uint8_t findNextActiveAlarm(uint8_t* time, uint8_t* bufferNU, uint8_t nbNU, uint8_t* bufferU, uint8_t nbU){
  //boucle Unique
  int16_t current = toMin(time[0], time[1]) + 1; //pour ne pas afficher celui de la minute en cour.
  uint8_t index = 0; // no alarm
  int16_t minDiff = 10080; // minDiff set at 7day = 10080 min
  if(nbU){ // At least 1 U alarm
    for(uint8_t i = 0; i < nbU; i++){
      int16_t diff = computeDiff(current, toMin(bufferU[i*2], bufferU[(i*2)+1]));
      if(diff < minDiff){
        minDiff = diff;
        index = i + 1; // least is 1 (first alarm)
      }
    }
  }
  if(nbNU){ // At least 1 NU alarm
    for(uint8_t i = 0; i < nbNU; i++){
      int16_t diff = computeDiffNU(current, toMin(bufferNU[i*3], bufferNU[(i*3)+1]), time[2], bufferNU[(i*3)+2]);
      if(diff < minDiff){
        minDiff = diff;
        index = (i + 1) | 0b10000000; // 0B1xxxxxxx to indicate its an NU
      }
    }
  }
  return index;
}

int16_t toMin(uint8_t min, uint8_t hour){
  
  return ((
    (min & 0b00001111) +
    (((min & 0b01110000) >> 4) * 10) +
    ((hour & 0b00001111) * 60) +
    (((hour & 0b00110000) >> 4) * 600)
  ));
}

int16_t computeDiff(int16_t current, int16_t alarm){
  int16_t diff = alarm - current;
  if(diff < 0){ // If it is negative, then add one day (example -2h + 24h --> alarm in 22h) 
    diff += 1440;
  }
  return diff;
}

int16_t computeDiffNU(int16_t current, int16_t alarm, uint8_t currentDay, uint8_t alarmDays){
  int16_t currentTotal = current + (currentDay - 1) * 1440; // Total time from the start of the week
  int16_t minDiff = 10080; // Max time
  int16_t diff = 10080; // Not important
  while(alarmDays){ // While we still have days from the alarm to check
    diff = alarm + (__builtin_ctz(alarmDays) * 1440) - currentTotal < 0 ? // We check if the total time from the start of the week of the alarm is more or less than current
      alarm + __builtin_ctz(alarmDays) * 1440 - currentTotal + 10080 : // If less we add a week (example - 22h become 6 days and 2 h)
      alarm + __builtin_ctz(alarmDays) * 1440 - currentTotal; // Else we keep the positive value
    if (diff < minDiff){ 
      minDiff = diff; // We keep the lowest diff
      if (minDiff < 1440){ // No need to go further, we already found the lowest possible
        return minDiff;
      }
    }
    alarmDays &= alarmDays - 1; // Remove the LSB
  }
  return minDiff;
}