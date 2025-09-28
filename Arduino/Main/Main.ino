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
volatile uint8_t mode = 0; // 0: normal, 1: setup, 2: menu reveil

//variable de setup
bool setupIsInit = false;
uint8_t newTimeData[7]; // traitement en amont des valeurs à envoyer dans le DS3231
uint8_t isSetting;
uint8_t selected = 1; // 0: heure, 1: minutes, 2: secondes, 3: jour(date), 4: mois, 5: année 
unsigned long lastBlink = 0; // variable de test pour le clignotement du setup
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

//timing (a retirer lors du passage sur interruption)
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
MAX7219 myMAX7219_1(10);
MAX7219 myMAX7219_2(9);
bool ddot = true; //variable d'affichage des deux points
volatile uint8_t brightness = 0x03; // Variable de luminosité des afficheurs (entre 0x00 et 0x0F)
volatile bool brightnessFlag = false;

//code d'affichage des chiffres pour les MAX7219/7seg respectivement de 0 à 9
const uint8_t chiffres[10] = {
  0b01111110,
  0b00110000,
  0b01101101,
  0b01111001,
  0b00110011,
  0b01011011,
  0b01011111,
  0b01110000,
  0b01111111,
  0b01111011
};

//PIN des bouton
uint8_t btPlus = 2;         // bouton + sur D2 / PIN 4 / PCINT18
bool btPlusLastState = false; // Enregistre la derniere valeur du bouton pour une meilleur fluidite dans l utilisation dans le mode setup/reveil.
uint8_t btMoins = 3;        // bouton - sur D3 / PIN 5 / PCINT19
bool btMoinsLastState = false; // Enregistre la derniere valeur du bouton pour une meilleur fluidite dans l utilisation dans le mode setup/reveil.
uint8_t btSetup = 4;        // bouton setup sur D4 / PIN 6 / PCINT20
uint8_t btA = 5;            // bouton au dessus du reveil sur D5 / PIN 11 / PCINT21
uint8_t btGauche = 6;       // bouton gauche sur D6 / PIN 12 / PCINT22
bool btGaucheLastState = false; // Enregistre la derniere valeur du bouton pour une meilleur fluidite dans l utilisation dans le mode setup/reveil.
uint8_t btDroite = 7;       // bouton droite sur D7 / PIN 13 / PCINT23
bool btDroiteLastState = false; // Enregistre la derniere valeur du bouton pour une meilleur fluidite dans l utilisation dans le mode setup/reveil.

void setup() { //-------------------------------------------------setup
  double setupTimer = millis(); // calcul du temps de setup
  Serial.begin(9600); // port série debug à 9600 bauds
  for(uint8_t i = 0; i < 50; i++){ 
    Serial.println(); //serial clear
  }
  Serial.println("Hello depuis l'ATmega328P !");

  //Display init
  MAX7219::begin(11, 13);
  myMAX7219_1.init();
  myMAX7219_2.init();

  //Allumage du display pendant le setup
  myMAX7219_1.send(1, 0xff);
  myMAX7219_1.send(2, 0xff);
  myMAX7219_1.send(3, 0xff);
  myMAX7219_1.send(4, 0xff);
  myMAX7219_1.send(5, 0xff);

  myMAX7219_2.send(1, 0xff);
  myMAX7219_2.send(2, 0xff);
  myMAX7219_2.send(3, 0xff);
  myMAX7219_2.send(4, 0xff);
  myMAX7219_2.send(5, 0xff);

  //I2C init
  Wire.begin(); // setup de wire.h pour le bus I2C
  DS3231::begin(DS3231addr, true, &Serial);
  AT24C32::begin(AT24C32addr, true, &Serial);

  //Interruption clock (1hz donc deux interruption par seconde -> passage 0-1 et 1-0)
  pinMode(A3, INPUT);         // Entrée de la clock du DS3131 sur A3 (PC3)
  PCICR |= (1 << PCIE1);      // Active les interruption sur PCINT[14:8] --> PC0 à PC5 / A0 à A5
  PCMSK1 |= (1 << PCINT11);   // Active PCINT11 --> PC3 / A3
  
  //Interruption boutons
  PCICR |= (1 << PCIE2);      // Active les interruptions PCINT[16:23] --> PD0 à PD7 / D0 à D7

  pinMode(btPlus, INPUT);     // pinMode bouton
  PCMSK2 |= (1 << PCINT18);   // Active l'interruption
  pinMode(btMoins, INPUT);
  PCMSK2 |= (1 << PCINT19);
  pinMode(btSetup, INPUT);
  PCMSK2 |= (1 << PCINT20);
  pinMode(btA, INPUT);
  PCMSK2 |= (1 << PCINT21);
  pinMode(btGauche, INPUT);   // Pas d'interruption sur le bouton gauche
  pinMode(btDroite, INPUT);
  PCMSK2 |= (1 << PCINT23);
  
  sei();  // Active les interruption globale

  //Sleep_mode init
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

    /*
    //read all eeprom for debug
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
    Wire.write(0x00);
    /*
    Wire.write(0);
    Wire.write((1 << 4) | (4));
    Wire.write((2 << 4) | (3));
    Wire.write(7);
    Wire.write((2 << 4) | (7));
    Wire.write((0 << 4) | (9));
    Wire.write((2 << 4) | (5));
    Wire.endTransmission();
    */

    /*
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

  //next alarm setup
  nextAlarmIndex = findNextActiveAlarm(timeData, ActiveNU, (nbActive & 0xf0)>>4, ActiveU, nbActive & 0x0f);
  displayAlarm(nextAlarmIndex, ActiveNU, ActiveU, displayNextAlarm);
  
  //eteindre les 7seg
  while((millis() - setupTimer) < secondInterval){ //setup dure min 1 seconde
    delay(50);
  }
  myMAX7219_1.send(1, 0);
  myMAX7219_1.send(2, 0);
  myMAX7219_1.send(3, 0);
  myMAX7219_1.send(4, 0);
  myMAX7219_1.send(5, 0);

  myMAX7219_2.send(1, 0);
  myMAX7219_2.send(2, 0);
  myMAX7219_2.send(3, 0);
  myMAX7219_2.send(4, 0);
  myMAX7219_2.send(5, 0);

  //premier affichage
  myMAX7219_1.send(4, chiffres[(displayTime[1] & 0b00110000)>>4]);
  myMAX7219_1.send(3, chiffres[displayTime[1] & 0b00001111]);
  myMAX7219_1.send(2, chiffres[(displayTime[0] & 0b01110000)>>4]);
  myMAX7219_1.send(1, chiffres[displayTime[0] & 0b00001111]);

  myMAX7219_2.send(4, chiffres[(displayNextAlarm[1] & 0b00110000)>>4]);
  myMAX7219_2.send(3, chiffres[displayNextAlarm[1] & 0b00001111]);
  myMAX7219_2.send(2, chiffres[(displayNextAlarm[0] & 0b01110000)>>4]);
  myMAX7219_2.send(1, chiffres[displayNextAlarm[0] & 0b00001111]);
  myMAX7219_2.send(5, displayNextAlarm[2]);
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
      double loopTimer = millis(); // calcul du temps
      RTCFlag = false;
      if(!nightMode){
        //display time
        DS3231::readDisplayTime(displayTime);
        
        myMAX7219_1.send(4, chiffres[(displayTime[1] & 0b00110000)>>4]);
        myMAX7219_1.send(3, chiffres[displayTime[1] & 0b00001111]);
        myMAX7219_1.send(2, chiffres[(displayTime[0] & 0b01110000)>>4]);
        myMAX7219_1.send(1, chiffres[displayTime[0] & 0b00001111]);
        myMAX7219_1.send(5, ddot<<6);
        ddot = !ddot;

        if (lastMinute != computeValue(displayTime[0])){
          /*Serial.println("new minute");*/
          lastMinute = computeValue(displayTime[0]);
          DS3231::readFullDate(timeData);
          if(nbActive != 0){
            AlarmIsActive = checkAlarm(timeData, ActiveNU, (nbActive & 0xf0)>>4, ActiveU, nbActive & 0x0f);
            if(AlarmIsActive){
              /*Serial.println("!----------------------!");
              Serial.println("    Alarme detectee");
              Serial.println("!----------------------!");*/
              AlarmIsActive = false;  // arrêt avec le bouton theoriquement --> enfait je vais ajouter une variable isRinging
              // Maj de la variable display next alarm
              nextAlarmIndex = findNextActiveAlarm(timeData, ActiveNU, (nbActive & 0xf0)>>4, ActiveU, nbActive & 0x0f);
              displayAlarm(nextAlarmIndex, ActiveNU, ActiveU, displayNextAlarm);
              refreshNextAlarmDislpay = true;  // Refresh de l'affichage du next alarm
            }
          }
        }

        if(refreshNextAlarmDislpay){
              myMAX7219_2.send(4, chiffres[(displayNextAlarm[1] & 0b00110000)>>4]);
              myMAX7219_2.send(3, chiffres[displayNextAlarm[1] & 0b00001111]);
              myMAX7219_2.send(2, chiffres[(displayNextAlarm[0] & 0b01110000)>>4]);
              myMAX7219_2.send(1, chiffres[displayNextAlarm[0] & 0b00001111]);
              myMAX7219_2.send(5, displayNextAlarm[2]);
              refreshNextAlarmDislpay = false;
        }
      }
      else{
        myMAX7219_1.send(1, 0);
        myMAX7219_1.send(2, 0);
        myMAX7219_1.send(3, 0);
        myMAX7219_1.send(4, 0);
        myMAX7219_1.send(5, 0);

        myMAX7219_2.send(1, 0);
        myMAX7219_2.send(2, 0);
        myMAX7219_2.send(3, 0);
        myMAX7219_2.send(4, 0);
        myMAX7219_2.send(5, 0);
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
      // partie debug serial
    }
    switch(selected){
      case 0:{ // secondes
        myMAX7219_1.send(4, chiffres[(isSetting & 0b00110000)>>4]);
        myMAX7219_1.send(3, chiffres[isSetting & 0b00001111]);
        if(setupIsBlinking){
          myMAX7219_1.send(2, chiffres[(newTimeData[1] & 0b01110000)>>4]);
          myMAX7219_1.send(1, chiffres[newTimeData[1] & 0b00001111]);
          myMAX7219_1.send(5, 0);
          myMAX7219_2.send(4, chiffres[(newTimeData[0] & 0b01110000)>>4]);
          myMAX7219_2.send(3, chiffres[newTimeData[0] & 0b00001111]);
          myMAX7219_2.send(2, 0);
          myMAX7219_2.send(1, 0);
          myMAX7219_2.send(5, 0);
        }
        else{
          myMAX7219_1.send(2, 0);
          myMAX7219_1.send(1, 0);
          myMAX7219_1.send(5, 0);
          myMAX7219_2.send(4, 0);
          myMAX7219_2.send(3, 0);
          myMAX7219_2.send(2, 0);
          myMAX7219_2.send(1, 0);
          myMAX7219_2.send(5, 0);
        }
        break;
      }
      case 1:{ // minutes
        myMAX7219_1.send(2, chiffres[(isSetting & 0b01110000)>>4]);
        myMAX7219_1.send(1, chiffres[isSetting & 0b00001111]);
        if(setupIsBlinking){
          myMAX7219_1.send(4, chiffres[(newTimeData[2] & 0b00110000)>>4]);
          myMAX7219_1.send(3, chiffres[newTimeData[2] & 0b00001111]);
          myMAX7219_1.send(5, 0);
          myMAX7219_2.send(4, chiffres[(newTimeData[0] & 0b01110000)>>4]);
          myMAX7219_2.send(3, chiffres[newTimeData[0] & 0b00001111]);
          myMAX7219_2.send(2, 0);
          myMAX7219_2.send(1, 0);
          myMAX7219_2.send(5, 0);
        }
        else{
            myMAX7219_1.send(4, 0);
            myMAX7219_1.send(3, 0);
            myMAX7219_1.send(5, 0);
            myMAX7219_2.send(4, 0);
            myMAX7219_2.send(3, 0);
            myMAX7219_2.send(2, 0);
            myMAX7219_2.send(1, 0);
            myMAX7219_2.send(5, 0);
        }
        break;
      }
      case 2:{ // secondes
        myMAX7219_2.send(4, chiffres[(isSetting & 0b01110000)>>4]);
        myMAX7219_2.send(3, chiffres[isSetting & 0b00001111]);
        if(setupIsBlinking){
          myMAX7219_1.send(4, chiffres[(newTimeData[2] & 0b00110000)>>4]);
          myMAX7219_1.send(3, chiffres[newTimeData[2] & 0b00001111]);
          myMAX7219_1.send(2, chiffres[(newTimeData[1] & 0b01110000)>>4]);
          myMAX7219_1.send(1, chiffres[newTimeData[1] & 0b00001111]);
          myMAX7219_1.send(5, 0);
          myMAX7219_2.send(2, 0);
          myMAX7219_2.send(1, 0);
          myMAX7219_2.send(5, 0);
        }
        else{
          myMAX7219_1.send(4, 0);
          myMAX7219_1.send(3, 0);
          myMAX7219_1.send(2, 0);
          myMAX7219_1.send(1, 0);
          myMAX7219_1.send(5, 0);
          myMAX7219_2.send(2, 0);
          myMAX7219_2.send(1, 0);
          myMAX7219_2.send(5, 0);
        }
        break;
      }
      case 3:{ // jour
        myMAX7219_1.send(4, chiffres[(isSetting & 0b00110000)>>4]);
        myMAX7219_1.send(3, chiffres[isSetting & 0b00001111]);
        if(setupIsBlinking){
          myMAX7219_1.send(2, chiffres[(newTimeData[5] & 0b01110000)>>4]);
          myMAX7219_1.send(1, chiffres[newTimeData[5] & 0b00001111]);
          myMAX7219_1.send(5, 0);
          myMAX7219_2.send(4, chiffres[2]);
          myMAX7219_2.send(3, chiffres[0]);
          myMAX7219_2.send(2, chiffres[(newTimeData[6] & 0b11110000)>>4]);
          myMAX7219_2.send(1, chiffres[newTimeData[6] & 0b00001111]);
          myMAX7219_2.send(5, 0);
        }
        else{
          myMAX7219_1.send(2, 0);
          myMAX7219_1.send(1, 0);
          myMAX7219_1.send(5, 0);
          myMAX7219_2.send(4, 0);
          myMAX7219_2.send(3, 0);
          myMAX7219_2.send(2, 0);
          myMAX7219_2.send(1, 0);
          myMAX7219_2.send(5, 0);
        }
        break;
      }
      case 4:{ // mois
        myMAX7219_1.send(2, chiffres[(isSetting & 0b00010000)>>4]);
        myMAX7219_1.send(1, chiffres[isSetting & 0b00001111]);
        if(setupIsBlinking){
          myMAX7219_1.send(4, chiffres[(newTimeData[4] & 0b00110000)>>4]);
          myMAX7219_1.send(3, chiffres[newTimeData[4] & 0b00001111]);
          myMAX7219_1.send(5, 0);
          myMAX7219_2.send(4, chiffres[2]);
          myMAX7219_2.send(3, chiffres[0]);
          myMAX7219_2.send(2, chiffres[(newTimeData[6] & 0b11110000)>>4]);
          myMAX7219_2.send(1, chiffres[newTimeData[6] & 0b00001111]);
          myMAX7219_2.send(5, 0);
        }
        else{
          myMAX7219_1.send(4, 0);
          myMAX7219_1.send(3, 0);
          myMAX7219_1.send(5, 0);
          myMAX7219_2.send(4, 0);
          myMAX7219_2.send(3, 0);
          myMAX7219_2.send(2, 0);
          myMAX7219_2.send(1, 0);
          myMAX7219_2.send(5, 0);
        }
        break;
      }
      case 5:{ // année
        myMAX7219_2.send(4, chiffres[2]);
        myMAX7219_2.send(3, chiffres[0]);
        myMAX7219_2.send(2, chiffres[(isSetting & 0b11110000)>>4]);
        myMAX7219_2.send(1, chiffres[isSetting & 0b00001111]);
        if(setupIsBlinking){
          myMAX7219_1.send(4, chiffres[(newTimeData[4] & 0b00110000)>>4]);
          myMAX7219_1.send(3, chiffres[newTimeData[4] & 0b00001111]);
          myMAX7219_1.send(2, chiffres[(newTimeData[5] & 0b01110000)>>4]);
          myMAX7219_1.send(1, chiffres[newTimeData[5] & 0b00001111]);
          myMAX7219_1.send(5, 0);
          myMAX7219_2.send(5, 0);
        }
        else{
          myMAX7219_1.send(4, 0);
          myMAX7219_1.send(3, 0);
          myMAX7219_1.send(2, 0);
          myMAX7219_1.send(1, 0);
          myMAX7219_1.send(5, 0);
          myMAX7219_2.send(5, 0);
        }
        break;
      }
      case 6:{ // jour de la semaine
        myMAX7219_2.send(5, 1<<(isSetting-1));
        if(setupIsBlinking){
          myMAX7219_1.send(4, chiffres[(newTimeData[4] & 0b00110000)>>4]);
          myMAX7219_1.send(3, chiffres[newTimeData[4] & 0b00001111]);
          myMAX7219_1.send(2, chiffres[(newTimeData[5] & 0b01110000)>>4]);
          myMAX7219_1.send(1, chiffres[newTimeData[5] & 0b00001111]);
          myMAX7219_1.send(5, 0);
          myMAX7219_2.send(4, chiffres[2]);
          myMAX7219_2.send(3, chiffres[0]);
          myMAX7219_2.send(2, chiffres[(newTimeData[6] & 0b11110000)>>4]);
          myMAX7219_2.send(1, chiffres[newTimeData[6] & 0b00001111]);
        }
        else{
          myMAX7219_1.send(4, 0);
          myMAX7219_1.send(3, 0);
          myMAX7219_1.send(2, 0);
          myMAX7219_1.send(1, 0);
          myMAX7219_1.send(5, 0);
          myMAX7219_2.send(4, 0);
          myMAX7219_2.send(3, 0);
          myMAX7219_2.send(2, 0);
          myMAX7219_2.send(1, 0);
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
        }
        else if(btGaucheLastState && !setupLeft){
          btGaucheLastState = false;
        }
      }
      else if(setupRight || btDroiteLastState){
        if(!btDroiteLastState){
          btDroiteLastState = true;
          //activer, désactiver
          if(selectedType){
            AlarmNU[(selectedAlarm - 1) * 3] ^= 0b1000000;
            uint8_t buffer[3];
            for(uint8_t i = 0; i < 3; i++){
              buffer[i] = AlarmNU[(selectedAlarm - 1) * 3 + i];
            }
            AT24C32::modifyNU(selectedAlarm - 1, buffer);
          }
          else{
            AlarmU[(selectedAlarm - 1) * 2] ^= 0b1000000;
            uint8_t buffer[2];
            for(uint8_t i = 0; i < 2; i++){
              buffer[i] = AlarmNU[(selectedAlarm - 1) * 2 + i];
            }
            AT24C32::modifyU(selectedAlarm - 1, buffer);
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
          if((selectedAlarm + setupDown < maxSelectedAlarm + 1) && (selectedAlarm - setupUp >= 0)){ // on test pour ne pas sortir
            selectedAlarm = selectedAlarm - setupUp + setupDown;
          }
        }
        else if(UDHysteresis && !setupUp && !setupDown){
          UDHysteresis = false;
        }
      }
      else if(setupEnd){
        menuReveilInit = false; // reset de l'initialisation du mode menu reveil
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
      

      //display
      if(selectedAlarm == 0){ // ecran retour
        myMAX7219_1.send(4, 0b01001111);
        myMAX7219_1.send(3, 0b01011011);
        myMAX7219_1.send(2, 0b01001110);
        myMAX7219_1.send(1, 0b00000001);
        myMAX7219_1.send(5, 0);
        myMAX7219_2.send(4, 0);
        myMAX7219_2.send(3, 0);
        myMAX7219_2.send(2, 0);
        myMAX7219_2.send(1, 0);
        myMAX7219_2.send(5, 0);
      }
      else{
        if(!selectedType){ // alarme de type non unique
          if((selectedAlarm) <= ((nbAlarm & 0b11110000)>>4)){ // affichage classique, sinon propo nouvelle alarme
            alarmIsNew = false;
            myMAX7219_1.send(4, chiffres[(AlarmNU[(selectedAlarm - 1) * 3 + 1] & 0b00110000)>>4]);
            myMAX7219_1.send(3, chiffres[AlarmNU[(selectedAlarm - 1) * 3 + 1] & 0b00001111]);
            myMAX7219_1.send(2, chiffres[(AlarmNU[(selectedAlarm - 1) * 3] & 0b01110000)>>4]);
            myMAX7219_1.send(1, chiffres[AlarmNU[(selectedAlarm - 1) * 3] & 0b00001111]);
            myMAX7219_1.send(5, (AlarmNU[(selectedAlarm - 1) * 3] & 0b10000000)>>1);
            myMAX7219_2.send(5, AlarmNU[(selectedAlarm - 1) * 3 + 2]);
            if(!confirmDelete){
              myMAX7219_2.send(4, chiffres[selectedAlarm / 10]);
              myMAX7219_2.send(3, chiffres[selectedAlarm % 10]);
              myMAX7219_2.send(2, 0b00000001);
              myMAX7219_2.send(1, chiffres[0]);
            }
            else{
              myMAX7219_2.send(4, 0b01011011);
              myMAX7219_2.send(3, 0b00111110);
              myMAX7219_2.send(2, 0b01100111);
              myMAX7219_2.send(1, 0b00000001);
            }
          }
          else{
            alarmIsNew = true;
            myMAX7219_1.send(4, 0b00000001);
            myMAX7219_1.send(3, 0);
            myMAX7219_1.send(2, 0);
            myMAX7219_1.send(1, 0);
            myMAX7219_1.send(5, 0);
            myMAX7219_2.send(4, chiffres[selectedAlarm / 10]);
            myMAX7219_2.send(3, chiffres[selectedAlarm % 10]);
            myMAX7219_2.send(2, 0b00000001);
            myMAX7219_2.send(1, chiffres[0]);
            myMAX7219_2.send(5, 0);
          }
        }
        else{ // alarme de type unique
          if((selectedAlarm) <= (nbAlarm & 0b00001111)){ // affichage classique, sinon propo nouvelle alarme
            alarmIsNew = false;
            myMAX7219_1.send(4, chiffres[(AlarmU[(selectedAlarm - 1) * 2 + 1] & 0b00110000)>>4]);
            myMAX7219_1.send(3, chiffres[AlarmU[(selectedAlarm - 1) * 2 + 1] & 0b00001111]);
            myMAX7219_1.send(2, chiffres[(AlarmU[(selectedAlarm - 1) * 2] & 0b01110000)>>4]);
            myMAX7219_1.send(1, chiffres[AlarmU[(selectedAlarm - 1) * 2] & 0b00001111]);
            myMAX7219_1.send(5, 0);
            myMAX7219_2.send(5, 0);
            if(!confirmDelete){
              myMAX7219_2.send(4, chiffres[selectedAlarm / 10]);
              myMAX7219_2.send(3, chiffres[selectedAlarm % 10]);
              myMAX7219_2.send(2, 0b00000001);
              myMAX7219_2.send(1, chiffres[1]);
            }
            else{
              myMAX7219_2.send(4, 0b01011011);
              myMAX7219_2.send(3, 0b00111110);
              myMAX7219_2.send(2, 0b01100111);
              myMAX7219_2.send(1, 0b00000001);
            }
          }
          else{
            alarmIsNew = true;
            myMAX7219_1.send(4, 0b00000001);
            myMAX7219_1.send(3, 0);
            myMAX7219_1.send(2, 0);
            myMAX7219_1.send(1, 0);
            myMAX7219_1.send(5, 0);
            myMAX7219_2.send(4, chiffres[selectedAlarm / 10]);
            myMAX7219_2.send(3, chiffres[selectedAlarm % 10]);
            myMAX7219_2.send(2, 0b00000001);
            myMAX7219_2.send(1, chiffres[0]);
            myMAX7219_2.send(5, 0);
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
        else{
          newTimeData[0] = 0;
          newTimeData[1] = 0;
          newTimeData[2] = 0;
        }
        Serial.print("val newTimeData : ");
        Serial.print(newTimeData[0], BIN);
        Serial.print(" : ");
        Serial.print(newTimeData[1], BIN);
        Serial.print(" : ");
        Serial.print(newTimeData[2], BIN);
        selected = 0;
        isSetting = newTimeData[0];
        setupIsBlinking = false;
      }

      /*
      // partie bouton
      const bool setupUp = digitalread(btHaut); // plus grande prio
      const bool setupDown = digitalread(btBas);
      const bool setupRight = digitalread(btDroite);
      const bool setupLeft = digitalread(btGauche);
      const bool setupEnd = digitalread(boutonEnd); // plus petite prio
      if(setupUp || setupDown){
        isSetting = modifyAlarmIsSetting(selected, setupUp, isSetting);
      }
      else if(setupRight || setupLeft){
        if((selected + setupRight < 3) && (selected - setupLeft >= 0)){ // on test pour ne pas sortir de [0;6]
          newTimeData[selected] = isSetting;
          if(!selectedType){
            AT24C32::modifyNU(selectedAlarm, newTimeData);
          }
          else{
            AT24C32::modifyU(selectedAlarm, newTimeData);
          }
          selected = setupRight ? selected + 1 : selected - 1; // on déplace le curseur
        }
      }
      else if(setupEnd){newTimeData[selected] = isSetting;
        if(!selectedType){
          AT24C32::modifyNU(selectedAlarm, newTimeData);
        }
        else{
          AT24C32::modifyU(selectedAlarm, newTimeData);
        }
        settingAlarmInit = false; // reset de l'initialisation du mode setup
        return; // on force le redemarrage de loop()
      }
      */

      unsigned long now = millis();
      if(now - lastBlink > blinkTime){
        lastBlink = now;
        setupIsBlinking = !setupIsBlinking;
      }

      switch(selected){
        case 0:{ // heure
          myMAX7219_1.send(4, chiffres[(isSetting & 0b00110000)>>4]);
          myMAX7219_1.send(3, chiffres[isSetting & 0b00001111]);
          if(setupIsBlinking){
            myMAX7219_1.send(2, chiffres[(newTimeData[0] & 0b01110000)>>4]);
            myMAX7219_1.send(1, chiffres[newTimeData[0] & 0b00001111]);
            myMAX7219_1.send(5, 0);
            myMAX7219_2.send(4, 0);
            myMAX7219_2.send(3, 0);
            myMAX7219_2.send(2, 0);
            myMAX7219_2.send(1, 0);
            myMAX7219_2.send(5, newTimeData[2]);
          }
          else{
            myMAX7219_1.send(2, 0);
            myMAX7219_1.send(1, 0);
            myMAX7219_1.send(5, 0);
            myMAX7219_2.send(4, 0);
            myMAX7219_2.send(3, 0);
            myMAX7219_2.send(2, 0);
            myMAX7219_2.send(1, 0);
            myMAX7219_2.send(5, 0);
          }
          break;
        }
        case 1:{ // minutes
          myMAX7219_1.send(2, chiffres[(isSetting & 0b01110000)>>4]);
          myMAX7219_1.send(1, chiffres[isSetting & 0b00001111]);
          if(setupIsBlinking){
            myMAX7219_1.send(4, chiffres[(newTimeData[1] & 0b00110000)>>4]);
            myMAX7219_1.send(3, chiffres[newTimeData[1] & 0b00001111]);
            myMAX7219_1.send(5, 0);
            myMAX7219_2.send(4, 0);
            myMAX7219_2.send(3, 0);
            myMAX7219_2.send(2, 0);
            myMAX7219_2.send(1, 0);
            myMAX7219_2.send(5, newTimeData[2]);
          }
          else{
            myMAX7219_1.send(4, 0);
            myMAX7219_1.send(3, 0);
            myMAX7219_1.send(5, 0);
            myMAX7219_2.send(4, 0);
            myMAX7219_2.send(3, 0);
            myMAX7219_2.send(2, 0);
            myMAX7219_2.send(1, 0);
            myMAX7219_2.send(5, 0);
          }
          break;
        }
        default:{ // jour de la semaine, pour les alarmes unique on a pas besoin de ça (test réalisé en amont)
            myMAX7219_2.send(4, chiffres[selected - 1]);
            myMAX7219_2.send(5, newTimeData[2]);
          if(setupIsBlinking){
            myMAX7219_1.send(4, chiffres[(newTimeData[1] & 0b00110000)>>4]);
            myMAX7219_1.send(3, chiffres[newTimeData[1] & 0b00001111]);
            myMAX7219_1.send(2, chiffres[(newTimeData[0] & 0b01110000)>>4]);
            myMAX7219_1.send(1, chiffres[newTimeData[0] & 0b00001111]);
            myMAX7219_1.send(5, 0);
            myMAX7219_2.send(3, 0);
            myMAX7219_2.send(2, 0);
            myMAX7219_2.send(1, 0);
          }
          else{
            myMAX7219_1.send(4, 0);
            myMAX7219_1.send(3, 0);
            myMAX7219_1.send(2, 0);
            myMAX7219_1.send(1, 0);
            myMAX7219_1.send(5, 0);
            myMAX7219_2.send(3, 0);
            myMAX7219_2.send(2, 0);
            myMAX7219_2.send(1, 0);
          }
          break;
        }
      }
    }
    delay(50);
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


uint8_t AlarmSetupSelectNew(const uint8_t newSelected, const uint8_t* nTD){
  const uint8_t table[8] = {
    1, // heure
    0, // minute
    2, 2, 2, 2, 2, 2 // jour de la semaine
  };
  return nTD[table[newSelected]];
}

uint8_t modifyAlarmIsSetting(const uint8_t selectedParam, const bool setupUp, uint8_t data){
  const uint8_t bounds[7][2] = {
    {23, 0}, // heure entre 0 et 23
    {59, 0} // minute entre 0 et 59
  };
  if(selectedParam < 2){
    data = computeValue(data); // + ou - 1 en fonction de setupUp
    if(setupUp){
      data = data == bounds[selected][0] ? bounds[selected][1] : data + 1; // test valeurs max
    }
    else{
      data = data == bounds[selected][1] ? bounds[selected][0] : data - 1; // test valeurs min
    }
    return computeValueInv(data);
  }
  else {
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
    data = data == bounds[selected][0] ? bounds[selected][1] : data + 1; // test valeurs max
  }
  else{
    data = data == bounds[selected][1] ? bounds[selected][0] : data - 1; // test valeurs min
  }
  return computeValueInv(data);
}

uint8_t computeValue(const uint8_t value){
  return (value & 0b00001111) + (((value & 0b11110000)>>4)*10);
}

uint8_t computeValueInv(const uint8_t value){
  return (uint8_t(value / 10)<<4) | (uint8_t(value % 10));
}

bool checkAlarm(uint8_t* time, uint8_t* bufferNU, uint8_t nbNU, uint8_t* bufferU, uint8_t nbU){
  uint8_t i = 0;
  //si il y a plusieur alarme unique a une même heure, seulement la premiere sera desarme.
  while(i < nbU){
    if(
      ((bufferU[i*2] & 0b01111111) == time[0]) &&
      (bufferU[i*2+1] == (time[1] & 0b00111111))
    ){
      const uint8_t tmp[2] = {bufferU[i] & 0b011111111, bufferU[i+1]};
      //AT24C32::modifyU(i, tmp);
      return true;
    }
    i++;
  }

  i = 0;
  while(i < nbNU){
    if(
      ((bufferNU[i*3] & 0b01111111) == time[0]) &&
      (bufferNU[i*3+1] == (time[1] & 0b00111111)) &&
      (bufferNU[i*3+2] & (0b1<< (time[2]-1)))
    ){
      return true;
    }
    i++;
  }
  return false;
}

void displayAlarm(uint8_t index, const uint8_t * bufferNU, const uint8_t* bufferU, uint8_t* display){
  if(index & 0b10000000){
    index &= 0b00001111;
    display[0] = bufferNU[index*3];
    display[1] = bufferNU[index*3+1];
    display[2] = bufferNU[index*3+2];
  }
  else{
    display[0] = bufferU[index*2];
    display[1] = bufferU[index*2+1];
    display[2] = 0x00;
  }
}

uint8_t findNextActiveAlarm(uint8_t* time, uint8_t* bufferNU, uint8_t nbNU, uint8_t* bufferU, uint8_t nbU){
  //boucle Unique
  int16_t current = toMin(time[0], time[1]) + 1; //pour ne pas afficher celui de la minute en cour.
  uint8_t index = 0;
  int16_t minDiff = 10080;
  if(nbU){
    minDiff = computeDiff(current, toMin(bufferU[0], bufferU[1]));
    
    for(uint8_t i = 2; i < nbU; i = i + 2){
      int16_t diff = computeDiff(current, toMin(bufferU[i], bufferU[i+1]));
      if(diff < minDiff){
        minDiff = diff;
        index = i / 2;
      }
    }
  }
  if(nbNU){
    for(uint8_t i = 0; i < nbNU; i = i + 3){
      int16_t diff = computeDiffNU(current, toMin(bufferNU[i], bufferNU[i+1]), time[2], bufferNU[i+2]);
      if(diff < minDiff){
        minDiff = diff;
        index = (i / 3) | 0b10000000;
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
  if(diff < 0){
    diff += 1440;
  }
  return diff;
}

int16_t computeDiffNU(int16_t current, int16_t alarm, uint8_t currentDay, uint8_t alarmDays){
  bool nextday = false;
  int16_t diff = alarm - current;
  if(diff < 0){
    diff += 1440;
    nextday = true;
  }

  uint8_t b = alarmDays;
  uint8_t lsb = b & -b;
  uint8_t bit = __builtin_ctz(lsb);
  int8_t shiftedBit = bit + 1 - currentDay - nextday;
  if (shiftedBit < 0){
    shiftedBit = shiftedBit + 7;
  }
  if((bit + 1 - currentDay - nextday)<0){
    b &= b - 1;
    if(b){
      bool isfirstday = false;
      while (b && !isfirstday) {
        lsb = b & -b;
        bit = __builtin_ctz(lsb);
        int8_t newshiftedBit = bit + 1 - currentDay - nextday;
        if (newshiftedBit < 0){
          newshiftedBit = newshiftedBit + 7;
        }
        if ((bit + 1 - currentDay - nextday)>=0){
          shiftedBit = newshiftedBit;
          isfirstday = true;
        }
        b &= b - 1;
      }
    }
  }
  return shiftedBit * 1440 + diff;
}