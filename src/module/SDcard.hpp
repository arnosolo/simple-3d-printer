#ifndef _SDcard_hpp_
#define _SDcard_hpp_
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "Utils/Array.hpp"

class SDcard {
private:
  uint8_t csPin;
public:
  Array<String> sdFileList;
  File selectedFile;
  bool sdFileSelected = false;

  SDcard(uint8_t csPin);
  void init();
  void getFileList();
  void printFileList();
  void readFile(int index);
};


// Array<String> SDcard::sdFileList = Array<String>();

SDcard::SDcard(uint8_t csPin) {
  this->csPin = csPin;
  this->init();
  this->sdFileList = Array<String>();
}

void SDcard::init() {
  if(SD.begin(this->csPin) == false){
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset or reopen this serial monitor after fixing your issue!");
    // while (true);
  } else {
    Serial.println("SD card initialization done.");
  }
}

void SDcard::getFileList() {
  this->sdFileList.clean();
  File root = SD.open("/");
  while (true) {
    File item =  root.openNextFile();
    if (!item) break;
    this->sdFileList.push(item.name());
    item.close();
  }
}

void printFileName(String str, int i) {
  Serial.print("Option ");
  Serial.print(i);
  Serial.print(": ");
  Serial.println(str);
};

void SDcard::printFileList() {
  this->sdFileList.forEach(printFileName);
}

void SDcard::readFile(int index) {
  this->selectedFile = SD.open(this->sdFileList[index]);
  this->sdFileSelected = true;
}

#endif 