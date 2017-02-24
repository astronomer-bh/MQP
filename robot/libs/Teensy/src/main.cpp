#include "Arduino.h"
#include "main.hpp"

#define LEDPIN 13

void setup(){
  delay(4000);
  pinMode(LEDPIN, OUTPUT);
  Serial.begin(9600);
  blinkLED();
  Serial2.begin(9600);
  blinkLED();
  Serial3.begin(9600);
  blinkLED();
  Serial4.begin(9600);
  blinkLED();
  Serial5.begin(9600);
  blinkLED();
  delay(1000);

  //set to polling mode
  Serial2.print("K 2\r\n");
  Serial3.print("K 2\r\n");
  Serial4.print("K 2\r\n");
  Serial5.print("K 2\r\n");

  //clear all buffers
  while(Serial2.read() != '\n'){};
  while(Serial3.read() != '\n'){};
  while(Serial4.read() != '\n'){};
  while(Serial5.read() != '\n'){};
}

void loop(){
  // check if newline is sent from create
  // if sent then ask sensors for data
  if (Serial.read() == '\n'){
    request();
    readSensors();
  }
}

void request(){
  Serial2.print("Z\r\n");
  Serial3.print("Z\r\n");
  Serial4.print("Z\r\n");
  Serial5.print("Z\r\n");
}

void readSensors(){
  String val = ""; // holds the string of the value
  char inChar = 'p';

  do{
    while(!Serial2.available()){}
    inChar = (char)Serial2.read();
    if(inChar != '\r' && inChar != 'Z' && inChar != ' ' && inChar != '\n'){
      val += inChar;
    }
  } while (inChar != '\n');
  val += ",";
  inChar = 'p';

  do{
    while(!Serial3.available()){}
    inChar = (char)Serial3.read();
    if(inChar != '\r' && inChar != 'Z' && inChar != ' ' && inChar != '\n'){
      val += inChar;
    }
  } while (inChar != '\n');
  val += ",";
  inChar = 'p';

  do{
    while(!Serial4.available()){}
    inChar = (char)Serial4.read();
    if(inChar != '\r' && inChar != 'Z' && inChar != ' ' && inChar != '\n'){
      val += inChar;
    }
  } while (inChar != '\n');
  val += ",";
  inChar = 'p';

  do{
    while(!Serial5.available()){}
    inChar = (char)Serial5.read();
    if(inChar != '\r' && inChar != 'Z' && inChar != ' ' && inChar != '\n'){
      val += inChar;
    }
  } while (inChar != '\n');

  Serial.print(val);
  Serial.print('\n');
  blinkLED();
}

void blinkLED(){
  digitalWrite(LEDPIN, HIGH);
  delay(100);
  digitalWrite(LEDPIN, LOW);
  delay(100);
}
