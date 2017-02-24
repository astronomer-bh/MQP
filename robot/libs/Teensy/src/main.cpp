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

}

void loop(){
  readSensors();
}

void readSensors(){
  String val = ""; //holds the string of the value
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

  Serial.println(val);
  blinkLED();
}

void blinkLED(){
  digitalWrite(LEDPIN, HIGH);
  delay(100);
  digitalWrite(LEDPIN, LOW);
  delay(100);
}
