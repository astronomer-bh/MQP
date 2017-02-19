#include "Arduino.h"
#include "main.hpp"

#define LEDPIN 13

void setup(){
  pinMode(LEDPIN, OUTPUT);

  delay(4000);
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
}

void loop(){
  if(Serial2.available()&&Serial3.available()
    &&Serial4.available()&&Serial5.available()) readSensors();
}

void readSensors(){
  String val = ""; //holds the string of the value
  while (Serial2.available()){
    char inChar = (char)(Serial2.read());
    if (inChar == 'z') {
      blinkLED();
      val += ",";
      do{}while(Serial2.read() != '\n'); // clear remaining buffer
    } else if (inChar != 'Z'&&inChar != ' '){
      val += inChar;
    }
  }

  while (Serial3.available()){
    char inChar = (char)(Serial3.read());
    if (inChar == 'z') {
      blinkLED();
      val += ",";
      do{}while(Serial3.read() != '\n'); // clear remaining buffer
    } else if (inChar != 'Z'&&inChar != ' '){
      val += inChar;
    }
  }

  while (Serial4.available()){
    char inChar = (char)(Serial4.read());
    if (inChar == 'z') {
      blinkLED();
      val += ",";
      do{}while(Serial4.read() != '\n'); // clear remaining buffer
    } else if (inChar != 'Z'&&inChar != ' '){
      val += inChar;
    }
  }

  while (Serial5.available()){
    char inChar = (char)(Serial5.read());
    if (inChar == 'z') {
      blinkLED();
      val += ",";
      do{}while(Serial5.read() != '\n'); // clear remaining buffer
    } else if (inChar != 'Z'&&inChar != ' '){
      val += inChar;
    }
  }

  Serial.println(val);
}

void blinkLED(){
  digitalWrite(LEDPIN, HIGH);
  delay(50);
  digitalWrite(LEDPIN, LOW);
  delay(100);
}
