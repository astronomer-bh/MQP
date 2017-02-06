#include "Arduino.h"
#include "main.hpp"

#define LEDPIN 13
#define SENSOR1 Serial1
#define SENSOR2 Serial2
#define SENSOR3 Serial3
#define SENSOR4 Serial4

String val= ""; //holds the string of the value
double co2 =0; // holds the actual value
double multiplier = 10; //each range of sensor has a different value.
// up to 2% =1
// up to 65% = 10
//up to 100% = 100;

void setup(){
  pinMode(LEDPIN, OUTPUT);
  blinkLED();

  delay(4000);

  Serial.begin(9600);
  Serial.println("Started Serial");
  blinkLED();

  Serial1.begin(9600);
  Serial.println("Started SENSOR1");
  blinkLED();

  Serial1.print("K 1\r\n");
}

int ind;
char buffer[25];

void loop(){

    while(buffer[ind-1] != 0x0A){
      if(Serial1.available()){
      buffer[ind] = Serial1.read();
      ind++;
    }

  }
    report();
}

void blinkLED(){
  digitalWrite(LEDPIN, HIGH);
  delay(50);
  digitalWrite(LEDPIN, LOW);
  delay(100);
}

void report(){
  for(int i=0; i < ind; i++){
    if(buffer[i] == 'z') //once we hit the 'z' we can stop
    break;

    if((buffer[i] != 0x5A)&&(buffer[i] != 0x20)) //ignore 'Z' and white space
    {
      val += buffer[i]-48; //because we break at 'z' the only bytes getting added are the numbers
      // we subtract 48 to get to the actual numerical value
      // example the character '9' has an ASCII value of 57. [57-48=9]
    }
  }

  co2 = (multiplier * val.toInt()); //now we multiply the value by a factor specific ot the sensor. see the Cozir software guide
  Serial.print(co2);
  Serial.println(" ");
  ind=0; //Reset the buffer index to overwrite the previous packet
  val=""; //Reset the value string
}
