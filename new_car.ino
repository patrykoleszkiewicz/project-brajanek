#include "Driver.hpp"

Driver driver;

int throttle;
int steer;

unsigned long timeoutS;
unsigned long timeoutT;
unsigned long timeoutSend;

unsigned long currMicros;
unsigned long currMillis;

void setup() {
  driver.init();
  Serial.begin(9600);
  Serial1.begin(9600);

  //driver.testEngines();
}

void loop() {
  currMicros = micros();
  currMillis = millis();
  
  if (Serial1.available() > 0) {
    if (Serial1.read() == '_') {
      char tmp = waitChar();
      switch (tmp) {
        case 'S':
          Serial1.read();
          steer = Serial1.parseInt();
          timeoutS = currMillis + 150;
          break;
        case 'T':
          Serial1.read();
          Serial1.parseInt();
          Serial1.read();
          throttle = 800 - Serial1.parseInt();
          timeoutT = currMillis + 150;
          break;
      }
      //Serial.print('\n');
    }
    //Serial.println(throttle);
  }

  if (timeoutS < currMillis) {
    steer = 0;
  }
  if (timeoutT < currMillis) {
    throttle = 0;
  }

  //driver.update(millis(), throttle, steer);
  driver.update(currMillis, 500, 500);

  if(timeoutSend < currMillis)
  {
    timeoutSend = currMillis + 100;
    Serial1.print("*Y");
    Serial1.print(throttle + 1000);
    Serial1.print("*");
    Serial1.print("*D");
    Serial1.print(steer + 1000);
    Serial1.print("*");
    //Serial.println(steer);
  }
  
  //Serial.print(" ");
  Serial.println(micros() - currMicros);
}

char waitChar() {
  while (!Serial1.available()) loop();
  return Serial1.read();
}