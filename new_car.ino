#define CAR

#include "Driver.hpp"
#include "Radio.hpp"

Driver driver;
Radio radio;

unsigned long debugTimer;

void setup()
{
    Serial.begin(115200);
    radio.init();
    driver.init();
}

void loop()
{
    radio.update();
    driver.update(millis(), -receivedData.throttle, receivedData.steer);
    //Serial.println(receivedData.throttle);
}