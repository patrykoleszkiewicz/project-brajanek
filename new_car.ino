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
    if(millis() - debugTimer > 200)
    {
        //Serial.println(receivedData.steer);
        debugTimer = millis();
    }
}