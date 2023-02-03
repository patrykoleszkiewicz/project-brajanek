#define CAR

#include "Driver.hpp"
#include "Radio.hpp"

Driver driver;
Radio radio;

unsigned long debugTimer;

void setup()
{
    radio.init();
    driver.init();
}

void loop()
{
    radio.update();
    driver.update(millis(), receivedData.throttle, receivedData.steer);
}