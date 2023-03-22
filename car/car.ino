#include "Driver.hpp"
#include "Radio.hpp"
#include "Sensor.hpp"

Driver driver;
Radio radio;
SensorArray sensors;

void setup()
{
    Serial.begin(2000000);
    //radio.init();
    driver.init();
    sensors.init();
}

void loop()
{
    sensors.update(driver.getTurnerPos() - STEER_MIDDLE);
    //radio.update();

    driver.update(receivedData.brakes || radio.shouldStop(), receivedData.throttle, receivedData.steer);
}