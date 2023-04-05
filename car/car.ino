#define SENSOR_THRESHOLD 800
#define HONK_PIN         11

#include "Driver.hpp"
#include "Radio.hpp"
#include "Sensor.hpp"

Driver driver;
Radio radio;
SensorArray sensors;

void setup()
{
    pinMode(HONK_PIN, OUTPUT);
    Serial.begin(2000000);
    radio.init();
    driver.init();
    sensors.init();
}

void loop()
{
    sensors.update(driver.getTurnerPos() - STEER_MIDDLE);

    sentData.sensorLeft = sensors.getLeft();
    sentData.sensorRight = sensors.getRight();

    Serial.print(sentData.sensorLeft);
    Serial.print(" ");
    Serial.println(sentData.sensorRight);

    radio.update();

    if(sensors.getLeft() < SENSOR_THRESHOLD || sensors.getRight() < SENSOR_THRESHOLD)
    {
        receivedData.throttle = constrain(receivedData.throttle, -1000, 0);
    }

    driver.update(receivedData.brakes || radio.shouldStop(), receivedData.throttle, receivedData.steer);

    digitalWrite(HONK_PIN, receivedData.honk);
}