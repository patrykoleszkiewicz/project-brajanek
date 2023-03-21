#include "Driver.hpp"
#include "Radio.hpp"
#include "Sensor.hpp"

Driver driver;
Radio radio;
SensorArray sensors;

float turnCoeffs[] = { 1.0, 1.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0, -1.0 };

void setup()
{
    Serial.begin(2000000);
    //radio.init();
    driver.init();
    sensors.init();
}

void loop()
{
    sensors.update();
    //radio.update();

    int turnBias = 0;

    for(int side = 0; side < 2; ++side)
    {
        for(int index = 0; index < sizeof(servoPositions); ++index)
        {
            //turnBias += turnCoeffs[side * sizeof(servoPositions) + index] * sensors.getReading(side, index);
            Serial.print(sensors.getReading(side, index));
            Serial.print(" ");
        }
    }
    Serial.println(" ");

    driver.update(receivedData.brakes || radio.shouldStop(), receivedData.throttle, receivedData.steer);
}