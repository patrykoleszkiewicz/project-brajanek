#include "Radio.hpp"
#include "Controls.hpp"

Radio radio;
Controls controls;

void setup()
{
    controls.init();
    radio.init();
    Serial.begin(115200);
}

void loop()
{
    controls.update();
    sentData.steer = controls.getAxis(0);
    sentData.throttle = -controls.getAxis(1);
    radio.update();
}
