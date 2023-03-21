#include "Radio.hpp"
#include "Controls.hpp"
#include "Display.hpp"

Radio radio;
Controls controls;
Display display;

void setup()
{
    controls.init();
    radio.init();
    display.init();
    Serial.begin(115200);
}

void loop()
{
    controls.update();
    sentData.steer = controls.getAxis(0);
    sentData.throttle = -controls.getAxis(1);
    radio.update();
    display.update();
}
