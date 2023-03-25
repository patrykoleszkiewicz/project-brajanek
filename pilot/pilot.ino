#include "Radio.hpp"
#include "Controls.hpp"
#include "Display.hpp"

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
    radio.update();
    display.update();
}
