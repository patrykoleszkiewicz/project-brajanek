#define PILOT

#include "Radio.hpp"

#define SAMPLE_COUNT 16

int axisCenter[2] = {500, 512};

byte axisSelector;

byte axisPins[2] = {A1, A0};

int axisReadout[2][SAMPLE_COUNT];
byte readoutSelector[2];

int axisData[2];

Radio radio;

void setup()
{
    radio.init();
    Serial.begin(115200);
}

void loop()
{
    switch(axisSelector)
    {
        case 0:
            updateAxis(0);
            axisSelector = 1;
            break;

        case 1:
            updateAxis(1);
            axisSelector = 0;
            break;
    }

    //Serial.println(axisData[0]);
    sentData.steer = axisData[0];
    sentData.throttle = -axisData[1];
    radio.update();
}

void updateAxis(byte index)
{
    axisReadout[index][readoutSelector[index]]  = analogRead(axisPins[index]);
    readoutSelector[index] = (readoutSelector[index] + 1) % SAMPLE_COUNT;
    
    int readout = 0;
    for(byte i = 0; i < SAMPLE_COUNT; ++i)
    {
        readout += axisReadout[index][i];
    }
    readout = readout / SAMPLE_COUNT;

    if(readout < axisCenter[index])
    {
        axisData[index] = map(readout, 0, axisCenter[index], -1000, 0);
    }
    else
    {
        axisData[index] = map(readout, axisCenter[index], 1023, 0, 1000);
    }
}
