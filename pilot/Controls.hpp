#define SAMPLE_COUNT         16
#define AXIS_ADC_INTERVAL_US 5000

int axisCenter[2] = { 500, 512 };

byte axisPins[2] = { A1, A0 };

class Controls
{
  public:
    void init()
    {
    }

    void update()
    {
        unsigned long now = micros();
        if(now - axisTimeout > AXIS_ADC_INTERVAL_US)
        {
            axisTimeout = now;
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
        }
    }

    int getAxis(int index)
    {
        return axisData[index];
    }
  private:
    unsigned long axisTimeout;

    byte readoutSelector[2];
    byte axisSelector;

    int axisReadout[2][SAMPLE_COUNT];

    int axisData[2];

    void updateAxis(byte index)
    {
        axisReadout[index][readoutSelector[index]] = analogRead(axisPins[index]);
        readoutSelector[index]++;
        if(readoutSelector[index] >= SAMPLE_COUNT)
        {
            readoutSelector[index] = 0;            
        }

        long readout = 0;
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
};
