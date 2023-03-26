#define HANDBRAKE_PIN 6

#define SAMPLE_COUNT         4
#define AXIS_ADC_INTERVAL_US 5000
#define SWITCH_THRESHOLD     800
#define BUTTON_DELAY_US      10000

byte axisPins[] = { A1, A0, A2 };

const int axisCount = sizeof(axisPins);

int axisCenter[] = { 500, 512, 488 };

bool axisAbsolute[] = { 0, 0, 1 };

class Controls
{
  public:
    void init()
    {
        pinMode(HANDBRAKE_PIN, INPUT_PULLUP);

        for(int axis = 0; axis < axisCount; ++axis)
        {
            for(int sample = 0; sample < SAMPLE_COUNT; ++sample)
            {
                if(!axisAbsolute[axis])
                {
                    axisReadout[axis][sample] = axisCenter[axis];
                }
            }
        }

        handbrake = true;
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
                    axisSelector = 2;
                    break;

                case 2:
                    updateAxis(2);
                    axisSelector = 0;
                    break;
            }
        }

        if(!gearSwitch && axisData[1] < -SWITCH_THRESHOLD)
        {
            gearSwitch = true;
            ++gear;

            if(gear > 3)  //P - D - P - R
            {
                gear = 0;
            }
        }

        if(axisData[1] > -SWITCH_THRESHOLD)
        {
            gearSwitch = false;
        }

        sentData.steer = axisData[0];
        sentData.throttle = axisData[2];

        if(now - handbrakeTimer >= BUTTON_DELAY_US)
        {
            if(!handbrakeSwitch && digitalRead(HANDBRAKE_PIN) == LOW)
            {
                handbrakeTimer = now;
                handbrakeSwitch = true;
                handbrake = !handbrake;
            }

            if(digitalRead(HANDBRAKE_PIN) == HIGH)
            {
                handbrakeSwitch = false;
            }
        }

        switch(gear)
        {
            case 0:
            case 2:
                sentData.brakes = true;
                break;
            case 1:
                sentData.brakes = false;
                sentData.reverse = false;
                break;
            case 3:
                sentData.brakes = false;
                sentData.reverse = true;
                break;
        }

        sentData.brakes = sentData.brakes || handbrake;
    }

    int getAxis(int index)
    {
        return axisData[index];
    }

    int getGear()
    {
        return gear;
    }

    bool getHandbrake()
    {
        return handbrake || axisData[1] > SWITCH_THRESHOLD;
    }
  private:
    unsigned long axisTimeout;

    byte readoutSelector[axisCount];
    byte axisSelector;

    int axisReadout[axisCount][SAMPLE_COUNT];

    int axisData[axisCount];

    bool gearSwitch;
    int gear;

    bool handbrake;
    bool handbrakeSwitch;
    unsigned long handbrakeTimer;

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

        if(axisAbsolute[index])
        {
            axisData[index] = map(readout, 0, 1023, 0, 1000);
        }
        else
        {
            if(readout < axisCenter[index])
            {
                axisData[index] = map(readout, 0, axisCenter[index], -1000, 0);
            }
            else
            {
                axisData[index] = map(readout, axisCenter[index], 1023, 0, 1000);
            }
        }
    }
} controls;
