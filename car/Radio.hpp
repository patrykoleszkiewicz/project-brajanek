#include <RF24.h>
#include <SPI.h>

struct DataCar
{
    uint16_t rpm1;
    uint16_t rpm2;
    uint16_t rpm3;
    uint16_t rpm4;
    uint16_t sensorLeft;
    uint16_t sensorRight;
};

struct DataPilot
{
    int16_t throttle;
    int16_t steer;
    bool reverse;
    bool brakes;
};

const byte addressIn[] = { 0xDE, 0xAD, 0x14, 0x51, 0xDE };
const byte addressOut[] = { 0x14, 0x51, 0xDE, 0xDE, 0xAD };

#define CE        9
#define CSN       10
#define ERROR_LED 13

#define STOP_TIMEOUT 300

DataCar sentData;
DataPilot receivedData;

#define RADIO_TIMEOUT 100

class Radio
{
  public:

    void init()
    {
        pinMode(ERROR_LED, OUTPUT);

        if(_radio.begin(CE, CSN))
        {
            _radio.openWritingPipe(addressOut);
            _radio.openReadingPipe(1, addressIn);
            _radio.setPALevel(RF24_PA_MIN);
            _radio.setDataRate(RF24_250KBPS);
        }
        else
        {
            digitalWrite(ERROR_LED, HIGH);
            //while(1) {}
        }
    }

    void update()
    {
        if(_transmit)
        {
            _radio.stopListening();
            _radio.write(&sentData, sizeof(sentData));
            _transmit = false;
        }
        else
        {
            _radio.startListening();
            if(_radio.available())
            {
                _radio.read(&receivedData, sizeof(receivedData));
                _transmit = true;
                receiveTimeout = millis();
                digitalWrite(ERROR_LED, LOW);
            }
            else if(millis() - receiveTimeout > RADIO_TIMEOUT)
            {
                digitalWrite(ERROR_LED, HIGH);
            }
        }
    }

    bool shouldStop()
    {
        return millis() - receiveTimeout > STOP_TIMEOUT;
    }

  private:
    RF24 _radio;
    bool _transmit;
    unsigned long receiveTimeout;
};
