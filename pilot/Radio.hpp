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
    bool honk;
};

const byte addressIn[] = { 0x14, 0x51, 0xDE, 0xDE, 0xAD };
const byte addressOut[] = { 0xDE, 0xAD, 0x14, 0x51, 0xDE };
#define CE        9
#define CSN       10
#define ERROR_LED 4
#define GOOD_LED  5

DataPilot sentData;
DataCar receivedData;

#define RADIO_TIMEOUT 100

class Radio
{
  public:
    bool init()
    {
        pinMode(ERROR_LED, OUTPUT);
        pinMode(GOOD_LED, OUTPUT);

        if(!_radio.begin(CE, CSN))
        {
            digitalWrite(ERROR_LED, HIGH);
            return false;
        }

        _radio.openWritingPipe(addressOut);
        _radio.openReadingPipe(1, addressIn);
        _radio.setPALevel(RF24_PA_MIN);
        _radio.setDataRate(RF24_250KBPS);

        analogWrite(GOOD_LED, 64);
        return true;
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
                _transmit = true;
                receiveTimeout = millis();
                digitalWrite(ERROR_LED, HIGH);
            }
        }
    }
  private:
    RF24 _radio;
    bool _transmit;
    unsigned long receiveTimeout;
} radio;
