#include "Driver.hpp"

#include <RF24.h>
#include <SPI.h>

#define CE 9
#define CSN 10
#define ERROR_LED 13

#define STOP_TIMEOUT  500
#define RADIO_TIMEOUT 100

RF24 radio(CE, CSN);
long radio_timer = 0;
byte cstatus;
// 0 - OK
// 1 - NO CONNECTION
const byte address[] = { 0xDE, 0xAD, 0x14, 0x51, 0xDE };

struct DataSend
{
    uint16_t rpm1 = 0;
    uint16_t rpm2 = 0;
    uint16_t rpm3 = 0;
    uint16_t rpm4 = 0;
    uint8_t s1 = 0;
    uint8_t s2 = 0;
    uint8_t s3 = 0;
    uint8_t s4 = 0;
    uint8_t s5 = 0;
    uint8_t s6 = 0;
    uint8_t s7 = 0;
    uint16_t bat = 0;
};

struct DataRecv
{
    int16_t throttle = 0;
    int16_t steer = 0;
    bool reverse = 0;
    bool brakes;

    uint8_t control_mode;
    uint8_t drive_mode;
    uint8_t steer_comp;
    uint8_t motor_comp;
};

DataSend local_state;
DataRecv remote_state;

long timeoutStop;

Driver driver;

unsigned long currMicros;
unsigned long currentMillis;

void setup()
{
    pinMode(ERROR_LED, OUTPUT);
    Serial.begin(115200);

    if(radio.begin())
    {
        radio.openReadingPipe(0, address);
        radio.setPALevel(RF24_PA_MAX);
        radio.setDataRate(RF24_250KBPS);
        radio.enableDynamicPayloads();
        radio.enableAckPayload();
        radio.startListening();
    }
    else
    {
        digitalWrite(ERROR_LED, HIGH);
        while(1) {}
    }

    driver.init();
}

void loop()
{
    currMicros = micros();
    currentMillis = millis();

    if(radio.available())
    {
        radio.read(&remote_state, sizeof(remote_state));
        radio.writeAckPayload(0, &local_state, sizeof(local_state));

        radio_timer = currentMillis;
        timeoutStop = currentMillis;
        cstatus = 0;
    }
    else if(currentMillis - radio_timer > RADIO_TIMEOUT)
    {
        radio_timer = currentMillis;
        cstatus = 1;

        radio.begin();
        radio.openReadingPipe(0, address);
        radio.setPALevel(RF24_PA_MAX);
        radio.setDataRate(RF24_250KBPS);
        radio.enableDynamicPayloads();
        radio.enableAckPayload();
        radio.startListening();
    }

    if(currentMillis - timeoutStop > STOP_TIMEOUT)
    {
        //remote_state.throttle = 0;
    }

    digitalWrite(ERROR_LED, cstatus);

    driver.update(currentMillis, (int)remote_state.throttle * 3, remote_state.steer);
    //Serial.println(remote_state.throttle);
    
    digitalWrite(ERROR_LED, cstatus);
    //driver.update(currentMillis, 500, 500);

    //Serial.print(" ");
    //Serial.println(micros() - currMicros);
}