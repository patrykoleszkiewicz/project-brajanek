#define SERVO 7

#define MOT_FR_DIR  28
#define MOT_FR_PWM  3
#define MOT_FR_ENCA 21
#define MOT_FR_ENCB 29

#define MOT_FL_DIR  26
#define MOT_FL_PWM  4
#define MOT_FL_ENCA 20
#define MOT_FL_ENCB 27

#define MOT_BR_DIR  24
#define MOT_BR_PWM  5
#define MOT_BR_ENCA 19
#define MOT_BR_ENCB 25

#define MOT_BL_DIR  22
#define MOT_BL_PWM  6
#define MOT_BL_ENCA 18
#define MOT_BL_ENCB 23

#define SAFETY_PIN 12


const int8_t STEER_TRIM = -4;
const uint8_t STEER_MAX = 130;
const uint8_t STEER_MIN = 50;

const float WHEELBASE_LENGHT = 305;
const float WHEELBASE_WIDTH = 257;

volatile long pulsesFR;
volatile long pulsesFL;
volatile long pulsesBR;
volatile long pulsesBL;

double readingFR, readingFL, readingBR, readingBL;

double outputFR, setPointFR;
double outputFL, setPointFL;
double outputBR, setPointBR;
double outputBL, setPointBL;

void blinkFR()
{
    if(digitalRead(MOT_FR_ENCA) ^ digitalRead(MOT_FR_ENCB))
    {
        ++pulsesFR;
    }
    else
    {
        --pulsesFR;
    }
}
void blinkFL()
{
    if(digitalRead(MOT_FL_ENCA) ^ digitalRead(MOT_FL_ENCB))
    {
        --pulsesFL;
    }
    else
    {
        ++pulsesFL;
    }
}
void blinkBR()
{
    if(digitalRead(MOT_BR_ENCA) ^ digitalRead(MOT_BR_ENCB))
    {
        ++pulsesBR;
    }
    else
    {
        --pulsesBR;
    }
}
void blinkBL()
{
    if(digitalRead(MOT_BL_ENCA) ^ digitalRead(MOT_BL_ENCB))
    {
        --pulsesBL;
    }
    else
    {
        ++pulsesBL;
    }
}

#include "Servo.h"
#include <PID_v1.h>

float kP = 1.0, kI = 4.0, kD = 0.0; //0.5 2.0 0.0

PID pidFR(&readingFR, &outputFR, &setPointFR, kP, kI, kD, DIRECT);
PID pidFL(&readingFL, &outputFL, &setPointFL, kP, kI, kD, DIRECT);
PID pidBR(&readingBR, &outputBR, &setPointBR, kP, kI, kD, DIRECT);
PID pidBL(&readingBL, &outputBL, &setPointBL, kP, kI, kD, DIRECT);

class Driver
{
  public:

    void init()
    {
        pinMode(SAFETY_PIN, INPUT_PULLUP);

        pinMode(MOT_FR_DIR, OUTPUT);
        pinMode(MOT_FR_PWM, OUTPUT);
        pinMode(MOT_FR_ENCA, INPUT);
        pinMode(MOT_FR_ENCB, INPUT);

        pinMode(MOT_FL_DIR, OUTPUT);
        pinMode(MOT_FL_PWM, OUTPUT);
        pinMode(MOT_FL_ENCA, INPUT);
        pinMode(MOT_FL_ENCB, INPUT);

        pinMode(MOT_BR_DIR, OUTPUT);
        pinMode(MOT_BR_PWM, OUTPUT);
        pinMode(MOT_BR_ENCA, INPUT);
        pinMode(MOT_BR_ENCB, INPUT);

        pinMode(MOT_BL_DIR, OUTPUT);
        pinMode(MOT_BL_PWM, OUTPUT);
        pinMode(MOT_BL_ENCA, INPUT);
        pinMode(MOT_BL_ENCB, INPUT);

        attachInterrupt(digitalPinToInterrupt(MOT_FR_ENCA), blinkFR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(MOT_FL_ENCA), blinkFL, CHANGE);
        attachInterrupt(digitalPinToInterrupt(MOT_BR_ENCA), blinkBR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(MOT_BL_ENCA), blinkBL, CHANGE);

        turner.attach(SERVO);
        turner.write(90);

        pidFR.SetMode(AUTOMATIC);
        pidFL.SetMode(AUTOMATIC);
        pidBR.SetMode(AUTOMATIC);
        pidBL.SetMode(AUTOMATIC);

        pidFR.SetOutputLimits(-255, 255);
        pidFL.SetOutputLimits(-255, 255);
        pidBR.SetOutputLimits(-255, 255);
        pidBL.SetOutputLimits(-255, 255);
    }

    void update(unsigned long current_millis, int throttle, int steer)
    {
        if(micros() - _encoderTimer > 42500)
        {
            readingFR = pulsesFR;
            readingFL = pulsesFL;
            readingBR = pulsesBR;
            readingBL = pulsesBL;
            pulsesFR = 0;
            pulsesFL = 0;
            pulsesBR = 0;
            pulsesBL = 0;
            _encoderTimer = micros();
        }
        setSteer(current_millis, steer);

        setThrottle(throttle);

        digitalDifferential();

        setMotorsSpeeds();
    }

  private:
    Servo turner;

    unsigned long _turner_timer;
    unsigned long _encoderTimer;

    int _throttle = 0;
    bool _reverse = 0;
    bool _brakes = false;

    uint8_t _steer_comp = 10;

    int16_t _turner_pos = 90;

    void setThrottle(int throttle)
    {
        _throttle = map(throttle, -1000, 1000, -255, 255);

        if(_brakes)
        {
            _throttle = 0;
        }
    }

    void digitalDifferential()
    {
        if(abs(_turner_pos - 90) < 2)
        {
            setPointFR = _throttle;
            setPointFL = _throttle;
            setPointBR = _throttle;
            setPointBL = _throttle;
            return;
        }

        float theta = (_turner_pos - 90.0) * 3.14159265 / 180.0;
        float turn_radius_back = abs(tan(theta) * WHEELBASE_LENGHT);
        float turn_radius_front = abs(WHEELBASE_LENGHT / cos(theta));
        float sign = theta < 0.0 ? 1.0 : -1.0;

        float TR_FR = turn_radius_front - sign * WHEELBASE_WIDTH / 2;
        float TR_FL = turn_radius_front + sign * WHEELBASE_WIDTH / 2;

        float TR_BR = turn_radius_back - sign * WHEELBASE_WIDTH / 2;
        float TR_BL = turn_radius_back + sign * WHEELBASE_WIDTH / 2;

        float TR_max = max(max(TR_FR, TR_FL), max(TR_BR, TR_BL));

        setPointFR = _throttle * (TR_FR / TR_max);
        setPointFL = _throttle * (TR_FL / TR_max);
        setPointBR = _throttle * (TR_BR / TR_max);
        setPointBL = _throttle * (TR_BL / TR_max);
    }

    void setMotorsSpeeds()
    {
        if(!digitalRead(SAFETY_PIN))
        {
            analogWrite(MOT_FR_PWM, 0);
            analogWrite(MOT_FL_PWM, 0);
            analogWrite(MOT_BR_PWM, 0);
            analogWrite(MOT_BL_PWM, 0);
            return;
        }

        setPointFR = constrain(setPointFR, readingFR - 100, readingFR + 100);
        setPointFL = constrain(setPointFL, readingFL - 100, readingFL + 100);
        setPointBR = constrain(setPointBR, readingBR - 100, readingBR + 100);
        setPointBL = constrain(setPointBL, readingBL - 100, readingBL + 100);

        pidFR.Compute();
        analogWrite(MOT_FR_PWM, abs(outputFR));
        digitalWrite(MOT_FR_DIR, outputFR < 0);

        pidFL.Compute();
        analogWrite(MOT_FL_PWM, abs(outputFL));
        digitalWrite(MOT_FL_DIR, outputFL < 0);

        pidBR.Compute();
        analogWrite(MOT_BR_PWM, abs(outputBR));
        digitalWrite(MOT_BR_DIR, outputBR < 0);

        pidBL.Compute();
        analogWrite(MOT_BL_PWM, abs(outputBL));
        digitalWrite(MOT_BL_DIR, outputBL < 0);
    }

    void setSteer(unsigned long current_millis, int steer)
    {
        if(current_millis - _turner_timer < _steer_comp)
        {
            return;
        }
        _turner_timer = current_millis;

        uint8_t target_pos = map(steer, -1000, 1000, STEER_MIN, STEER_MAX);

        if(_turner_pos < target_pos)
        {
            ++_turner_pos;
            if(_turner_pos < 90) _turner_pos += 2;
        }

        if(_turner_pos > target_pos)
        {
            --_turner_pos;
            if(_turner_pos > 90) _turner_pos -= 2;
        }

        turner.write(constrain(_turner_pos, STEER_MIN, STEER_MAX) + STEER_TRIM);
    }
};
