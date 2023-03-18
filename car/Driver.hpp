#define SERVO 7

#define MOT_FR_DIR  28
#define MOT_FR_PWM  3
#define MOT_FR_ENCA 18
#define MOT_FR_ENCB 23

#define MOT_FL_DIR  26
#define MOT_FL_PWM  4
#define MOT_FL_ENCA 19
#define MOT_FL_ENCB 25

#define MOT_BR_DIR  24
#define MOT_BR_PWM  5
#define MOT_BR_ENCA 20
#define MOT_BR_ENCB 27

#define MOT_BL_DIR  22
#define MOT_BL_PWM  6
#define MOT_BL_ENCA 21
#define MOT_BL_ENCB 29

#define SAFETY_PIN 12

#define STEER_UPDATE_INTERVAL_US   10000
#define ENCODER_UPDATE_INTERVAL_US 42500

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

float kP = 1.0, kI = 4.0, kD = 0.0;  //0.5 2.0 0.0

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

        pidFR.SetSampleTime(1);
        pidFL.SetSampleTime(1);
        pidBR.SetSampleTime(1);
        pidBL.SetSampleTime(1);
    }

    void update(unsigned long current_millis, int throttle, int steer)
    {
        unsigned long now = micros();
        if(now - _turnerTimer >= STEER_UPDATE_INTERVAL_US)
        {
            _turnerTimer = now;
            setSteer(steer);
        }

        if(now - _encoderTimer >= ENCODER_UPDATE_INTERVAL_US)
        {
            _encoderTimer = now;

            readingFR = pulsesFR;
            readingFL = pulsesFL;
            readingBR = pulsesBR;
            readingBL = pulsesBL;
            pulsesFR = 0;
            pulsesFL = 0;
            pulsesBR = 0;
            pulsesBL = 0;

            setThrottle(throttle);

            digitalDifferential();

            setMotorsSpeeds();
        }
    }

  private:
    Servo turner;

    unsigned long _turnerTimer;
    unsigned long _encoderTimer;

    int _throttle = 0;
    bool _reverse = 0;
    bool _brakes = false;

    int16_t _turner_pos = 90;

    void setThrottle(int throttle)
    {
        _throttle = throttle;

        if(_brakes)
        {
            _throttle = 0;
        }
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

    void setSteer(int steer)
    {
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

    void digitalDifferential()
    {
        switch(180 - _turner_pos)
        {
            case 45:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1675, 1675, -MAX_PULSES, MAX_PULSES);  //0.596679
                setPointBR = map(_throttle, -1263, 1263, -MAX_PULSES, MAX_PULSES);  //0.791759
                setPointBL = map(_throttle, -3067, 3067, -MAX_PULSES, MAX_PULSES);  //0.326018
                break;

            case 46:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1669, 1669, -MAX_PULSES, MAX_PULSES);  //0.59899
                setPointBR = map(_throttle, -1251, 1251, -MAX_PULSES, MAX_PULSES);  //0.799033
                setPointBL = map(_throttle, -2936, 2936, -MAX_PULSES, MAX_PULSES);  //0.340513
                break;

            case 47:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1662, 1662, -MAX_PULSES, MAX_PULSES);  //0.601587
                setPointBR = map(_throttle, -1240, 1240, -MAX_PULSES, MAX_PULSES);  //0.806214
                setPointBL = map(_throttle, -2816, 2816, -MAX_PULSES, MAX_PULSES);  //0.355019
                break;

            case 48:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1654, 1654, -MAX_PULSES, MAX_PULSES);  //0.604472
                setPointBR = map(_throttle, -1229, 1229, -MAX_PULSES, MAX_PULSES);  //0.8133
                setPointBL = map(_throttle, -2706, 2706, -MAX_PULSES, MAX_PULSES);  //0.369537
                break;

            case 49:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1645, 1645, -MAX_PULSES, MAX_PULSES);  //0.607649
                setPointBR = map(_throttle, -1219, 1219, -MAX_PULSES, MAX_PULSES);  //0.820293
                setPointBL = map(_throttle, -2603, 2603, -MAX_PULSES, MAX_PULSES);  //0.384069
                break;

            case 50:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1636, 1636, -MAX_PULSES, MAX_PULSES);  //0.611119
                setPointBR = map(_throttle, -1208, 1208, -MAX_PULSES, MAX_PULSES);  //0.827191
                setPointBL = map(_throttle, -2508, 2508, -MAX_PULSES, MAX_PULSES);  //0.398618
                break;

            case 51:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1626, 1626, -MAX_PULSES, MAX_PULSES);  //0.614885
                setPointBR = map(_throttle, -1199, 1199, -MAX_PULSES, MAX_PULSES);  //0.833994
                setPointBL = map(_throttle, -2420, 2420, -MAX_PULSES, MAX_PULSES);  //0.413184
                break;

            case 52:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1615, 1615, -MAX_PULSES, MAX_PULSES);  //0.618949
                setPointBR = map(_throttle, -1189, 1189, -MAX_PULSES, MAX_PULSES);  //0.840701
                setPointBL = map(_throttle, -2337, 2337, -MAX_PULSES, MAX_PULSES);  //0.427769
                break;

            case 53:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1604, 1604, -MAX_PULSES, MAX_PULSES);  //0.623314
                setPointBR = map(_throttle, -1180, 1180, -MAX_PULSES, MAX_PULSES);  //0.847313
                setPointBL = map(_throttle, -2260, 2260, -MAX_PULSES, MAX_PULSES);  //0.442376
                break;

            case 54:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1592, 1592, -MAX_PULSES, MAX_PULSES);  //0.62798
                setPointBR = map(_throttle, -1171, 1171, -MAX_PULSES, MAX_PULSES);  //0.853828
                setPointBL = map(_throttle, -2188, 2188, -MAX_PULSES, MAX_PULSES);  //0.457005
                break;

            case 55:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1579, 1579, -MAX_PULSES, MAX_PULSES);  //0.63295
                setPointBR = map(_throttle, -1162, 1162, -MAX_PULSES, MAX_PULSES);  //0.860245
                setPointBL = map(_throttle, -2120, 2120, -MAX_PULSES, MAX_PULSES);  //0.471658
                break;

            case 56:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1566, 1566, -MAX_PULSES, MAX_PULSES);  //0.638225
                setPointBR = map(_throttle, -1153, 1153, -MAX_PULSES, MAX_PULSES);  //0.866563
                setPointBL = map(_throttle, -2056, 2056, -MAX_PULSES, MAX_PULSES);  //0.486337
                break;

            case 57:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1553, 1553, -MAX_PULSES, MAX_PULSES);  //0.643807
                setPointBR = map(_throttle, -1145, 1145, -MAX_PULSES, MAX_PULSES);  //0.872781
                setPointBL = map(_throttle, -1995, 1995, -MAX_PULSES, MAX_PULSES);  //0.501043
                break;

            case 58:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1539, 1539, -MAX_PULSES, MAX_PULSES);  //0.649697
                setPointBR = map(_throttle, -1137, 1137, -MAX_PULSES, MAX_PULSES);  //0.878896
                setPointBL = map(_throttle, -1938, 1938, -MAX_PULSES, MAX_PULSES);  //0.515776
                break;

            case 59:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1524, 1524, -MAX_PULSES, MAX_PULSES);  //0.655895
                setPointBR = map(_throttle, -1130, 1130, -MAX_PULSES, MAX_PULSES);  //0.884909
                setPointBL = map(_throttle, -1884, 1884, -MAX_PULSES, MAX_PULSES);  //0.530539
                break;

            case 60:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1509, 1509, -MAX_PULSES, MAX_PULSES);  //0.662404
                setPointBR = map(_throttle, -1122, 1122, -MAX_PULSES, MAX_PULSES);  //0.890815
                setPointBL = map(_throttle, -1833, 1833, -MAX_PULSES, MAX_PULSES);  //0.545332
                break;

            case 61:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1494, 1494, -MAX_PULSES, MAX_PULSES);  //0.669223
                setPointBR = map(_throttle, -1115, 1115, -MAX_PULSES, MAX_PULSES);  //0.896615
                setPointBL = map(_throttle, -1785, 1785, -MAX_PULSES, MAX_PULSES);  //0.560156
                break;

            case 62:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1478, 1478, -MAX_PULSES, MAX_PULSES);  //0.676353
                setPointBR = map(_throttle, -1108, 1108, -MAX_PULSES, MAX_PULSES);  //0.902304
                setPointBL = map(_throttle, -1739, 1739, -MAX_PULSES, MAX_PULSES);  //0.575011
                break;

            case 63:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1462, 1462, -MAX_PULSES, MAX_PULSES);  //0.683795
                setPointBR = map(_throttle, -1101, 1101, -MAX_PULSES, MAX_PULSES);  //0.907881
                setPointBL = map(_throttle, -1695, 1695, -MAX_PULSES, MAX_PULSES);  //0.589899
                break;

            case 64:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1446, 1446, -MAX_PULSES, MAX_PULSES);  //0.69155
                setPointBR = map(_throttle, -1094, 1094, -MAX_PULSES, MAX_PULSES);  //0.913344
                setPointBL = map(_throttle, -1653, 1653, -MAX_PULSES, MAX_PULSES);  //0.60482
                break;

            case 65:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1429, 1429, -MAX_PULSES, MAX_PULSES);  //0.699616
                setPointBR = map(_throttle, -1088, 1088, -MAX_PULSES, MAX_PULSES);  //0.918688
                setPointBL = map(_throttle, -1613, 1613, -MAX_PULSES, MAX_PULSES);  //0.619773
                break;

            case 66:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1412, 1412, -MAX_PULSES, MAX_PULSES);  //0.707996
                setPointBR = map(_throttle, -1082, 1082, -MAX_PULSES, MAX_PULSES);  //0.923911
                setPointBL = map(_throttle, -1575, 1575, -MAX_PULSES, MAX_PULSES);  //0.634759
                break;

            case 67:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1395, 1395, -MAX_PULSES, MAX_PULSES);  //0.716687
                setPointBR = map(_throttle, -1076, 1076, -MAX_PULSES, MAX_PULSES);  //0.92901
                setPointBL = map(_throttle, -1538, 1538, -MAX_PULSES, MAX_PULSES);  //0.649779
                break;

            case 68:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1377, 1377, -MAX_PULSES, MAX_PULSES);  //0.725691
                setPointBR = map(_throttle, -1070, 1070, -MAX_PULSES, MAX_PULSES);  //0.933982
                setPointBL = map(_throttle, -1504, 1504, -MAX_PULSES, MAX_PULSES);  //0.664831
                break;

            case 69:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1360, 1360, -MAX_PULSES, MAX_PULSES);  //0.735007
                setPointBR = map(_throttle, -1065, 1065, -MAX_PULSES, MAX_PULSES);  //0.938821
                setPointBL = map(_throttle, -1470, 1470, -MAX_PULSES, MAX_PULSES);  //0.679915
                break;

            case 70:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1342, 1342, -MAX_PULSES, MAX_PULSES);  //0.744634
                setPointBR = map(_throttle, -1059, 1059, -MAX_PULSES, MAX_PULSES);  //0.943524
                setPointBL = map(_throttle, -1438, 1438, -MAX_PULSES, MAX_PULSES);  //0.69503
                break;

            case 71:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1325, 1325, -MAX_PULSES, MAX_PULSES);  //0.754572
                setPointBR = map(_throttle, -1054, 1054, -MAX_PULSES, MAX_PULSES);  //0.948087
                setPointBL = map(_throttle, -1408, 1408, -MAX_PULSES, MAX_PULSES);  //0.710176
                break;

            case 72:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1307, 1307, -MAX_PULSES, MAX_PULSES);  //0.76482
                setPointBR = map(_throttle, -1049, 1049, -MAX_PULSES, MAX_PULSES);  //0.952504
                setPointBL = map(_throttle, -1378, 1378, -MAX_PULSES, MAX_PULSES);  //0.725351
                break;

            case 73:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1289, 1289, -MAX_PULSES, MAX_PULSES);  //0.775377
                setPointBR = map(_throttle, -1045, 1045, -MAX_PULSES, MAX_PULSES);  //0.956772
                setPointBL = map(_throttle, -1350, 1350, -MAX_PULSES, MAX_PULSES);  //0.740553
                break;

            case 74:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1271, 1271, -MAX_PULSES, MAX_PULSES);  //0.786242
                setPointBR = map(_throttle, -1040, 1040, -MAX_PULSES, MAX_PULSES);  //0.960884
                setPointBL = map(_throttle, -1323, 1323, -MAX_PULSES, MAX_PULSES);  //0.755782
                break;

            case 75:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1254, 1254, -MAX_PULSES, MAX_PULSES);  //0.797413
                setPointBR = map(_throttle, -1036, 1036, -MAX_PULSES, MAX_PULSES);  //0.964836
                setPointBL = map(_throttle, -1296, 1296, -MAX_PULSES, MAX_PULSES);  //0.771034
                break;

            case 76:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1236, 1236, -MAX_PULSES, MAX_PULSES);  //0.808888
                setPointBR = map(_throttle, -1032, 1032, -MAX_PULSES, MAX_PULSES);  //0.96862
                setPointBL = map(_throttle, -1271, 1271, -MAX_PULSES, MAX_PULSES);  //0.786307
                break;

            case 77:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1218, 1218, -MAX_PULSES, MAX_PULSES);  //0.820667
                setPointBR = map(_throttle, -1028, 1028, -MAX_PULSES, MAX_PULSES);  //0.972232
                setPointBL = map(_throttle, -1247, 1247, -MAX_PULSES, MAX_PULSES);  //0.801598
                break;

            case 78:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1200, 1200, -MAX_PULSES, MAX_PULSES);  //0.832747
                setPointBR = map(_throttle, -1024, 1024, -MAX_PULSES, MAX_PULSES);  //0.975664
                setPointBL = map(_throttle, -1224, 1224, -MAX_PULSES, MAX_PULSES);  //0.816905
                break;

            case 79:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1183, 1183, -MAX_PULSES, MAX_PULSES);  //0.845125
                setPointBR = map(_throttle, -1021, 1021, -MAX_PULSES, MAX_PULSES);  //0.97891
                setPointBL = map(_throttle, -1201, 1201, -MAX_PULSES, MAX_PULSES);  //0.832223
                break;

            case 80:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1165, 1165, -MAX_PULSES, MAX_PULSES);  //0.857799
                setPointBR = map(_throttle, -1018, 1018, -MAX_PULSES, MAX_PULSES);  //0.981963
                setPointBL = map(_throttle, -1179, 1179, -MAX_PULSES, MAX_PULSES);  //0.847549
                break;

            case 81:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1148, 1148, -MAX_PULSES, MAX_PULSES);  //0.870766
                setPointBR = map(_throttle, -1015, 1015, -MAX_PULSES, MAX_PULSES);  //0.984814
                setPointBL = map(_throttle, -1158, 1158, -MAX_PULSES, MAX_PULSES);  //0.862878
                break;

            case 82:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1131, 1131, -MAX_PULSES, MAX_PULSES);  //0.884024
                setPointBR = map(_throttle, -1012, 1012, -MAX_PULSES, MAX_PULSES);  //0.987456
                setPointBL = map(_throttle, -1138, 1138, -MAX_PULSES, MAX_PULSES);  //0.878206
                break;

            case 83:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1114, 1114, -MAX_PULSES, MAX_PULSES);  //0.897567
                setPointBR = map(_throttle, -1010, 1010, -MAX_PULSES, MAX_PULSES);  //0.989881
                setPointBL = map(_throttle, -1119, 1119, -MAX_PULSES, MAX_PULSES);  //0.893526
                break;

            case 84:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1097, 1097, -MAX_PULSES, MAX_PULSES);  //0.911393
                setPointBR = map(_throttle, -1007, 1007, -MAX_PULSES, MAX_PULSES);  //0.992081
                setPointBL = map(_throttle, -1100, 1100, -MAX_PULSES, MAX_PULSES);  //0.908833
                break;

            case 85:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1080, 1080, -MAX_PULSES, MAX_PULSES);  //0.925497
                setPointBR = map(_throttle, -1005, 1005, -MAX_PULSES, MAX_PULSES);  //0.994045
                setPointBL = map(_throttle, -1082, 1082, -MAX_PULSES, MAX_PULSES);  //0.924121
                break;

            case 86:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1063, 1063, -MAX_PULSES, MAX_PULSES);  //0.939874
                setPointBR = map(_throttle, -1004, 1004, -MAX_PULSES, MAX_PULSES);  //0.995765
                setPointBL = map(_throttle, -1064, 1064, -MAX_PULSES, MAX_PULSES);  //0.939382
                break;

            case 87:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1047, 1047, -MAX_PULSES, MAX_PULSES);  //0.954519
                setPointBR = map(_throttle, -1002, 1002, -MAX_PULSES, MAX_PULSES);  //0.997231
                setPointBL = map(_throttle, -1047, 1047, -MAX_PULSES, MAX_PULSES);  //0.954609
                break;

            case 88:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1031, 1031, -MAX_PULSES, MAX_PULSES);  //0.969426
                setPointBR = map(_throttle, -1001, 1001, -MAX_PULSES, MAX_PULSES);  //0.998432
                setPointBL = map(_throttle, -1031, 1031, -MAX_PULSES, MAX_PULSES);  //0.969794
                break;

            case 89:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1015, 1015, -MAX_PULSES, MAX_PULSES);  //0.984589
                setPointBR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //0.999359
                setPointBL = map(_throttle, -1015, 1015, -MAX_PULSES, MAX_PULSES);  //0.984927
                break;

            case 90:
                setPointFR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                break;

            case 91:
                setPointFR = map(_throttle, -1015, 1015, -MAX_PULSES, MAX_PULSES);  //0.984589
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1015, 1015, -MAX_PULSES, MAX_PULSES);  //0.984927
                setPointBL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //0.999359
                break;

            case 92:
                setPointFR = map(_throttle, -1031, 1031, -MAX_PULSES, MAX_PULSES);  //0.969426
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1031, 1031, -MAX_PULSES, MAX_PULSES);  //0.969794
                setPointBL = map(_throttle, -1001, 1001, -MAX_PULSES, MAX_PULSES);  //0.998432
                break;

            case 93:
                setPointFR = map(_throttle, -1047, 1047, -MAX_PULSES, MAX_PULSES);  //0.954519
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1047, 1047, -MAX_PULSES, MAX_PULSES);  //0.954609
                setPointBL = map(_throttle, -1002, 1002, -MAX_PULSES, MAX_PULSES);  //0.997231
                break;

            case 94:
                setPointFR = map(_throttle, -1063, 1063, -MAX_PULSES, MAX_PULSES);  //0.939874
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1064, 1064, -MAX_PULSES, MAX_PULSES);  //0.939382
                setPointBL = map(_throttle, -1004, 1004, -MAX_PULSES, MAX_PULSES);  //0.995765
                break;

            case 95:
                setPointFR = map(_throttle, -1080, 1080, -MAX_PULSES, MAX_PULSES);  //0.925497
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1082, 1082, -MAX_PULSES, MAX_PULSES);  //0.924121
                setPointBL = map(_throttle, -1005, 1005, -MAX_PULSES, MAX_PULSES);  //0.994045
                break;

            case 96:
                setPointFR = map(_throttle, -1097, 1097, -MAX_PULSES, MAX_PULSES);  //0.911393
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1100, 1100, -MAX_PULSES, MAX_PULSES);  //0.908833
                setPointBL = map(_throttle, -1007, 1007, -MAX_PULSES, MAX_PULSES);  //0.992081
                break;

            case 97:
                setPointFR = map(_throttle, -1114, 1114, -MAX_PULSES, MAX_PULSES);  //0.897567
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1119, 1119, -MAX_PULSES, MAX_PULSES);  //0.893526
                setPointBL = map(_throttle, -1010, 1010, -MAX_PULSES, MAX_PULSES);  //0.989881
                break;

            case 98:
                setPointFR = map(_throttle, -1131, 1131, -MAX_PULSES, MAX_PULSES);  //0.884024
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1138, 1138, -MAX_PULSES, MAX_PULSES);  //0.878206
                setPointBL = map(_throttle, -1012, 1012, -MAX_PULSES, MAX_PULSES);  //0.987456
                break;

            case 99:
                setPointFR = map(_throttle, -1148, 1148, -MAX_PULSES, MAX_PULSES);  //0.870766
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1158, 1158, -MAX_PULSES, MAX_PULSES);  //0.862878
                setPointBL = map(_throttle, -1015, 1015, -MAX_PULSES, MAX_PULSES);  //0.984814
                break;

            case 100:
                setPointFR = map(_throttle, -1165, 1165, -MAX_PULSES, MAX_PULSES);  //0.857799
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1179, 1179, -MAX_PULSES, MAX_PULSES);  //0.847549
                setPointBL = map(_throttle, -1018, 1018, -MAX_PULSES, MAX_PULSES);  //0.981963
                break;

            case 101:
                setPointFR = map(_throttle, -1183, 1183, -MAX_PULSES, MAX_PULSES);  //0.845125
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1201, 1201, -MAX_PULSES, MAX_PULSES);  //0.832223
                setPointBL = map(_throttle, -1021, 1021, -MAX_PULSES, MAX_PULSES);  //0.97891
                break;

            case 102:
                setPointFR = map(_throttle, -1200, 1200, -MAX_PULSES, MAX_PULSES);  //0.832747
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1224, 1224, -MAX_PULSES, MAX_PULSES);  //0.816905
                setPointBL = map(_throttle, -1024, 1024, -MAX_PULSES, MAX_PULSES);  //0.975664
                break;

            case 103:
                setPointFR = map(_throttle, -1218, 1218, -MAX_PULSES, MAX_PULSES);  //0.820667
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1247, 1247, -MAX_PULSES, MAX_PULSES);  //0.801598
                setPointBL = map(_throttle, -1028, 1028, -MAX_PULSES, MAX_PULSES);  //0.972232
                break;

            case 104:
                setPointFR = map(_throttle, -1236, 1236, -MAX_PULSES, MAX_PULSES);  //0.808888
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1271, 1271, -MAX_PULSES, MAX_PULSES);  //0.786307
                setPointBL = map(_throttle, -1032, 1032, -MAX_PULSES, MAX_PULSES);  //0.96862
                break;

            case 105:
                setPointFR = map(_throttle, -1254, 1254, -MAX_PULSES, MAX_PULSES);  //0.797413
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1296, 1296, -MAX_PULSES, MAX_PULSES);  //0.771034
                setPointBL = map(_throttle, -1036, 1036, -MAX_PULSES, MAX_PULSES);  //0.964836
                break;

            case 106:
                setPointFR = map(_throttle, -1271, 1271, -MAX_PULSES, MAX_PULSES);  //0.786242
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1323, 1323, -MAX_PULSES, MAX_PULSES);  //0.755782
                setPointBL = map(_throttle, -1040, 1040, -MAX_PULSES, MAX_PULSES);  //0.960884
                break;

            case 107:
                setPointFR = map(_throttle, -1289, 1289, -MAX_PULSES, MAX_PULSES);  //0.775377
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1350, 1350, -MAX_PULSES, MAX_PULSES);  //0.740553
                setPointBL = map(_throttle, -1045, 1045, -MAX_PULSES, MAX_PULSES);  //0.956772
                break;

            case 108:
                setPointFR = map(_throttle, -1307, 1307, -MAX_PULSES, MAX_PULSES);  //0.76482
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1378, 1378, -MAX_PULSES, MAX_PULSES);  //0.725351
                setPointBL = map(_throttle, -1049, 1049, -MAX_PULSES, MAX_PULSES);  //0.952504
                break;

            case 109:
                setPointFR = map(_throttle, -1325, 1325, -MAX_PULSES, MAX_PULSES);  //0.754572
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1408, 1408, -MAX_PULSES, MAX_PULSES);  //0.710176
                setPointBL = map(_throttle, -1054, 1054, -MAX_PULSES, MAX_PULSES);  //0.948087
                break;

            case 110:
                setPointFR = map(_throttle, -1342, 1342, -MAX_PULSES, MAX_PULSES);  //0.744634
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1438, 1438, -MAX_PULSES, MAX_PULSES);  //0.69503
                setPointBL = map(_throttle, -1059, 1059, -MAX_PULSES, MAX_PULSES);  //0.943524
                break;

            case 111:
                setPointFR = map(_throttle, -1360, 1360, -MAX_PULSES, MAX_PULSES);  //0.735007
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1470, 1470, -MAX_PULSES, MAX_PULSES);  //0.679915
                setPointBL = map(_throttle, -1065, 1065, -MAX_PULSES, MAX_PULSES);  //0.938821
                break;

            case 112:
                setPointFR = map(_throttle, -1377, 1377, -MAX_PULSES, MAX_PULSES);  //0.725691
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1504, 1504, -MAX_PULSES, MAX_PULSES);  //0.664831
                setPointBL = map(_throttle, -1070, 1070, -MAX_PULSES, MAX_PULSES);  //0.933982
                break;

            case 113:
                setPointFR = map(_throttle, -1395, 1395, -MAX_PULSES, MAX_PULSES);  //0.716687
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1538, 1538, -MAX_PULSES, MAX_PULSES);  //0.649779
                setPointBL = map(_throttle, -1076, 1076, -MAX_PULSES, MAX_PULSES);  //0.92901
                break;

            case 114:
                setPointFR = map(_throttle, -1412, 1412, -MAX_PULSES, MAX_PULSES);  //0.707996
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1575, 1575, -MAX_PULSES, MAX_PULSES);  //0.634759
                setPointBL = map(_throttle, -1082, 1082, -MAX_PULSES, MAX_PULSES);  //0.923911
                break;

            case 115:
                setPointFR = map(_throttle, -1429, 1429, -MAX_PULSES, MAX_PULSES);  //0.699616
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1613, 1613, -MAX_PULSES, MAX_PULSES);  //0.619773
                setPointBL = map(_throttle, -1088, 1088, -MAX_PULSES, MAX_PULSES);  //0.918688
                break;

            case 116:
                setPointFR = map(_throttle, -1446, 1446, -MAX_PULSES, MAX_PULSES);  //0.69155
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1653, 1653, -MAX_PULSES, MAX_PULSES);  //0.60482
                setPointBL = map(_throttle, -1094, 1094, -MAX_PULSES, MAX_PULSES);  //0.913344
                break;

            case 117:
                setPointFR = map(_throttle, -1462, 1462, -MAX_PULSES, MAX_PULSES);  //0.683795
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1695, 1695, -MAX_PULSES, MAX_PULSES);  //0.589899
                setPointBL = map(_throttle, -1101, 1101, -MAX_PULSES, MAX_PULSES);  //0.907881
                break;

            case 118:
                setPointFR = map(_throttle, -1478, 1478, -MAX_PULSES, MAX_PULSES);  //0.676353
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1739, 1739, -MAX_PULSES, MAX_PULSES);  //0.575011
                setPointBL = map(_throttle, -1108, 1108, -MAX_PULSES, MAX_PULSES);  //0.902304
                break;

            case 119:
                setPointFR = map(_throttle, -1494, 1494, -MAX_PULSES, MAX_PULSES);  //0.669223
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1785, 1785, -MAX_PULSES, MAX_PULSES);  //0.560156
                setPointBL = map(_throttle, -1115, 1115, -MAX_PULSES, MAX_PULSES);  //0.896615
                break;

            case 120:
                setPointFR = map(_throttle, -1509, 1509, -MAX_PULSES, MAX_PULSES);  //0.662404
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1833, 1833, -MAX_PULSES, MAX_PULSES);  //0.545332
                setPointBL = map(_throttle, -1122, 1122, -MAX_PULSES, MAX_PULSES);  //0.890815
                break;

            case 121:
                setPointFR = map(_throttle, -1524, 1524, -MAX_PULSES, MAX_PULSES);  //0.655895
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1884, 1884, -MAX_PULSES, MAX_PULSES);  //0.530539
                setPointBL = map(_throttle, -1130, 1130, -MAX_PULSES, MAX_PULSES);  //0.884909
                break;

            case 122:
                setPointFR = map(_throttle, -1539, 1539, -MAX_PULSES, MAX_PULSES);  //0.649697
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1938, 1938, -MAX_PULSES, MAX_PULSES);  //0.515776
                setPointBL = map(_throttle, -1137, 1137, -MAX_PULSES, MAX_PULSES);  //0.878896
                break;

            case 123:
                setPointFR = map(_throttle, -1553, 1553, -MAX_PULSES, MAX_PULSES);  //0.643807
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -1995, 1995, -MAX_PULSES, MAX_PULSES);  //0.501043
                setPointBL = map(_throttle, -1145, 1145, -MAX_PULSES, MAX_PULSES);  //0.872781
                break;

            case 124:
                setPointFR = map(_throttle, -1566, 1566, -MAX_PULSES, MAX_PULSES);  //0.638225
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -2056, 2056, -MAX_PULSES, MAX_PULSES);  //0.486337
                setPointBL = map(_throttle, -1153, 1153, -MAX_PULSES, MAX_PULSES);  //0.866563
                break;

            case 125:
                setPointFR = map(_throttle, -1579, 1579, -MAX_PULSES, MAX_PULSES);  //0.63295
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -2120, 2120, -MAX_PULSES, MAX_PULSES);  //0.471658
                setPointBL = map(_throttle, -1162, 1162, -MAX_PULSES, MAX_PULSES);  //0.860245
                break;

            case 126:
                setPointFR = map(_throttle, -1592, 1592, -MAX_PULSES, MAX_PULSES);  //0.62798
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -2188, 2188, -MAX_PULSES, MAX_PULSES);  //0.457005
                setPointBL = map(_throttle, -1171, 1171, -MAX_PULSES, MAX_PULSES);  //0.853828
                break;

            case 127:
                setPointFR = map(_throttle, -1604, 1604, -MAX_PULSES, MAX_PULSES);  //0.623314
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -2260, 2260, -MAX_PULSES, MAX_PULSES);  //0.442376
                setPointBL = map(_throttle, -1180, 1180, -MAX_PULSES, MAX_PULSES);  //0.847313
                break;

            case 128:
                setPointFR = map(_throttle, -1615, 1615, -MAX_PULSES, MAX_PULSES);  //0.618949
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -2337, 2337, -MAX_PULSES, MAX_PULSES);  //0.427769
                setPointBL = map(_throttle, -1189, 1189, -MAX_PULSES, MAX_PULSES);  //0.840701
                break;

            case 129:
                setPointFR = map(_throttle, -1626, 1626, -MAX_PULSES, MAX_PULSES);  //0.614885
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -2420, 2420, -MAX_PULSES, MAX_PULSES);  //0.413184
                setPointBL = map(_throttle, -1199, 1199, -MAX_PULSES, MAX_PULSES);  //0.833994
                break;

            case 130:
                setPointFR = map(_throttle, -1636, 1636, -MAX_PULSES, MAX_PULSES);  //0.611119
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -2508, 2508, -MAX_PULSES, MAX_PULSES);  //0.398618
                setPointBL = map(_throttle, -1208, 1208, -MAX_PULSES, MAX_PULSES);  //0.827191
                break;

            case 131:
                setPointFR = map(_throttle, -1645, 1645, -MAX_PULSES, MAX_PULSES);  //0.607649
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -2603, 2603, -MAX_PULSES, MAX_PULSES);  //0.384069
                setPointBL = map(_throttle, -1219, 1219, -MAX_PULSES, MAX_PULSES);  //0.820293
                break;

            case 132:
                setPointFR = map(_throttle, -1654, 1654, -MAX_PULSES, MAX_PULSES);  //0.604472
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -2706, 2706, -MAX_PULSES, MAX_PULSES);  //0.369537
                setPointBL = map(_throttle, -1229, 1229, -MAX_PULSES, MAX_PULSES);  //0.8133
                break;

            case 133:
                setPointFR = map(_throttle, -1662, 1662, -MAX_PULSES, MAX_PULSES);  //0.601587
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -2816, 2816, -MAX_PULSES, MAX_PULSES);  //0.355019
                setPointBL = map(_throttle, -1240, 1240, -MAX_PULSES, MAX_PULSES);  //0.806214
                break;

            case 134:
                setPointFR = map(_throttle, -1669, 1669, -MAX_PULSES, MAX_PULSES);  //0.59899
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -2936, 2936, -MAX_PULSES, MAX_PULSES);  //0.340513
                setPointBL = map(_throttle, -1251, 1251, -MAX_PULSES, MAX_PULSES);  //0.799033
                break;

            case 135:
                setPointFR = map(_throttle, -1675, 1675, -MAX_PULSES, MAX_PULSES);  //0.596679
                setPointFL = map(_throttle, -1000, 1000, -MAX_PULSES, MAX_PULSES);  //1
                setPointBR = map(_throttle, -3067, 3067, -MAX_PULSES, MAX_PULSES);  //0.326018
                setPointBL = map(_throttle, -1263, 1263, -MAX_PULSES, MAX_PULSES);  //0.791759
                break;
        }
    }
};
