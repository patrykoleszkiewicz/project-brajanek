#define SERVO 7

#define MOT_BL_DIR 22
#define MOT_BL_PWM 6
#define MOT_BL_ENCA 18
#define MOT_BL_ENCB 23

#define MOT_BR_DIR 24
#define MOT_BR_PWM 5
#define MOT_BR_ENCA 21
#define MOT_BR_ENCB 29

#define MOT_FL_DIR 26
#define MOT_FL_PWM 4
#define MOT_FL_ENCA 19
#define MOT_FL_ENCB 25

#define MOT_FR_DIR 28
#define MOT_FR_PWM 3
#define MOT_FR_ENCA 20
#define MOT_FR_ENCB 27


const int8_t STEER_TRIM = -4;
const uint8_t STEER_MAX = 130;
const uint8_t STEER_MIN = 50;

const float WHEELBASE_LENGHT = 284;
const float WHEELBASE_WIDTH = 267;

#include "Servo.h"

Servo turner;

class Driver
{
  public:
    void init()
    {
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

        turner.attach(SERVO);
        turner.write(90);
    }

    void update(unsigned long current_millis, int throttle, int steer)
    {
        setSteer(current_millis, steer);

        setThrottle(current_millis, throttle);
    }

  private:
    unsigned long _throttle_timer;
    unsigned long _turner_timer;

    int _throttle = 0;
    bool _reverse = 0;
    bool _brakes = false;

    uint8_t _steer_comp = 0;
    uint8_t _motor_comp = 0;

    int16_t _turner_pos = 90;

    void setThrottle(unsigned long current_millis, int throttle)
    {
        if(current_millis - _throttle_timer < _motor_comp)
        {
            return;
        }
        _throttle_timer = current_millis;

        bool reverse = throttle < 0;
        throttle = map(abs(throttle), 0, 1000, 0, 255);

        if(_brakes)
        {
            _throttle = 0;
        }

        if(_throttle == 0)
        {
            _reverse = reverse;
        }

        bool fast;
        if(_reverse != reverse)
        {
            throttle = 0;
            fast = true;
        }
        else
        {
            fast = false;
        }

        if(_throttle > throttle)
        {
            --_throttle;
            if(fast) _throttle -= 2;
        }
        else if(_throttle < throttle)
        {
            ++_throttle;
            if(fast) _throttle += 2;
        }

        _throttle = constrain(_throttle, 0, 255);

        digitalWrite(MOT_FR_DIR, _reverse);
        digitalWrite(MOT_FL_DIR, _reverse);
        digitalWrite(MOT_BR_DIR, _reverse);
        digitalWrite(MOT_BL_DIR, _reverse);

        digitalDifferential();
    }

    void digitalDifferential()
    {
        if(abs(_turner_pos - 90) < 2)
        {
            analogWrite(MOT_FR_PWM, _throttle);
            analogWrite(MOT_FL_PWM, _throttle);
            analogWrite(MOT_BR_PWM, _throttle);
            analogWrite(MOT_BL_PWM, _throttle);
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

        float throttle_FR = _throttle * (TR_FR / TR_max);
        float throttle_FL = _throttle * (TR_FL / TR_max);
        float throttle_BR = _throttle * (TR_BR / TR_max);
        float throttle_BL = _throttle * (TR_BL / TR_max);

        analogWrite(MOT_FR_PWM, abs(throttle_FR));
        analogWrite(MOT_FL_PWM, abs(throttle_FL));
        analogWrite(MOT_BR_PWM, abs(throttle_BR));
        analogWrite(MOT_BL_PWM, abs(throttle_BL));
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
