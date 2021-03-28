#ifndef DCMotor_H
#define DCMotor_H

#include <Arduino.h>

class DCMotor
{
  public:
    uint8_t CCW;
    uint8_t CW;

    // constructor
    DCMotor(uint8_t INA_pin, uint8_t INB_pin, uint8_t PWM_pin, uint8_t CCW):
    // initializer list
    _INA_pin{INA_pin},
    _INB_pin{INB_pin},
    _PWM_pin{PWM_pin},
    CCW{CCW},
    CW{!CCW}
    {
      // setup code goes here
      pinMode(_INA_pin, OUTPUT);
      pinMode(_INB_pin, OUTPUT);
      pinMode(_PWM_pin, OUTPUT);
      set_direction(CCW);
    }

    // sets the motor direction using the CCW or CW variables
    void set_direction(uint8_t direction) {
      digitalWrite(_INA_pin, direction);
      digitalWrite(_INB_pin, !direction);
    }

    // writes the voltage to the pin
    void set_voltage(float voltage)
    {
      if (voltage < 0)
      {
        _current_direction = !_current_direction;  // update the direction flag
        set_direction(_current_direction);         // send cmd to motor
        voltage = -1*voltage;                      // change sign on voltage
      }

      voltage = constrain(voltage, 0, 12);         // constrain the amount
      long pwm = map(voltage, 0, 12, 0, 255);      // map voltage to pwm
      analogWrite(_PWM_pin, pwm);                  // send the signal
    }

  private:
    uint8_t _INA_pin;
    uint8_t _INB_pin;
    uint8_t _PWM_pin;
    uint8_t _current_direction;

};

#endif