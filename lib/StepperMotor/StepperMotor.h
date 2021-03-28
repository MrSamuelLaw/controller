#ifndef StepperMotor_H
#define StepperMotor_H

#include <Arduino.h>

class StepperMotor
{
  public:
    // constructer
    StepperMotor(int step_pin, int dir_pin, uint8_t ClockWise):
    _step_pin{step_pin},   // step pin definition
    _dir_pin{dir_pin},     // direction pin number
    CW{ClockWise},         // ClockWise is either HIGH or LOW
    CCW{!ClockWise}        // CCW is the oposite of ClockWise
    {
      // StepperMotor setup code goes here
      pinMode(_dir_pin, OUTPUT);     // dir_pin is output
      pinMode(_step_pin, OUTPUT);    // step_pin is output
      digitalWrite(_dir_pin, CCW);  // set the direction to CCW by default
    }

    // step command in the CCW direction
    void set_dir(uint8_t dir)
    {
      _cur_dir = dir;
      digitalWrite(_dir_pin, dir);  // set direction to CCW
    }

    // implements the step command
    void step()
    {
      digitalWrite(_step_pin, HIGH);
      digitalWrite(_step_pin, LOW);
    }

    uint8_t cur_dir()
    {
      return _cur_dir;
    }

    // variables
    const uint8_t CW;
    const uint8_t CCW;

  // private:
    const uint8_t _step_pin;
    const uint8_t _dir_pin;
    uint8_t _cur_dir;
};

#endif