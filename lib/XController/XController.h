#ifndef XController_H
#define XController_H

#include <Arduino.h>
#include <StepperMotor.h>

class XController
{
  public:
    // constructor
    XController(StepperMotor &z_stepper_motor, uint8_t probe_pin):
    // initializer list
    _z_stepper{z_stepper_motor},  // grab the stepper motor object
    _current_position{0},         // current position starts at zero
    _probe_pin{probe_pin}         // the pin that goes high when probing
    {
      // setup code goes here
      pinMode(_probe_pin, INPUT);

    }

    /* Function sets the target position relative to the zero position
    using the steps per revolution and lead screw constant */
    void set_linear_target(float mm)
    {
      // unit analysis used to compute steps mm*(steps/rev)/(mm/rev) [=] steps
      long steps = (long)(mm*(_SPR/(2*M_PI))/_R);  // compute target steps using
      set_target(steps);                           // set the target in steps
    }

    void set_linear_vel(float mm_per_sec)
    {
      long steps_per_second = (long)(mm_per_sec*(_SPR/(2*M_PI))/_R);  // compute the target steps/second
      set_vel(steps_per_second);                                      // set the target steps/sercond
    }

    /* Function that sets the current position in steps */
    void set_position(long position)
    {
      _current_position = position;
    }

    /* Function that sets the target position in steps */
    void set_target(long position_steps)
    {
      _target_position = position_steps;                                                     // set the new target
      long delta = _target_position - _current_position;                                     // compute delta
      (delta < 0) ? _z_stepper.set_dir(_z_stepper.CW) : _z_stepper.set_dir(_z_stepper.CCW);  // set direction
    }

    /* Function that sets the target velocity in steps per second */
    void set_vel(int steps_per_second)
    {
      _delay = 1000/steps_per_second;
    }

    /* Calls the stepper motor to  move if it's not at it's target and
    enough time has passed. */
    bool update(unsigned long &cur_time)
    {
      if ((cur_time - _interval_start >= _delay) && (_current_position - _target_position))
      {
        _z_stepper.step();
        _current_position += (_z_stepper.cur_dir() == _z_stepper.CCW) ? 1 : -1;
        _interval_start = millis();
        return true;
      }
      return false;
    }

    /* Function that operates in joint space to determine
    where the starting position should be for the z axis assembly. It turns
    the stepper motor one step at a time until it pushed the probe button,
    then it backs off one step at a time until the probe button is not pressed,
    this position is considered "home" or zero. */
    void home()
    {
      /* -------------- move up the rod to push the button -------------- */
      set_vel(200);              // set speed
      set_target(-10000);        // set target past the top of the rod
      long count = 0;            // set the count
      long max_count = 50000;    // set max count
      Serial.println("going up...");
      while (count < max_count)  // start the while loop
      {
        unsigned long now = millis();                  // update the clock
        if (digitalRead(_probe_pin) == HIGH) {break;}  // check if the probe has been hit
        else if (update(now)){count++;}                // if a step was taken, update the count
      }

      /* ------------- move back until button is not pushed ------------- */
      count = 0;                 // reset the count
      max_count = 10;            // set maxcount to small number
      set_position(0);           // set temp home
      set_target(50);            // set target
      Serial.println("going down...");
      while (count < max_count)  // start the while loop
      {
        unsigned long now = millis();                                 // update the clock
        if (digitalRead(_probe_pin) == LOW){set_position(0); break;}  // check if button is no longer pressed
        else if (update(now)){count++;}                               // if step taken, update the count
      }
      Serial.println("homed..");
    }

  private:
    StepperMotor _z_stepper;        // stepper motor object
    uint8_t _probe_pin;             // pin number
    long _current_position;         // steps
    long _target_position;          // steps
    unsigned long _delay;           // time object
    unsigned long _interval_start;  // timer object
    const unsigned long _SPR{400};  // steps per revolution
    const float _R{6.11};           // radius in mm
};

#endif