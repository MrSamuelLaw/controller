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


class StepperMotor
{
  public:
    // constructer
    StepperMotor(uint8_t step_pin, uint8_t dir_pin, uint8_t ClockWise):
    _step_pin{step_pin},  // step pin definition
    _dir_pin{dir_pin},    // direction pin number
    CW{ClockWise},        // ClockWise is either HIGH or LOW
    CCW{!ClockWise}       // CCW is the oposite of ClockWise
    {
      // StepperMotor setup code goes here
      pinMode(_dir_pin, OUTPUT);     // dir_pin is output
      pinMode(_step_pin, OUTPUT);    // step_pin is output
      digitalWrite(_dir_pin, CCW);  // set the direction to CCW by default
    }

    // step command in the CCW direction
    void set_dir(uint8_t dir)
    {
      digitalWrite(_dir_pin, dir);  // set direction to CCW
    }

    // implements the step command
    void step()
    {
      digitalWrite(_step_pin, HIGH);
      digitalWrite(_step_pin, LOW);
    }

    // variables
    const uint8_t CW;
    const uint8_t CCW;

  private:
    const uint8_t _step_pin;
    const uint8_t _dir_pin;
};


class Z_Controller
{
  public:
    // constructor
    Z_Controller(StepperMotor &z_stepper_motor):
    // initializer list
    _z_stepper{z_stepper_motor},
    _current_position{0}
    {
      // setup code goes here
    }

    void set_position(long position)
    {
      // re-implement this using the encoder when you have one
      _current_position = position;
    }

    void move_to(int position_steps)
    {
      _target_position = position_steps;
    }

    void set_vel(int steps_per_second)
    {
      _delay = 1000/steps_per_second;
    }

    void update(unsigned long &cur_time)
    {
      if ((cur_time - _interval_start >= _delay) && (_current_position - _target_position))
      {
        _z_stepper.step();
        _current_position += 1;
        _interval_start = millis();
      }
    }

  private:
    StepperMotor _z_stepper;
    long _current_position;
    long _target_position;
    unsigned long _delay;
    unsigned long _interval_start;
};


class Brush_Controller
{
  public:
    // variables
    const unsigned int MAX_RPM;

    // constructor
    Brush_Controller(DCMotor &brush_motor, const unsigned int MAX_RPM):
    // initializer list
    _brush_motor{brush_motor},
    MAX_RPM{MAX_RPM}
    {
      _t_cutoff = 60000/MAX_RPM;
    }

    // isr that reads the signal from the hall effect sensor
    // indicating that a full revolution of the dc motor has been achieved
    void read_high()
    {
      _ping = millis();
    }

    // setter for the speed
    void set_speed(unsigned int rpm)
    {
      _target_rpm = rpm;
      _target_delay = 60000/rpm;
    }

    // function that implements the PI controller for
    // controlling the rps of the dc motor
    void update(unsigned long &cur_time)
    {
      // compute all the the time variables shifts
      noInterrupts();  // turn off interrupts
      if (_ping - _last_ping > _t_cutoff)
      {
        _new_time = _ping;       // t' = p'
        _old_time = _last_ping;  // to = po
        _last_ping = _ping;      // po = p'
      }
      interrupts();    // turn on interrupts

      // compute dt terms
      _dt_signal = (cur_time - _old_time > _target_delay) ? cur_time - _old_time :
                                                            _new_time - _old_time;
      _dt_loop = _dt_loop ? cur_time - _old_time : 0;

      // compute controller terms
      _E = (long)(_dt_signal - _target_delay);  // error term
      _Esum += (float)_E*_dt_loop;              // error integration term
      _P = (float)_KP*_E;                       // proportional control component
      _I = (float)_KI*_Esum;                    // integral control component
      _dV = _P + _I;                            // change in voltage signal
      _V += _dV;                                // current voltage signal
      _brush_motor.set_voltage(_V);             // write out the voltage

      Serial.print(" dt = "); Serial.print(_dt_signal);
      Serial.print(" Esum = "); Serial.print(_Esum);
      Serial.print(" dV = "); Serial.print(_dV);
      Serial.print(" V = "); Serial.println(_V);
    }

  private:
    DCMotor _brush_motor;
    // cutoff and targets
    unsigned long _t_cutoff;
    unsigned int _target_rpm{0};
    unsigned long _target_delay{0};
    // time variables
    unsigned long _ping{0};
    unsigned long _last_ping{0};
    unsigned long _new_time{0};
    unsigned long _old_time{0};
    unsigned long _dt_signal{0};
    unsigned long _dt_loop{0};
    unsigned long _old_cur_time{0};
    // voltage and control variables
    long _E{0};
    float _Esum{0};
    float _P{0};
    float _KP{1e-6};
    float _I{0};
    float _KI{1};
    float _dV{0};
    float _V{0};
};

// set up the stepper motors
StepperMotor z_stepper(50, 51, HIGH);  // stepper for z axis
Z_Controller z_controller(z_stepper);  // controller

// set up the DC motor
DCMotor brush_motor(11, 10, 5, HIGH);            // dc motor for the brush array
Brush_Controller brush_controller(brush_motor, 200);  // controller for the brush array

// create the time variable
unsigned long time;


void setup()
{
  Serial.begin(9600);             // start coms
  Serial.println("\nstarting...");

  // attach isr to the interupt pin
  attachInterrupt(
    digitalPinToInterrupt(2),
    [](){brush_controller.read_high();},
    RISING);

  // dc motor test commands
  brush_controller.set_speed(120);
}


void loop()
{
  time = millis();
  z_controller.update(time);
  brush_controller.update(time);
}