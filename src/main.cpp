#include <Arduino.h>


//    _____ _                _____ _____ ______  _____
//   / ____| |        /\    / ____/ ____|  ____|/ ____|
//  | |    | |       /  \  | (___| (___ | |__  | (___
//  | |    | |      / /\ \  \___ \\___ \|  __|  \___ \
//  | |____| |____ / ____ \ ____) |___) | |____ ____) |
//   \_____|______/_/    \_\_____/_____/|______|_____/

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
    StepperMotor(int step_pin, int dir_pin, uint8_t ClockWise):
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
      (_target_position < _current_position) ?_z_stepper.set_dir(_z_stepper.CW) :
                                              _z_stepper.set_dir(_z_stepper.CCW);

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
        _current_position += _z_stepper.cur_dir() == _z_stepper.CCW ? 1 : -1;
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
      unsigned long now = millis();
      _ping_vec[0] = _ping_vec[1];
      _ping_vec[1] = now;
      // filter between pings to prevent it from picking up
      // the signal bounce after the initially detected ping
      if (_ping_vec[1] - _ping_vec[0] > 2) {_ping_new = now;}
    }

    // setter for the speed
    void set_speed(unsigned int rpm)
    {
      _target_rpm = rpm;                         // rot per min
      _target_delay = 60000/rpm;                 // delay in ms
      _target_freq = (float)1000/_target_delay;  // signal in hz
    }

    // function that implements the PI controller for
    // controlling the rps of the dc motor
    void update(unsigned long &now)
    {
      // back up filter to determine if the time delta
      // detected is not higher than the motor's max speed.
      noInterrupts();  // turn off interrupts
      if (_ping_new - _ping_old > _t_cutoff)
      {
        _time_new = _ping_new;  // t' = p'
        _time_old = _ping_old;  // to = po
        _ping_old = _ping_new;  // po = p'
      }
      interrupts();    // turn on interrupts

      // compute dt terms
      // _dt_signal = (now - _time_old > _target_delay) ? now - _time_old : _time_new - _time_old;
      _dt_signal = (now - _time_new > _target_delay) ? now - _time_new : _time_new - _time_old;
      _dt_loop = _loop_time_old ? now - _loop_time_old : 0;

      // compute error terms
      _signal_freq = _dt_signal ? (float)1000/_dt_signal : 0.0;        // signal in Hz
      _E_new = (float)(_target_freq - _signal_freq);                   // diff of the signal in Hz
      _E_sum += (float)_E_new*_dt_loop/1000;                           // error integration term _E*dt
      _dEdt =_dt_loop ? (float)(_E_new - _E_old)*1000/_dt_loop : 0.0;  // error differentiation term d_E/dt, = 0 if dt = 0

      // compute controller terms
      _P = (float)_KP*_E_new;        // proportional control component
      _I = (float)_KI*_E_sum;        // integral control component
      _D = (float)_KD*_dEdt;         // derivative components
      _dV = _K*(_P + _I + _D);       // change in voltage signal
      _V += _dV;                     // current voltage signal
      _V = constrain(_V, 0, 12);      // constrain to prevent run away
      _brush_motor.set_voltage(_V);  // write out the voltage

      // prep for the next loop
      _loop_time_old = now;
      _E_old = _E_new;

      // print stuff out
      Serial.println(_V);
      // Serial.print(" dt = "); Serial.print(_dt_signal);
      // Serial.print(" P = "); Serial.print(_P);
      // Serial.print(" I = "); Serial.print(_I);
      // Serial.print(" D = "); Serial.print(_D);
      // Serial.print(" V = "); Serial.println(_V);
    }

  private:
    DCMotor _brush_motor;
    // cutoff and targets
    unsigned long _t_cutoff;
    unsigned int _target_rpm{0};
    unsigned long _target_delay{0};
    float _target_freq{0};

    // signal time variables
    unsigned long _ping_vec[2]{0, 0};
    unsigned long _ping_new{0};
    unsigned long _ping_old{0};
    unsigned long _ping_last{0};
    unsigned long _time_new{0};
    unsigned long _time_old{0};
    unsigned long _dt_signal{0};
    float _signal_freq{0};

    // loop time variables
    unsigned long _dt_loop{0};
    unsigned long _loop_time_old{0};

    // voltage and control variables
    float _E_new{0};
    float _dEdt{0};
    float _E_sum{0};
    float _E_old{0};
    float _P{0};
    float _I{0};
    float _D{0};
    float _K{1e-1};
    float _KP{0.1};
    float _KI{1e-4};
    float _KD{1e-3};
    float _dV{0};
    float _V{0};
};



//    _____ ______ _______ _    _ _____
//   / ____|  ____|__   __| |  | |  __ \
//  | (___ | |__     | |  | |  | | |__) |
//   \___ \|  __|    | |  | |  | |  ___/
//   ____) | |____   | |  | |__| | |
//  |_____/|______|  |_|   \____/|_|

// set up the stepper motors
StepperMotor z_stepper(3, 4, HIGH);  // stepper for z axis
Z_Controller z_controller(z_stepper);  // controller

// set up the DC motor
DCMotor brush_motor(11, 10, 5, HIGH);                  // dc motor for the brush array
Brush_Controller brush_controller(brush_motor, 3000);  // controller for the brush array

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

  // stepper motor commands
  z_controller.set_vel(200);
  // z_controller.set_position(0);
  z_controller.move_to(1000);

  // dc motor test commands
  brush_controller.set_speed(190);
}

//   ________      ________ _   _ _______   _      ____   ____  _____
//  |  ____\ \    / /  ____| \ | |__   __| | |    / __ \ / __ \|  __ \
//  | |__   \ \  / /| |__  |  \| |  | |    | |   | |  | | |  | | |__) |
//  |  __|   \ \/ / |  __| | . ` |  | |    | |   | |  | | |  | |  ___/
//  | |____   \  /  | |____| |\  |  | |    | |___| |__| | |__| | |
//  |______|   \/   |______|_| \_|  |_|    |______\____/ \____/|_|

void loop()
{
  time = millis();
  z_controller.update(time);
  brush_controller.update(time);
}