#ifndef BrushController_H
#define BrushContoller_H

#include <Arduino.h>
#include <DCMotor.h>

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
      if (_ping_vec[1] - _ping_vec[0] > 20) {_ping_new = now;}
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
      // Serial.print(" dt = "); Serial.print(_dt_signal);
      // Serial.print("rpm = "); Serial.print(60000/_dt_signal);
      // Serial.print(" P = "); Serial.print(_P);
      // Serial.print(" I = "); Serial.print(_I);
      // Serial.print(" D = "); Serial.print(_D);
      Serial.print(" V = "); Serial.println(_V);
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
    float _K{1e-3};
    float _KP{1};
    float _KI{1e-3};
    float _KD{2e-1};
    float _dV{0};
    float _V{0};
};

#endif