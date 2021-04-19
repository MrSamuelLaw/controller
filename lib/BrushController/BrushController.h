#ifndef BrushController_H
#define BrushContoller_H

#include <Arduino.h>
#include <DCMotor.h>
#include <Encoder.h>

class Brush_Controller
{
  public:

    // constructor
    Brush_Controller(DCMotor &brush_motor, int encoder_pin1, int encoder_pin2):
    // initializer list
    _brush_motor{brush_motor},
    _encoder{Encoder(encoder_pin1, encoder_pin2)}
    {
      // setup code goes here
      _time_old = millis();
    }

    void set_encoder(Encoder &encoder)
    {
      _encoder = encoder;
    }

    // setter for the speed
    void set_speed(unsigned int rpm)
    {
      _target_rpm = rpm;                         // rot per min
      _target_delay = 60000/rpm;                 // ms/rot
      _target_freq = (float)1000/_target_delay;  // rot/sec = hz
    }

    int32_t read()
    {
      _q = _encoder.read();
      _encoder.write(0);
      _q = abs(_q);
      return _q;
    }

    // function that implements the PI controller for
    // controlling the rps of the dc motor
    void update(unsigned long &now)
    {
      // compute frequency
      _dq = read();  // read change in angle
      _dt = now - _time_old;  // compute change in time

      // compute error terms if _dt != 0
      if (_dt)
      {
        _signal_freq = (float)1000*_dq/(_dt*_CPR);      // rev/s = hz
        _E_new = (float)(_target_freq - _signal_freq);  // diff of the signal in Hz
        _E_sum += (float)_E_new*_dt/1000;               // error integration term _E*dt
        _dEdt = (float)(_E_new - _E_old)*1000/_dt;      // error differentiation term d_E/dt, = 0 if dt = 0
      }
      // compute error terms if _dt == 0
      else
      {
        _signal_freq = 0.0;                             // rev/s = hz
        _E_new = (float)(_target_freq - _signal_freq);  // diff of the signal in Hz
        _E_sum += (float)_E_new*_dt/1000;               // error integration term _E*dt
        _dEdt =  0.0;                                   // error differentiation term d_E/dt, = 0 if dt = 0
      }

      // compute controller terms
      _P = (float)_KP*_E_new;        // proportional control component
      _I = (float)_KI*_E_sum;        // integral control component
      _D = (float)_KD*_dEdt;         // derivative components
      _dV = _K*(_P + _I + _D);       // change in voltage signal
      _V += _dV;                     // current voltage signal
      _V = constrain(_V, 0, 12);     // constrain to prevent run away
      _brush_motor.set_voltage(_V);  // write out the voltage

      // prep for the next loop
      _time_old = now;
      _E_old = _E_new;

      // print stuff out
      // Serial.print(" dt = "); Serial.print(_dt_signal);
      Serial.print("rpm = "); Serial.println(60*_signal_freq);
      // Serial.print(" P = "); Serial.print(_P);
      // Serial.print(" I = "); Serial.print(_I);
      // Serial.print(" D = "); Serial.print(_D);
      // Serial.print(" V = "); Serial.println(_V);
    }

  private:
    DCMotor _brush_motor;
    Encoder _encoder;

    // constants
    const int _CPR = 192;

    // cutoff and targets
    unsigned long _t_cutoff;
    unsigned int _target_rpm{0};
    unsigned long _target_delay{0};
    float _target_freq{0};

    // signal variables
    int32_t _q{0};
    long _dq{0};
    long _dt{1};
    float _time_old{0};
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
    float _KP{1.2};
    float _KI{1e-3};
    float _KD{2e-3};
    float _dV{0};
    float _V{0};
};

#endif