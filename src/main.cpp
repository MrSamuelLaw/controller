#include <Arduino.h>
#include <Encoder.h>

class DCMotor {
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
    void set_voltage(int voltage)
    {
      if (voltage < 0)
      {
        _current_direction = !_current_direction;  // update the direction flag
        set_direction(_current_direction);         // send cmd to motor
        voltage = -1*voltage;                      // change sign on voltage
      }
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


class Z_Controller{
  public:
    // constructor
    Z_Controller(StepperMotor &z_stepper_motor,
                 Encoder &z_encoder):
    // initializer list
    _z_stepper{z_stepper_motor},
    _z_encoder{z_encoder},
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
    Encoder _z_encoder;
    long _current_position;
    long _target_position;
    unsigned long _delay;
    unsigned long _interval_start;
};


class Brush_Controller {
  public:
    // constructor
    Brush_Controller(DCMotor &brush_motor):
    // initializer list
    _brush_motor{brush_motor}
    {
      // setup code goes here
    }

    // isr that reads the signal from the hall effect sensor
    // indicating that a full revolution of the dc motor has been achieved
    void read_high()
    {
      _high_time_1 = _high_time_2;
      _high_time_2 = millis();
    }

    // setter for the speed
    void set_speed(uint8_t rps)
    {
      _target_rps = rps;
    }

    // function that implements the PI controller for
    // controlling the rps of the dc motor
    void update()
    {
      // make it an actual PI controller
      _rps = (double)(1000/(_high_time_2 - _high_time_1));
      if (_rps < 1.0){_rps = 1;}
      float dV = 0.001*(_target_rps - _rps);
      _V = _V + dV;
      _brush_motor.set_voltage(_V);
      Serial.print(_rps);
      Serial.print(" ");
      Serial.println(_V);

    }



  private:
    DCMotor _brush_motor;
    unsigned long _high_time_1 = 0;
    unsigned long _high_time_2 = 1000;
    double _rps = 0;
    double _V = 0;
    uint8_t _target_rps = 0;
};

// set up the stepper motors
Encoder z_encoder(4, 5);                          // encoder for z axis
StepperMotor z_stepper(50, 51, HIGH);               // stepper for z axis
Z_Controller z_controller(z_stepper, z_encoder);  // controller

// set up the DC motor
DCMotor brush_motor(52, 53, 5, HIGH);  // dc motor for the brush array
Brush_Controller brush_controller(brush_motor);         // controller for the brush array

// create the time variable
unsigned long time;


void setup()
{
  Serial.begin(9600);             // start coms
  Serial.println("starting...");
  // z controller commands
  z_controller.set_position(0);
  z_controller.move_to(2000);
  z_controller.set_vel(50);
  // brush_motor_commands
  brush_controller.set_speed(2);
  // attach isr to the interupt pin
  attachInterrupt(digitalPinToInterrupt(2), [](){brush_controller.read_high();}, RISING);
}


void loop()
{
  time = millis();
  z_controller.update(time);
  brush_controller.update();
}