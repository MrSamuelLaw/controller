#include <Arduino.h>
#include <DCMotor.h>
#include <Encoder.h>
#include <StepperMotor.h>
#include <ZController.h>
#include <XController.h>
#include <BrushController.h>
#include <DCMotor.h>


//    _____ ______ _______ _    _ _____
//   / ____|  ____|__   __| |  | |  __ \
//  | (___ | |__     | |  | |  | | |__) |
//   \___ \|  __|    | |  | |  | |  ___/
//   ____) | |____   | |  | |__| | |
//  |_____/|______|  |_|   \____/|_|

// // set up the stepper motors
StepperMotor z_stepper(4, 5, HIGH);      // stepper for z axis
ZController z_controller(z_stepper, 13);  // controller for z axis
StepperMotor x_stepper(7, 8, LOW);       // stepper for x axis
XController x_controller(x_stepper, 9);  // controller for x axis

// set up the DC motor
DCMotor brush_motor(12, 11, 10, HIGH);                 // dc motor for the brush array
Brush_Controller brush_controller(brush_motor, 2, 3);  // controller for the brush array

// create the time variable
unsigned long time;

// create pause variable and function
int PAUSE_PIN = 18;
bool PAUSE_FLAG = true;
void TOGGLE_PAUSE()
{
  PAUSE_FLAG = !PAUSE_FLAG;  // toggle the pause state
}

void update_all(unsigned long time)
{
  // if not paused
  if (!PAUSE_FLAG)
  {
    z_controller.update(time);      // update the z-controller
    x_controller.update(time);      // update the x-controller
    brush_controller.update(time);  // update the brush controller
  }

  // if paused
  else
  {
    brush_controller.e_stop(time);  // stop the brushes
  }
}

bool lower_tray()
{
  Serial.println("lowering...");
  z_controller.set_linear_vel(10);     // mm/s
  z_controller.set_linear_target(90);  // mm
  while (true)
  {
    time = millis();                         // update the time
    update_all(time);                        // update the device
    if (!z_controller.delta()){return true;}  // exit the function once down
  }
}

bool raise_tray()
{
  Serial.println("raising");
  z_controller.set_linear_vel(10);  // mm/s
  z_controller.set_target(0);       // steps
  while (true)
  {
    time = millis();                          // update the time
    update_all(time);                         // update the device
    if (!z_controller.delta()){return true;}  // exit the function once down
  }
}

bool index_tray(int i)
{
  Serial.println("indexing...");
  x_controller.set_linear_vel(10);         // mm/s
  float target = (float)i*32.995;          // mm
  x_controller.set_linear_target(target);  // mm
  while (true)
  {
    time = millis();                          // update the time
    update_all(time);                         // update the device
    if (!x_controller.delta()){return true;}  // exit the function once down
  }
}

void setup()
{
  // start coms
  Serial.begin(9600);
  Serial.println("\nstarting...");

  // // home the z axis
  z_controller.home();
  z_controller.set_linear_vel(15);        // mm/s

  // // home the x axis
  x_controller.home();
  x_controller.set_linear_vel(10);        // mm/s

  // set the brush speed
  brush_controller.set_speed(120);     // rpm

  // attach pause interrupt
  attachInterrupt(
    digitalPinToInterrupt(PAUSE_PIN),  // pause pin 18
    TOGGLE_PAUSE,                      // function to call
    RISING                             // state that pin 18 must read for function to be called
  );

  // wait for user to press go
  while (PAUSE_FLAG)
  {
    delay(1);  // prevent loop that goes so fast user input is not registered.
  }
}

//   ________      ________ _   _ _______   _      ____   ____  _____
//  |  ____\ \    / /  ____| \ | |__   __| | |    / __ \ / __ \|  __ \
//  | |__   \ \  / /| |__  |  \| |  | |    | |   | |  | | |  | | |__) |
//  |  __|   \ \/ / |  __| | . ` |  | |    | |   | |  | | |  | |  ___/
//  | |____   \  /  | |____| |\  |  | |    | |___| |__| | |__| | |
//  |______|   \/   |______|_| \_|  |_|    |______\____/ \____/|_|

int i = 1;                  // sub cycle count
void loop()
{
  // run 10 number of cycles
  while (i <= 11)
  {
    lower_tray();
    raise_tray();
    index_tray(i);
    i++;
    // stop the machine at the end
    if (i == 10){PAUSE_FLAG = true;}
  }
  update_all(time);
}