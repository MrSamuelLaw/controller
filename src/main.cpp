#include <Arduino.h>
#include <DCMotor.h>
#include <Encoder.h>
#include <StepperMotor.h>
#include <ZController.h>
#include <XController.h>
#include <BrushController.h>

//    _____ ______ _______ _    _ _____
//   / ____|  ____|__   __| |  | |  __ \
//  | (___ | |__     | |  | |  | | |__) |
//   \___ \|  __|    | |  | |  | |  ___/
//   ____) | |____   | |  | |__| | |
//  |_____/|______|  |_|   \____/|_|

// set up the stepper motors
StepperMotor z_stepper(4, 5, HIGH);      // stepper for z axis
ZController z_controller(z_stepper, 6);  // controller for z axis
StepperMotor x_stepper(7, 8, LOW);      // stepper for x axis
XController x_controller(x_stepper, 9);  // controller for x axis

// set up the DC motor
DCMotor brush_motor(12, 11, 10, HIGH);                          // dc motor for the brush array
Encoder brush_encoder(2, 3);                                    // encoder for dc motor
Brush_Controller brush_controller(brush_motor, brush_encoder);  // controller for the brush array

// create the time variable
unsigned long time;

void setup()
{
  // start coms
  Serial.begin(9600);
  Serial.println("\nstarting...");

  // home the z axis
  z_controller.home();
  z_controller.set_linear_vel(10);     // mm/s
  z_controller.set_linear_target(30);  // mm

  // home the x axis
  x_controller.home();
  x_controller.set_linear_vel(10);     // mm/s
  x_controller.set_linear_target(30);  // mm

  // set the brush speed
  brush_controller.set_speed(900);     // rpm
}

//   ________      ________ _   _ _______   _      ____   ____  _____
//  |  ____\ \    / /  ____| \ | |__   __| | |    / __ \ / __ \|  __ \
//  | |__   \ \  / /| |__  |  \| |  | |    | |   | |  | | |  | | |__) |
//  |  __|   \ \/ / |  __| | . ` |  | |    | |   | |  | | |  | |  ___/
//  | |____   \  /  | |____| |\  |  | |    | |___| |__| | |__| | |
//  |______|   \/   |______|_| \_|  |_|    |______\____/ \____/|_|

void loop()
{
  time = millis();                // update the time
  z_controller.update(time);      // update the z-controller
  x_controller.update(time);      // update the x-controller
  brush_controller.update(time);  // update the brush controller
}