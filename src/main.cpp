#include <Arduino.h>
#include <DCMotor.h>
#include <StepperMotor.h>
#include <ZController.h>
#include <BrushController.h>

//    _____ ______ _______ _    _ _____
//   / ____|  ____|__   __| |  | |  __ \
//  | (___ | |__     | |  | |  | | |__) |
//   \___ \|  __|    | |  | |  | |  ___/
//   ____) | |____   | |  | |__| | |
//  |_____/|______|  |_|   \____/|_|

// set up the stepper motors
StepperMotor z_stepper(3, 4, HIGH);       // stepper for z axis
ZController z_controller(z_stepper, 6);  // controller

// set up the DC motor
DCMotor brush_motor(11, 10, 5, HIGH);                  // dc motor for the brush array
Brush_Controller brush_controller(brush_motor, 3000);   // controller for the brush array

// create the time variable
unsigned long time;

void setup()
{
  // start coms
  Serial.begin(9600);
  Serial.println("\nstarting...");

  // attach isr to the interupt pin
  attachInterrupt(
    digitalPinToInterrupt(2),
    [](){brush_controller.read_high();},
    RISING);

  // home the z axis
  z_controller.home();
  z_controller.set_linear_vel(10);
  z_controller.set_linear_target(50);
}

//   ________      ________ _   _ _______   _      ____   ____  _____
//  |  ____\ \    / /  ____| \ | |__   __| | |    / __ \ / __ \|  __ \
//  | |__   \ \  / /| |__  |  \| |  | |    | |   | |  | | |  | | |__) |
//  |  __|   \ \/ / |  __| | . ` |  | |    | |   | |  | | |  | |  ___/
//  | |____   \  /  | |____| |\  |  | |    | |___| |__| | |__| | |
//  |______|   \/   |______|_| \_|  |_|    |______\____/ \____/|_|

void loop()
{
  time = millis();            // update the time
  z_controller.update(time);  // update the z-controller
  // brush_controller.update(time);
}