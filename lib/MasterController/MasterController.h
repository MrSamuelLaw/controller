#ifndef MasterController_H
#define MasterController_H

#include <Arduino.h>
#include <DCMotor.h>
#include <Encoder.h>
#include <StepperMotor.h>
#include <ZController.h>
#include <XController.h>
#include <BrushController.h>
#include <DCMotor.h>


namespace MasterController
{
    // create the time variable
    unsigned long time;

    // set up the stepper motors
    StepperMotor z_stepper(4, 5, HIGH);       // stepper for z axis
    ZController z_controller(z_stepper, 13);  // controller for z axis
    StepperMotor x_stepper(7, 8, LOW);        // stepper for x axis
    XController x_controller(x_stepper, 9);   // controller for x axis

    // set up the DC motor
    DCMotor brush_motor(12, 11, 10, HIGH);                 // dc motor for the brush array
    Brush_Controller brush_controller(brush_motor, 2, 3);  // controller for the brush array

    // create pause variable and function
    int PAUSE_PIN = 18;
    bool PAUSE_FLAG = true;
    void TOGGLE_PAUSE()
    {
        PAUSE_FLAG = !PAUSE_FLAG;  // toggle the pause state
    }

    // function that services the other controllers
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

    // function to lower the tray to a specified height
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

    // function to raise the tray to a specified height
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

    // function to move the tray over a specified amount.
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
}

#endif