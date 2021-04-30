#include <Arduino.h>
#include <MasterController.h>

using namespace MasterController;

void setup()
{
  // start coms
  Serial.begin(9600);
  Serial.println("\nstarting...");

  // // home the z axis
  z_controller.home();
  z_controller.set_linear_vel(15);  // mm/s

  // // home the x axis
  x_controller.home();
  x_controller.set_linear_vel(10);  // mm/s

  // set the brush speed
  brush_controller.set_speed(120);  // rpm

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


int i = 1;       // sub cycle counter
int count = 11;  // total number of sub cycles
void loop()
{
  // run 10 number of cycles
  while (i <= count)
  {
    lower_tray();
    raise_tray();
    index_tray(i);
    i++;
    // stop the machine at the end
    if (i == count){PAUSE_FLAG = true;}
  }
  update_all(time);
}