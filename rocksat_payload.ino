#include <Servo.h>
//#include <SD.h>
#include "rsx2013.h"

void setup()
{
    pinMode(12, INPUT_PULLUP);
    rsx2013 payload;


    if(digitalRead(12) == 0)
    {
        payload.debugMode();
    }
    else
    {
        payload.flight();
    }
}

void loop() { /* nothing! */ }
