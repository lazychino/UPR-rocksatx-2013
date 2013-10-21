/*****************************************************************
 *  Author: Pedro A. Melendez
 *  rsx2013.h: library for controlling UPR's RocksatX 2013 payload
 * 
 *****************************************************************/
#include <Arduino.h>
#include "linearEasyDriver.h"

/*******************************************************************
 * 
 *              arduino pins name definitions
 * 
 *******************************************************************/

// firgelli L12 linear servo pins
#define L12_rc 31

// firgelli L16 linear actuator pins 
#define L16_open 39 
#define L16_close 41
#define L16_enable 37

// easy driver pins
#define easyDriver_dir 47
#define easyDriver_ena 43
#define easyDriver_stp 45 
#define easyDriver_limit 7 

// valve solenoid 
#define valve_activate 35
#define valve_enable 33

// parallel pins
#define parallel_1 30
#define parallel_2 32
#define parallel_3 34
#define parallel_4 36
#define parallel_5 38
#define parallel_6 40
#define parallel_7 42
#define parallel_8 44

//analogs inputs
#define proximity A3
#define L12_position A4
#define Launch A0
#define SkirtOff A1
#define PowerOff A2

//GoPro cameras
//side camera pins
#define camera1_pwr 22
#define camera1_trig 24 
#define camera1_wifi 26
#define camera1_stat 28

//bottom camera
#define camera2_pwr 23
#define camera2_trig 25
#define camera2_wifi 27
#define camera2_stat 29

/*******************************************************************
 * 
 *                      constants
 * 
 *******************************************************************/


#define CAMERA_Launch 1500
#define CAMERA_Experi 1000
#define CAMERA_Earth  2000

#define TRAVEL_PER_STEP 0.00015625

/* ---------------------------Notes----------------------------------
 * actuator linear travel/step= 0.00015625
 * 
 * A8 to A13 are the piezo signals
 */
 
/* ----------TO DO--------------
 * 
 * -flight function
 * -function to move the camera1
 * -debuggin commands and help docs 
 * 
 * 
 */
class rsx2013
{
private:
    Servo L12;
    linearEasyDriver stepperActuator;
    void initPins();
    //~ void sdInit();
    String* input(HardwareSerial uart );
    //~ File data;
    
public:
    rsx2013();
    ~rsx2013(){}
    void openRail();
    void closeRail();
    void parallelLine( const uint8_t );
    void debugMode();
    void flight();
    void moveL16( const bool );
    void readSensors();
    void testParallel();
    void testSolenoid();
    void openSolenoid();
    void closeSolenoid();
    void dataDump();
    void startCameras();
    void stopCameras();
};


rsx2013::rsx2013()
{
    L12.attach(L12_rc);
    stepperActuator.attach(easyDriver_dir, easyDriver_stp, easyDriver_ena, TRAVEL_PER_STEP, easyDriver_limit);
    //sdInit();
    initPins();
}


void rsx2013::parallelLine( const uint8_t state )  // receives a byte and send it via parallel line
{
    digitalWrite(parallel_1, state & 0x1 ); // parallel status BOOT
    digitalWrite(parallel_2, (state & 0x2)>>1 );
    digitalWrite(parallel_3, (state & 0x4)>>2 );
    digitalWrite(parallel_4, (state & 0x8)>>3 );
    digitalWrite(parallel_5, (state & 0x10)>>4 );
    digitalWrite(parallel_6, (state & 0x20)>>5 );
    digitalWrite(parallel_7, (state & 0x40)>>6 );
    digitalWrite(parallel_8, (state & 0x80)>>7 );
    delay(10);
}

/*
void rsx2013::sdInit()
{
    //code from SD tutorial of arduino.cc
    
    // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
    // Note that even if it's not used as the CS pin, the hardware SS pin
    // (10 on most Arduino boards, 53 on the Mega) must be left as an output
    // or the SD library functions will not work.
    
    pinMode(53, OUTPUT);

    if( !SD.begin(53) ) 
    {
        Serial.begin(115200);
        Serial.println("\n\rCard failed, or not present");
        return;
    }
}
*/

void rsx2013::initPins()
{
/*
 * INITIALIZE ARDUINO PINS SIGNALS
 */
    pinMode(L12_rc, OUTPUT);
    digitalWrite(L12_rc ,LOW);
    
    pinMode(L16_enable, OUTPUT);
    digitalWrite(L16_enable ,LOW);
    pinMode(L16_open, OUTPUT);
    digitalWrite(L16_open ,LOW);
    pinMode(L16_close, OUTPUT);
    digitalWrite(L16_close ,LOW);
    
    pinMode(valve_activate, OUTPUT);
    digitalWrite(valve_activate ,LOW);
    pinMode(valve_enable, OUTPUT);
    digitalWrite(valve_enable ,LOW);
    
    pinMode(parallel_1, OUTPUT);
    pinMode(parallel_2, OUTPUT);
    pinMode(parallel_3, OUTPUT);
    pinMode(parallel_4, OUTPUT);
    pinMode(parallel_5, OUTPUT);
    pinMode(parallel_6, OUTPUT);
    pinMode(parallel_7, OUTPUT);
    pinMode(parallel_8, OUTPUT);
    parallelLine(0);
    
    pinMode(camera1_pwr, OUTPUT);
    pinMode(camera1_trig, OUTPUT);
    pinMode(camera1_wifi, OUTPUT);
    pinMode(camera2_pwr, OUTPUT);
    pinMode(camera2_trig, OUTPUT);
    pinMode(camera2_wifi, OUTPUT);
}

void rsx2013::moveL16( const bool dir )
{
    Serial.println("start!");
    int dirPin;
    
    if(dir)
        dirPin = L16_open;
    else
        dirPin = L16_close;
        
    digitalWrite(dirPin, HIGH);
    digitalWrite(L16_enable, HIGH);
    delay(30000);
    digitalWrite(dirPin, LOW);
    digitalWrite(L16_enable, LOW);
    Serial.println("done");
}

void rsx2013::testSolenoid()
{
    Serial.println("start!");
    digitalWrite(valve_enable, HIGH);
    for(int i=0; i < 5; i++)
    {
        digitalWrite(valve_activate, HIGH);
        delay(1000);
        digitalWrite(valve_activate, LOW);
        delay(1000);
    }
    digitalWrite(valve_enable, LOW);
    Serial.println("done");
}

void rsx2013::openSolenoid()
{
    Serial.println("start!");
    digitalWrite(valve_enable, HIGH);
    digitalWrite(valve_activate, HIGH);
    Serial.println("done");
}

void rsx2013::closeSolenoid()
{
    Serial.println("start!");
    digitalWrite(valve_enable, LOW);
    digitalWrite(valve_activate, LOW);
    Serial.println("done");
}

void rsx2013::openRail()
{
    while(analogRead(proximity) > 100){}
    digitalWrite(L16_open, HIGH);
    digitalWrite(L16_enable, HIGH);
    stepperActuator.move(OPEN, double(4.6));      // open 4 inches 409600
    digitalWrite(L16_open, LOW);
    digitalWrite(L16_enable, LOW);
}

void rsx2013::closeRail()
{
    digitalWrite(L16_close, HIGH);
    digitalWrite(L16_enable, HIGH);
    stepperActuator.moveLimit(CLOSE, double(4.45));    
    digitalWrite(L16_close, LOW);
    digitalWrite(L16_enable, LOW);
}

void rsx2013::startCameras()
{
    digitalWrite(camera1_pwr, HIGH);
    digitalWrite(camera2_pwr, HIGH);
    delay(250);
    digitalWrite(camera1_pwr, LOW);
    digitalWrite(camera2_pwr, LOW);
}

void rsx2013::stopCameras()
{
    digitalWrite(camera1_pwr, HIGH);
    digitalWrite(camera2_pwr, HIGH);
    delay(2600);
    digitalWrite(camera1_pwr, LOW);
    digitalWrite(camera2_pwr, LOW);
}

void rsx2013::readSensors()
{
    Serial.println("Sensors");
    Serial.print("Launch: ");
    Serial.println(analogRead(A0));
    Serial.print("Skirt Deploy: ");
    Serial.println(analogRead(A1));
    Serial.print("Power off: ");
    Serial.println(analogRead(A2));
    Serial.print("Proximity: ");
    Serial.println(analogRead(A3));
    Serial.print("Camera position: ");
    Serial.println(analogRead(A4));
    Serial.print("Limit Switch: ");
    Serial.println(digitalRead(easyDriver_limit));
    
    Serial.println("\nPiezos elements:");
    Serial.print("1: ");
    Serial.println(analogRead(A8));
    Serial.print("2: ");
    Serial.println(analogRead(A9));
    Serial.print("3: ");
    Serial.println(analogRead(A10));
    Serial.print("4: ");
    Serial.println(analogRead(A11));
    Serial.print("5: ");
    Serial.println(analogRead(A12));
    Serial.print("6: ");
    Serial.println(analogRead(A13));
    
}

void rsx2013::testParallel()
{
    //{1,2,4,8,16,32,64,128,0,255}
    parallelLine(1);
    delay(1500);
    parallelLine(2);
    delay(1500);
    parallelLine(4);
    delay(1500);
    parallelLine(8);
    delay(1500);
    parallelLine(16);
    delay(1500);
    parallelLine(32);
    delay(1500);
    parallelLine(64);
    delay(1500);
    parallelLine(128);
    delay(1500);
    parallelLine(255);
    delay(1500);
    parallelLine(0);
}

/*
void rsx2013::dataDump()
{
    data = SD.open("data.csv", FILE_READ);
    
    if (data)
    {
        while (data.available()) 
        {
            Serial.write(data.read());
        }
        data.close();
    }  
    // if the file isn't open, pop up an error:
    else 
    {
        Serial.println("error opening data file");
    } 
}
*/

void rsx2013::flight()
{
    parallelLine(1); // on
    if( digitalRead(easyDriver_limit) == 1 )
            closeRail();
    L12.writeMicroseconds(CAMERA_Launch);
    
    parallelLine(2); // ready for launch
    
    while(analogRead(Launch) < 1000 ){}
    
    parallelLine(3); // lift off!
    
    digitalWrite(valve_enable, HIGH);   // open valve
    digitalWrite(valve_activate, HIGH);
    parallelLine(4);
    
    startCameras(); // cameras start recording
    parallelLine(5);
    
    while(analogRead(SkirtOff) < 1000) {}
    
    parallelLine(6);
    
    digitalWrite(valve_activate, LOW); //close valve
    digitalWrite(valve_enable, LOW);
    
    parallelLine(7);
    
    L12.writeMicroseconds(CAMERA_Experi);       // move camera
    
    parallelLine(8);
    
    openRail();
    
    parallelLine(9);
    
    while(analogRead(PowerOff) < 1000 )
    {
        // collect data
    }
    
    parallelLine(11);
    
    L12.writeMicroseconds(CAMERA_Earth);     // move camera
    
    parallelLine(12);
    
    closeRail();
    
    parallelLine(13);
}

void rsx2013::debugMode()
{
    Serial.begin(115200);
    
    String *cmd = new String();
    String *mesg = new String();
    
    *mesg =  "\n\r _____ _____ _____    _____         _           _   __ __ ";
    *mesg += "\n\r|  |  |  _  | __  |  | __  |___ ___| |_ ___ ___| |_|  |  |";
    *mesg += "\n\r|  |  |   __|    -|  |    -| . |  _| '_|_ -| .'|  _|-   -|";
    *mesg += "\n\r|_____|__|  |__|__|  |__|__|___|___|_,_|___|__,|_| |__|__|";
    *mesg += "\n\r";
    *mesg += "\n\rDebug mode (enter \'help\' for options)";
    Serial.println(*mesg);
    
    while( 1 )
    {
        Serial.print("you@upr_payload# ");    
        cmd =  input(Serial);
        
        if(*cmd == "")
        {
            /* pass */
        }
        else if(*cmd == "help")
        {
            *mesg = "Commands:\n\r";
            *mesg +=" **PLEASE BE SURE DEPLOYMENT AREA IS CLEAR WHEN ACTIVATING DEPLOYABLES**\n\r";
            *mesg +=" openL16 - opens firgelli L16 railbox actuator\n\r";
            *mesg +=" closeL16 - closes firgelli L16 railbox actuator\n\r";
            *mesg +=" openStepper - opens Stepper motor actuator 4.58 inches \n\r";
            *mesg +=" closeStepper - closes Stepper motor actuator 4.58 inches\n\r";
            *mesg +=" closeStepperLimit - closes Stepper motor actuator until Limit is toggle\n\r";
            *mesg +=" openRail - opens Railbox box with both actuator, same as the real flight\n\r";
            *mesg +=" closeRail - closes Railbox box with both actuator, same as the real flight\n\r";
            *mesg +=" cameraPosLaunch - moves camera to Launch position\n\r";
            *mesg +=" cameraPosExperiment - moves camera to look at the experiment\n\r";
            *mesg +=" cameraPosReentry - moves camera to look down to Earth\n\n\r";
            *mesg +=" testValve - makes the solenoid valve open and close\n\r";
            *mesg +=" openValve - opens the solenoid valve\n\r";
            *mesg +=" closeVaslve - closes solenoid valve\n\r";
            *mesg +=" readSensors - displays currents values of all sensors\n\r";
            *mesg +=" testParallelLines - send the sequence of numbers {1,2,4,8,16,32,64,128,0,255} to parallel lines\n\r";
            //~ *mesg +=" dumpData - display all the content inside the data.csv file\n\r";
            Serial.println(*mesg);
        }
        else if(*cmd == "openL16")
        {
            moveL16(OPEN);
        }
        else if(*cmd == "closeL16")
        {
            moveL16(CLOSE);
        }
        else if(*cmd == "openStepper")
        {
            stepperActuator.move(OPEN, double(4.6));
        }
        else if(*cmd == "openStepperSlow")
        {
            stepperActuator.moveSlow(OPEN, double(4.58));
        }
        else if(*cmd == "closeStepper")
        {
            stepperActuator.move(CLOSE, double(4.58));
        }
        else if(*cmd == "closeStepperL")
        {
            stepperActuator.moveLimit(CLOSE, double(4.45));
        }
        else if(*cmd == "openRail")
        {
            openRail();
        }
        else if(*cmd == "closeRail")
        {
            closeRail();
        }
        else if(*cmd == "cameraPosLaunch")
        {
            L12.writeMicroseconds(CAMERA_Launch);
        }
        else if(*cmd == "cameraPosExperiment")
        {
            L12.writeMicroseconds(CAMERA_Experi);
        }
        else if(*cmd == "cameraPosReentry")
        {
            L12.writeMicroseconds(CAMERA_Earth);
        }
        else if(*cmd == "testValve")
        {
            testSolenoid();
        }
        else if(*cmd == "openValve")
        {
            openSolenoid();
        }
        else if(*cmd == "closeValve")
        {
            closeSolenoid();
        }
        else if(*cmd == "readSensors")
        {
            readSensors();
        }
        else if(*cmd == "testParallelLines")
        {
            for(int i=0; i < 3; i++) 
                testParallel();
        }
        else if(*cmd == "startCameras")
        {
            startCameras();
        }
        else if(*cmd == "stopCameras")
        {
            stopCameras();
        }
        //~ else if(*cmd == "dumpData")
        //~ {
            //~ dataDump();
        //~ }
        else
        {
            *mesg = "command \'" + *cmd +"\' not found!\n\r";
            *mesg += "enter 'help' for available commands";
            Serial.println(*mesg);
        }
    } 
}

String* rsx2013::input(HardwareSerial uart )
{ 
    String *in = new String("");
    String *cmd = NULL;
    char rx = 0;
        
    while( rx != 13 ) // finish by pressing return key (ENTER)
    {
        if( uart.available() )
        {
            rx = uart.read();
            
            if(rx == 8) // backspace
            {
                if( in->length() > 0 )
                {
                    uart.print("\b \b");
                    *in = in->substring(0, (in->length()-1) );
                }
            }
            else if(rx == 27) // ignore ESC or special key
            {
                //cmd = new String("");
                delay(10);
                while( uart.available() )
                {
                    rx = uart.read();
                    *cmd += rx;
                }
                //uart.print("comando ");
                //uart.println(*cmd);
            }
            else if(rx > 32 && rx < 127) // any character or number
            {
                uart.print(rx);
                *in += rx;
            }
            else
            {
            }
        }
    }
    Serial.print("\n\r");
    return in;
}
