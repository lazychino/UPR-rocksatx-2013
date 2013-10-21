/*
 * linearEasyDriver.h
 * 
 * a library for controlling a single stepper motor using 
 * the Easy Driver www.schmalzhaus.com/EasyDriver
 *  
 * 
 * Author: Pedro A. Melendez
 * email: lazy.chino@gmail.com
 *
 */

#include <Arduino.h>

#define OPEN 1
#define CLOSE 0

class linearEasyDriver
{
private:
    // arduino pins for controlling
    uint8_t direction;
    uint8_t enable;
    uint8_t step;
    uint8_t o_limit;
    uint8_t c_limit;
    long double travel_step;
    
public:
    linearEasyDriver(){}
    ~linearEasyDriver(){}
    void attach(uint8_t, uint8_t, uint8_t, long double);
    void attach(uint8_t, uint8_t, uint8_t, long double, uint8_t);
    void move(const bool, const uint32_t);
    void move(const bool, double);
    void moveLimit(const bool, const double);
    void moveLimit(const bool, const uint32_t);
    void moveSlow(const bool, const uint32_t);
    void moveSlow(const bool, double);
    void moveSlowLimit(const bool);
};

void linearEasyDriver::attach(uint8_t dir, uint8_t stp, uint8_t ena, long double distance)
{
    direction = dir;
    step = stp;
    enable = ena;
    travel_step = distance;
    
    pinMode(dir, OUTPUT);
    pinMode(stp, OUTPUT);
    pinMode(ena, OUTPUT);
    digitalWrite(ena, HIGH);
}

void linearEasyDriver::attach(uint8_t dir, uint8_t stp, uint8_t ena, long double distance, uint8_t limitClose)
{
    direction = dir;
    step = stp;
    enable = ena;
    c_limit = limitClose;
    travel_step = distance;
    
    pinMode(dir, OUTPUT);
    pinMode(stp, OUTPUT);
    pinMode(ena, OUTPUT);
    pinMode(limitClose, INPUT_PULLUP);
    digitalWrite(ena, HIGH);
}


/**
 * move with acelaration
 * 
 * dir: {0, 1}
 * steps: quatity of microsteps to move, 3200 steps = 1 revolution
 */
void linearEasyDriver::move(const bool dir, const uint32_t steps)
{    
    
    if(dir)
    {
	digitalWrite(direction, HIGH);	// set direction outward
    }
    else
    {
	digitalWrite(direction, LOW);	// set direction inward
    }
    
    int period = 75;			// set initial speed
    digitalWrite(enable, LOW);		// enable board
    //~ Serial.println(steps);
    for(uint32_t i=steps; i > 0; i--)
    {
	delayMicroseconds(period);		//one square pulse is send to the driver equals one step
	digitalWrite(step, HIGH);
	delayMicroseconds(period);
	digitalWrite(step, LOW);
        
	if( i >= 3200)
	{ 
	    if(period > 18 && i%400 == 0)		// on the first revolutions
		period--;			// the period of the pulse is decrease to accelerate
	}
	else if( i < 3200)
	{ 
	    if( i%20 == 0)		// on the last revolution period
		period++;				// increase to decelerate
	}
	else{}
    }
    digitalWrite(enable, HIGH);			// disable board
}

/**
 * move with acelaration
 * 
 * dir: {0, 1}
 * dist: distance to move the linear stepper actuator (inches)
 */
void linearEasyDriver::move(const bool dir, double dist)
{    
    uint32_t steps = (dist/travel_step)*16;
    move(dir, steps);
}

void linearEasyDriver::moveLimit(const bool dir, const double dist)
{    
    uint32_t steps = (dist/travel_step)*16;
    moveLimit(dir, steps);
}

void linearEasyDriver::moveLimit(const bool dir, const uint32_t steps)
{    
    
    if(dir)
    {
	digitalWrite(direction, HIGH);	// set direction outward
    }
    else
    {
	digitalWrite(direction, LOW);	// set direction inward
    }
    
    int period = 75;			// set initial speed
    digitalWrite(enable, LOW);		// enable board
    
    for(uint32_t i=steps; i > 0; i--)
    {
	delayMicroseconds(period);		//one square pulse is send to the driver equals one step
	digitalWrite(step, HIGH);
	delayMicroseconds(period);
	digitalWrite(step, LOW);
	
	if(digitalRead(c_limit) == 0)
	    break;
	    
	if( i >= 3200)
	{ 
	    if(period > 18 && i%400 == 0)		// on the first revolutions
		period--;			// the period of the pulse is decrease to accelerate
	}
	else if( i < 3200)
	{ 
	    if(period < 75 && i%20 == 0)		// on the last revolution period
		period++;				// increase to decelerate
	}
	else{}
    }
    int done = 0;
    while(1)
    {
	delayMicroseconds(period);		//one square pulse is send to the driver equals one step
	digitalWrite(step, HIGH);
	delayMicroseconds(period);
	digitalWrite(step, LOW);
	    
    	if(digitalRead(c_limit) == 0)
	    done++;
	if(done == 32000)    
	    break;
    }
    digitalWrite(enable, HIGH);			// disable board
}

void linearEasyDriver::moveSlowLimit(const bool dir)
{    
    
    if(dir)
    {
	digitalWrite(direction, HIGH);	// set direction outward
    }
    else
    {
	digitalWrite(direction, LOW);	// set direction inward
    }
    
    int period = 50;			// set initial speed
    digitalWrite(enable, LOW);		// enable board
    
    for(uint32_t i=1000000; 1; i--)
    {
	delayMicroseconds(period);		//one square pulse is send to the driver equals one step
	digitalWrite(step, HIGH);
	delayMicroseconds(period);
	digitalWrite(step, LOW);
        
	if(digitalRead(c_limit) == 0)
	    break;
    }
    digitalWrite(enable, HIGH);			// disable board
}

/**
 * move with constant slow speep
 * 
 * dir: {0, 1}
 * steps: quatity of microsteps to move, 3200 steps = 1 revolution
 */
void linearEasyDriver::moveSlow(const bool dir, const uint32_t steps)
{    
    
    if(dir)
    {
	digitalWrite(direction, HIGH);	// set direction outward
    }
    else
    {
	digitalWrite(direction, LOW);	// set direction inward
    }
    
    int period = 50;			// set initial speed
    digitalWrite(enable, LOW);		// enable board
    //~ Serial.println(steps);
    for(uint32_t i=steps; i > 0; i--)
    {
	delayMicroseconds(period);		//one square pulse is send to the driver equals one step
	digitalWrite(step, HIGH);
	delayMicroseconds(period);
	digitalWrite(step, LOW);
	//~ Serial.println("step");
    }
    digitalWrite(enable, HIGH);			// disable board
}

void linearEasyDriver::moveSlow(const bool dir, double dist)
{
    uint32_t steps = (dist/travel_step)*16;
    moveSlow(dir, steps);
}
