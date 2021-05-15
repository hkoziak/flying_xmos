#include <Arduino.h>
#include <Servo.h> 

#include "globalvariables.h"

Servo d1;
Servo d2;
Servo d3;
Servo d4;

constexpr double MinValue = 1000;
constexpr double MamValue = 2000;
constexpr double DiffValue = MamValue - MinValue;

uint64_t LastUpdateEngine = 0;

void setupDriver() 
{ 
    d1.attach(8);
    d1.writeMicroseconds(1000);
    
    d2.attach(9);
    d2.writeMicroseconds(1000);
    
    d3.attach(10);
    d3.writeMicroseconds(1000);
    
    d4.attach(11);
    d4.writeMicroseconds(1000);
}

double calcMicroseconds(double p)
{
    return MinValue + (DiffValue * (p / 100.0));
} 

void setValues(double p1, double p2, double p3, double p4) // range [0; 100]
{
    d1.writeMicroseconds(calcMicroseconds(p1));
    d2.writeMicroseconds(calcMicroseconds(p2));
    d3.writeMicroseconds(calcMicroseconds(p3));
    d4.writeMicroseconds(calcMicroseconds(p4));

#ifdef DEBUG_OUTPUT
    Serial.print(" engines (1 2 3 4): ");
    Serial.print(speedRequest);
    Serial.print(" ");
    Serial.print(speedRequest - ((calcMicroseconds(p1) - MinValue) / 10.0));
    Serial.print(" ");
    Serial.print(speedRequest - ((calcMicroseconds(p2) - MinValue) / 10.0));
    Serial.print(" ");
    Serial.print(speedRequest - ((calcMicroseconds(p3) - MinValue) / 10.0));
    Serial.print(" ");
    Serial.println(speedRequest - ((calcMicroseconds(p4) - MinValue) / 10.0));
#endif

#ifdef QUADRO_DRAW_OUTPUT
// #ENGP:100,100,100,100,100#
    Serial.print("#ENGP:");
    Serial.print(speedRequest);
    Serial.print(",");
    Serial.print((calcMicroseconds(p1) - MinValue) / 10.0);
    Serial.print(",");
    Serial.print((calcMicroseconds(p2) - MinValue) / 10.0);
    Serial.print(",");
    Serial.print((calcMicroseconds(p3) - MinValue) / 10.0);
    Serial.print(",");
    Serial.print((calcMicroseconds(p4) - MinValue) / 10.0);
    Serial.println("#");
// Serial.println();
#endif
} 