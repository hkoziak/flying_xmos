#include <Arduino.h>
#include <Servo.h> 

#include "globalvariables.h"

Servo driver1;
Servo driver2;
Servo driver3;
Servo driver4;

constexpr float MinValue = 1000;
constexpr float MaxValue = 2000;
constexpr float DiffValue = MaxValue - MinValue;

uint64_t LastUpdateEngine = 0;

void setupDriver() 
{ 
    driver1.attach(8);
    driver1.writeMicroseconds(1000);
    
    driver2.attach(9);
    driver2.writeMicroseconds(1000);
    
    driver3.attach(10);
    driver3.writeMicroseconds(1000);
    
    driver4.attach(11);
    driver4.writeMicroseconds(1000);
}

float calcMicroseconds(float p)
{
    return MinValue + (DiffValue * (p / 100.0));
} 

void setValues(float p1, float p2, float p3, float p4) // range [0; 100]
{
    driver1.writeMicroseconds(calcMicroseconds(p1));
    driver2.writeMicroseconds(calcMicroseconds(p2));
    driver3.writeMicroseconds(calcMicroseconds(p3));
    driver4.writeMicroseconds(calcMicroseconds(p4));

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

#ifdef QUADRO_SERIAL_PLOTTER
// #ENGP:100,100,100,100,100#
    // Serial.print(speedRequest);
    // Serial.print(" ");

    // Serial.print("   (pitchIn, rollIn): ");

    // Serial.print(pitchInput);
    // Serial.print(" ");
    Serial.print(rollInput);
    Serial.print(" ");

    // Serial.print("   (pitchOut, rollOut): ");

    // Serial.print(pitchOutput);
    // Serial.print(" ");
    Serial.print(rollOutput);
    Serial.println(" ");

    // Serial.print("");
    // Serial.print((calcMicroseconds(p1) - MinValue) / 10.0);
    // Serial.print("");
    // Serial.print((calcMicroseconds(p2) - MinValue) / 10.0);
    // Serial.print("");
    // Serial.print((calcMicroseconds(p3) - MinValue) / 10.0);
    // Serial.print("");
    // Serial.print((calcMicroseconds(p4) - MinValue) / 10.0);
    // Serial.println("");
// Serial.println();
#endif

} 