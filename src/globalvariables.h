
#ifndef GLOBAL_VARIABLES_H
#define GLOBAL_VARIABLES_H

// requested paarameters [0, 100]%
double speedRequest = 0;
double pitchRequest = 0;
double rollRequest = 0;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// #define DEBUG_OUTPUT

// #define QUADRO_DRAW_OUTPUT

#endif