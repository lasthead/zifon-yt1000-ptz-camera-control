#include <Arduino.h>

#ifndef handler_H
#define handler_H

namespace ArduinoMTI {
    const int MOTOR_1_1_CHANNEL = 0;
    const int MOTOR_1_2_CHANNEL = 1;
    const int MOTOR_2_1_CHANNEL = 2;
    const int MOTOR_2_2_CHANNEL = 3;

    const int panSpeedMin = 150;
    const int tiltSpeedMin = 160;
    const int panTiltSpeedMax = 255;
    // int panCurrentSpeed = panSpeedMin;
    // int tiltCurrentSpeed = tiltSpeedMin;
    const int freq = 20000;
    const int resolution = 8;

    enum DIRECTIONS {
        UP,
        RIGHT,
        DOWN,
        LEFT,
    };

    void init();
    
    void processAxis(int8_t posX, int8_t posY);
    void processButton(uint8_t direction);
}

#endif