#include <Arduino.h>
#include <SoftwareSerial.h>
#include "handler.h"

namespace ArduinoMTI {
    #define motor1Pin1 22
    #define motor1Pin2 21
    #define motor2Pin1 17
    #define motor2Pin2 16
    #define titlStopSignal 26
    bool tiltStop = false;
    int lastTiltDirection = 0;
    int forbiddenDirection = 0;

    
    void init() {
        pinMode(titlStopSignal, INPUT_PULLUP);

        ledcSetup(MOTOR_1_1_CHANNEL, freq, resolution);
        ledcSetup(MOTOR_1_2_CHANNEL, freq, resolution);
        ledcSetup(MOTOR_2_1_CHANNEL, freq, resolution);
        ledcSetup(MOTOR_2_2_CHANNEL, freq, resolution);

        ledcAttachPin(motor1Pin1, MOTOR_1_1_CHANNEL);
        ledcAttachPin(motor1Pin2, MOTOR_1_2_CHANNEL);
        ledcAttachPin(motor2Pin1, MOTOR_2_1_CHANNEL);
        ledcAttachPin(motor2Pin2, MOTOR_2_2_CHANNEL);
    }
    
    void processButton(uint8_t direction) {
        tiltStop = !digitalRead(titlStopSignal);
        
        switch (direction) {
            case DIRECTIONS::UP:
                if (tiltStop && lastTiltDirection != 1 ) {
                    ledcWrite(MOTOR_2_2_CHANNEL, panSpeedMin);
                    lastTiltDirection = 2;
                } else if (!tiltStop) {
                    ledcWrite(MOTOR_2_2_CHANNEL, panSpeedMin);
                    lastTiltDirection = 1;
                } else {
                    ledcWrite(MOTOR_2_2_CHANNEL, 0);
                    lastTiltDirection = 1;
                }
                break;
            case DIRECTIONS::DOWN:
                if (tiltStop && lastTiltDirection != 2 ) {
                    ledcWrite(MOTOR_2_1_CHANNEL, panSpeedMin);
                    lastTiltDirection = 1;
                } else if (!tiltStop) {
                    ledcWrite(MOTOR_2_1_CHANNEL, panSpeedMin);
                    lastTiltDirection = 2;
                } else {
                    ledcWrite(MOTOR_2_1_CHANNEL, 0);
                    lastTiltDirection = 2;
                }
                break;
            case DIRECTIONS::RIGHT:
                ledcWrite(MOTOR_1_1_CHANNEL, panSpeedMin);
                break;
            case DIRECTIONS::LEFT:
                ledcWrite(MOTOR_1_2_CHANNEL, panSpeedMin);
                break;
        }
    }

    void processAxis(int8_t posX, int8_t posY) {
        tiltStop = !digitalRead(titlStopSignal);
        int posYmod = abs(posY);
        int posXmod = abs(posX);
        if (posYmod >= 12) {
            int posYdir = posY > 0 ? 2 : 1;
            int currentTiltMotor = MOTOR_2_1_CHANNEL;

            int range = 5; 
            int posYDefault = 3;
            int reading = map(posYmod, 0, 127, 0, range);
            int posYspeed = 0;
            posYspeed = tiltSpeedMin + (panTiltSpeedMax - tiltSpeedMin) / range * reading;


            Serial.println(posYspeed);

            if (posYdir == 2 && posYspeed) {
                if (tiltStop && lastTiltDirection != 1 ) {
                    ledcWrite(MOTOR_2_2_CHANNEL, posYspeed);
                    lastTiltDirection = 2;
                } else if (!tiltStop) {
                    ledcWrite(MOTOR_2_2_CHANNEL, posYspeed);
                    lastTiltDirection = 1;
                } else {
                    ledcWrite(MOTOR_2_2_CHANNEL, 0);
                    lastTiltDirection = 1;
                }
            }

            if (posYdir == 1 && posYspeed) {
                if (tiltStop && lastTiltDirection != 2 ) {
                    ledcWrite(MOTOR_2_1_CHANNEL, posYspeed);
                    lastTiltDirection = 1;
                } else if (!tiltStop) {
                    ledcWrite(MOTOR_2_1_CHANNEL, posYspeed);
                    lastTiltDirection = 2;
                } else {
                    ledcWrite(MOTOR_2_1_CHANNEL, 0);
                    lastTiltDirection = 2;
                }
            }
        } else {
            ledcWrite(MOTOR_2_2_CHANNEL, 0);
            ledcWrite(MOTOR_2_1_CHANNEL, 0);
        }
        
        if (posXmod >= 12) {
            int posXdir = posX > 0 ? 2 : 1;
            int currentTiltMotor = MOTOR_1_1_CHANNEL;

            int range = 5; 
            int posXDefault = 3;
            int reading = map(posXmod, 0, 127, 0, range);
            int posXspeed = 0;
            posXspeed = panSpeedMin + (panTiltSpeedMax - panSpeedMin) / range * reading;

            if (posXdir == 2 && posXspeed) {
                ledcWrite(MOTOR_1_1_CHANNEL, posXspeed);
            }
            if (posXdir == 1 && posXspeed) {
                ledcWrite(MOTOR_1_2_CHANNEL, posXspeed);
            }
        } else {
            ledcWrite(MOTOR_1_2_CHANNEL, 0);
            ledcWrite(MOTOR_1_1_CHANNEL, 0);
        }
    }   
}
