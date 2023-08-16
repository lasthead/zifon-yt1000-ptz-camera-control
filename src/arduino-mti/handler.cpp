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
        switch (direction) {
            case DIRECTIONS::UP:
                ledcWrite(MOTOR_2_1_CHANNEL, panTiltSpeedMax);
                break;
            case DIRECTIONS::RIGHT:
                ledcWrite(MOTOR_1_1_CHANNEL, panTiltSpeedMax);
                break;
            case DIRECTIONS::DOWN:
                ledcWrite(MOTOR_2_2_CHANNEL, panTiltSpeedMax);
                break;
            case DIRECTIONS::LEFT:
                ledcWrite(MOTOR_1_2_CHANNEL, panTiltSpeedMax);
                break;
            default:
                ledcWrite(MOTOR_2_1_CHANNEL, 0);
                ledcWrite(MOTOR_2_2_CHANNEL, 0);
                ledcWrite(MOTOR_1_1_CHANNEL, 0);
                ledcWrite(MOTOR_1_2_CHANNEL, 0);
        }
    }

    void processAxis(int8_t posX, int8_t posY) {
        tiltStop = !digitalRead(titlStopSignal);

        if (posY > 20) {
            if (tiltStop && lastTiltDirection != 1 ) {
                ledcWrite(MOTOR_2_2_CHANNEL, panTiltSpeedMax);
                lastTiltDirection = 2;
            } else if (!tiltStop) {
                ledcWrite(MOTOR_2_2_CHANNEL, panTiltSpeedMax);
                lastTiltDirection = 1;
            } else {
                ledcWrite(MOTOR_2_2_CHANNEL, 0);
                lastTiltDirection = 1;
            }
        } else {
            ledcWrite(MOTOR_2_2_CHANNEL, 0);
        }

        if (posY < -20) {
            if (tiltStop && lastTiltDirection != 2 ) {
                ledcWrite(MOTOR_2_1_CHANNEL, panTiltSpeedMax);
                lastTiltDirection = 1;
            } else if (!tiltStop) {
                ledcWrite(MOTOR_2_1_CHANNEL, panTiltSpeedMax);
                lastTiltDirection = 2;
            } else {
                ledcWrite(MOTOR_2_1_CHANNEL, 0);
                lastTiltDirection = 2;
            }
        } else {
            ledcWrite(MOTOR_2_1_CHANNEL, 0);
        }

        if (posX > 20) {
            ledcWrite(MOTOR_1_1_CHANNEL, panTiltSpeedMax);
        } else {
            ledcWrite(MOTOR_1_1_CHANNEL, 0);
        }

        if (posX < -20) {
            ledcWrite(MOTOR_1_2_CHANNEL, panTiltSpeedMax);
        } else {
            ledcWrite(MOTOR_1_2_CHANNEL, 0);
        }
    }   
}
