#include <Arduino.h>
#include <SoftwareSerial.h>
#include <PS4Controller.h>
// Ps3.begin("A0:AB:51:5D:E9:1B");
#include "arduino-mti/handler.h"

int player = 0;

#define cmdPin 18
#define lancPin 19

int ledStatus = 4;

boolean ZOOM_IN_2[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, LOW, LOW};   // 28 04
boolean ZOOM_IN_3[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, LOW};  // 28 06
boolean ZOOM_IN_4[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, LOW, LOW, LOW};   // 28 08
boolean ZOOM_IN_6[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, LOW, LOW};  // 28 0C
boolean ZOOM_IN_7[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, HIGH, LOW}; // 28 0E

boolean ZOOM_OUT_2[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, LOW, HIGH, LOW, LOW};   // 28 14
boolean ZOOM_OUT_3[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, LOW, HIGH, HIGH, LOW};  // 28 16
boolean ZOOM_OUT_4[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, LOW, LOW, LOW};   // 28 18
boolean ZOOM_OUT_6[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, HIGH, LOW, LOW};  // 28 1C
boolean ZOOM_OUT_7[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, HIGH, HIGH, LOW}; // 28 1E


boolean POWER_OFF[] = {LOW,LOW,LOW,HIGH,HIGH,LOW,LOW,LOW,   LOW,HIGH,LOW,HIGH,HIGH,HIGH,HIGH,LOW}; //18 5E
boolean POWER_ON[] = {LOW,LOW,LOW,HIGH,HIGH,LOW,LOW,LOW,   LOW,HIGH,LOW,HIGH,HIGH,HIGH,LOW,LOW}; //18 5C  Doesn't work because there's no power supply from the LANC port when the camera is off
boolean POWER_OFF2[] = {LOW,LOW,LOW,HIGH,HIGH,LOW,LOW,LOW,   LOW,LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW}; //18 2A Turns the XF300 off and then on again
boolean POWER_SAVE[] = {LOW,LOW,LOW,HIGH,HIGH,LOW,LOW,LOW,   LOW,HIGH,HIGH,LOW,HIGH,HIGH,LOW,LOW}; //18 6C Didn't work

//Focus control. Camera must be switched to manual focus
boolean FOCUS_NEAR[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,HIGH,LOW,LOW,LOW,HIGH,HIGH,HIGH}; //28 47
boolean FOCUS_FAR[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,HIGH,LOW,LOW,LOW,HIGH,LOW,HIGH}; //28 45

boolean FOCUS_AUTO[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,HIGH,LOW,LOW,LOW,LOW,LOW,HIGH}; //28 41

int cmdRepeatCount;
int bitDuration = 104; // Duration of one LANC bit in microseconds.

void lancCommand(boolean lancBit[]) {

        cmdRepeatCount = 0;
        // Serial.println("lanc comand");
        while (cmdRepeatCount < 5) { // repeat 5 times to make sure the camera accepts the command

            while (pulseIn(lancPin, HIGH) < 5000) {
                //"pulseIn, HIGH" catches any 0V TO +5V TRANSITION and waits until the LANC line goes back to 0V
                //"pulseIn" also returns the pulse duration so we can check if the previous +5V duration was long enough (>5ms) to be the pause before a new 8 byte data packet
                // Loop till pulse duration is >5ms
            }
            // Serial.println(pulseIn(lancPin, HIGH));

            // if (pulseIn(lancPin, HIGH) < 6000) {
            //   return;
            // }
            // LOW after long pause means the START bit of Byte 0 is here
            delayMicroseconds(bitDuration); // wait START bit duration
      
            // Write the 8 bits of byte 0
            // Note that the command bits have to be put out in reverse order with the least significant, right-most bit (bit 0) first
            for (int i = 7; i > -1; i--) {
                digitalWrite(cmdPin, lancBit[i]); // Write bits.
                delayMicroseconds(bitDuration);
            }

            // Byte 0 is written now put LANC line back to +5V
            digitalWrite(cmdPin, LOW);
            delayMicroseconds(10); // make sure to be in the stop bit before byte 1

            while (digitalRead(lancPin)) {
                // Loop as long as the LANC line is +5V during the stop bit
            }

            // 0V after the previous stop bit means the START bit of Byte 1 is here
            delayMicroseconds(bitDuration); // wait START bit duration

            // Write the 8 bits of Byte 1
            // Note that the command bits have to be put out in reverse order with the least significant, right-most bit (bit 0) first
            for (int i = 15; i > 7; i--) {
                digitalWrite(cmdPin, lancBit[i]); // Write bits
                delayMicroseconds(bitDuration);
            }

            // Byte 1 is written now put LANC line back to +5V
            digitalWrite(cmdPin, LOW);

            cmdRepeatCount++; // increase repeat count by 1

            /*Control bytes 0 and 1 are written, now don’t care what happens in Bytes 2 to 7
            and just wait for the next start bit after a long pause to send the first two command bytes again.*/

        } // While cmdRepeatCount < 5
}

void notify()
{
  if (PS4.isConnected()) {
    if (PS4.Right()) {
      Serial.println("Right Button");
      digitalWrite(2, HIGH);
    } else {
      digitalWrite(2, LOW);
    }
  }
}


void setup()
{
    Serial.begin(9600);
    // pinMode(2, OUTPUT);
    // PS4.attach(notify);

    // digitalWrite(panStopSignal, HIGH);

    PS4.begin();

    pinMode(lancPin, INPUT_PULLUP);
    pinMode(cmdPin, OUTPUT); //writes to the LANC line
    digitalWrite(cmdPin, LOW);
    delay(5000);

    ArduinoMTI::init();
}

void indicateBatteryLevel() {
  if (PS4.Battery() >= 3 && ledStatus != 3) {
    PS4.setLed(0, 100, 0);
    PS4.sendToController();
    ledStatus = 3;
  } else if (PS4.Battery() == 2 && ledStatus != 2){
    PS4.setLed(100, 79, 2);
    PS4.sendToController();
    ledStatus = 2;
  } else if (PS4.Battery() < 2 && ledStatus > 1){
    PS4.setLed(100, 0, 0);
    PS4.sendToController();
    ledStatus = 1;
  }
}

void loop()
{
  if (PS4.PSButton() && PS4.Options()) {
    Serial.println("PS Button OFF");
    lancCommand(POWER_OFF);
    delay(200);
  } else if (PS4.PSButton()) {
    Serial.printf("Battery Level : %d\n", PS4.Battery());
    delay(200);
  }

  if (PS4.isConnected()) {
    indicateBatteryLevel();

    // Serial.printf("panStop: %d\n", panStop);

    if (PS4.Right()) {
      ArduinoMTI::processButton(ArduinoMTI::DIRECTIONS::RIGHT);
    } else if (PS4.Left()) {
      ArduinoMTI::processButton(ArduinoMTI::DIRECTIONS::LEFT);
    }

    if (PS4.Circle()) {
      lancCommand(FOCUS_FAR);
    }

    if (PS4.Cross()) {
      lancCommand(FOCUS_NEAR);
    }

    if (PS4.Triangle()) {
      lancCommand(FOCUS_AUTO);
    }

    ArduinoMTI::processAxis(PS4.LStickX(), PS4.LStickY());

    if (PS4.RStickY() > 30) {
      Serial.printf("Right Stick y at %d\n", PS4.RStickX());
      lancCommand(ZOOM_IN_7);
    }
    if (PS4.RStickY() < -30) {
      Serial.printf("Right Stick y at %d\n", PS4.RStickX());
      lancCommand(ZOOM_OUT_7);
    }
  }
}

// bool acceptTiltDirection(int direction) {
//   if (panStop && lastTiltDirection == direction) {
//     return false;
//   } else {
//     return true;
//   }
// }

// #define BOOT_CONFIG_PIN 1

// #define cmdPin 18
// #define lancPin 19
// #define motor1Pin1 22
// #define motor1Pin2 21
// #define motor2Pin1 17
// #define motor2Pin2 16
// #define titlStopSignal 26

// const int freq = 20000;
// const int MOTOR_1_1_CHANNEL = 0;
// const int MOTOR_1_2_CHANNEL = 1;
// const int MOTOR_2_1_CHANNEL = 2;
// const int MOTOR_2_2_CHANNEL = 3;

// const int resolution = 8;

// const int panSpeedMin = 140;
// const int tiltSpeedMin = 160;
// const int panTiltSpeedMax = 255;
// int panCurrentSpeed = panSpeedMin;
// int tiltCurrentSpeed = tiltSpeedMin;
// int lastTiltDirection = 0;
// int forbiddenDirection = 0;

// int cmdRepeatCount;
// int bitDuration = 104; // Duration of one LANC bit in microseconds.
// boolean panStop = false;

// struct JoystickData {
//   int direction;
//   int turbo;
//   int zoom;
// };

// uint32_t myTimer1, myTimer2;

// JoystickData joystickData;
// boolean ZOOM_IN_2[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, LOW, LOW};   // 28 04
// boolean ZOOM_IN_3[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, LOW};  // 28 06
// boolean ZOOM_IN_4[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, LOW, LOW, LOW};   // 28 08
// boolean ZOOM_IN_6[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, LOW, LOW};  // 28 0C
// boolean ZOOM_IN_7[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, HIGH, LOW}; // 28 0E

// boolean ZOOM_OUT_2[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, LOW, HIGH, LOW, LOW};   // 28 14
// boolean ZOOM_OUT_3[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, LOW, HIGH, HIGH, LOW};  // 28 16
// boolean ZOOM_OUT_4[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, LOW, LOW, LOW};   // 28 18
// boolean ZOOM_OUT_6[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, HIGH, LOW, LOW};  // 28 1C
// boolean ZOOM_OUT_7[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, HIGH, HIGH, LOW}; // 28 1E

// void lancCommand(boolean lancBit[]) {

//         cmdRepeatCount = 0;
//         // Serial.println("lanc comand");
//         while (cmdRepeatCount < 5) { // repeat 5 times to make sure the camera accepts the command

//             while (pulseIn(lancPin, HIGH) < 5000) {
//                 //"pulseIn, HIGH" catches any 0V TO +5V TRANSITION and waits until the LANC line goes back to 0V
//                 //"pulseIn" also returns the pulse duration so we can check if the previous +5V duration was long enough (>5ms) to be the pause before a new 8 byte data packet
//                 // Loop till pulse duration is >5ms
//             }
//             // Serial.println(pulseIn(lancPin, HIGH));

//             // if (pulseIn(lancPin, HIGH) < 6000) {
//             //   return;
//             // }
//             // LOW after long pause means the START bit of Byte 0 is here
//             delayMicroseconds(bitDuration); // wait START bit duration
      
//             // Write the 8 bits of byte 0
//             // Note that the command bits have to be put out in reverse order with the least significant, right-most bit (bit 0) first
//             for (int i = 7; i > -1; i--) {
//                 digitalWrite(cmdPin, lancBit[i]); // Write bits.
//                 delayMicroseconds(bitDuration);
//             }

//             // Byte 0 is written now put LANC line back to +5V
//             digitalWrite(cmdPin, LOW);
//             delayMicroseconds(10); // make sure to be in the stop bit before byte 1

//             while (digitalRead(lancPin)) {
//                 // Loop as long as the LANC line is +5V during the stop bit
//             }

//             // 0V after the previous stop bit means the START bit of Byte 1 is here
//             delayMicroseconds(bitDuration); // wait START bit duration

//             // Write the 8 bits of Byte 1
//             // Note that the command bits have to be put out in reverse order with the least significant, right-most bit (bit 0) first
//             for (int i = 15; i > 7; i--) {
//                 digitalWrite(cmdPin, lancBit[i]); // Write bits
//                 delayMicroseconds(bitDuration);
//             }

//             // Byte 1 is written now put LANC line back to +5V
//             digitalWrite(cmdPin, LOW);

//             cmdRepeatCount++; // increase repeat count by 1

//             /*Control bytes 0 and 1 are written, now don’t care what happens in Bytes 2 to 7
//             and just wait for the next start bit after a long pause to send the first two command bytes again.*/

//         } // While cmdRepeatCount < 5
// }

// void beginConfigurator();

// void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int length) {
//   if (length < 1) return;
//   memcpy(&joystickData, incomingData, length);
//   // Serial.print("Direction: ");
//   // Serial.print(joystickData.direction);
//   // Serial.print(", Zoom: ");
//   // Serial.println(joystickData.zoom);

//   // Serial.print(", Turbo: ");
//   // Serial.println(joystickData.turbo);
// }

// void setup() {
//     Serial.begin(9600);
//     Serial.print("ESP Board MAC Address:  ");
//     Serial.println(WiFi.macAddress());

//     WiFi.mode(WIFI_STA);
//     WiFi.disconnect();

//     // Init ESP-NOW
//     if (esp_now_init() != 0) {
//       Serial.println("Error initializing ESP-NOW");
//       return;
//     }

//     esp_now_register_recv_cb(OnDataRecv);
    
//     pinMode(titlStopSignal, INPUT_PULLUP);
//     // digitalWrite(panStopSignal, HIGH);

//     pinMode(lancPin, INPUT_PULLUP);
//     pinMode(cmdPin, OUTPUT); //writes to the LANC line
    
//     digitalWrite(cmdPin, LOW);
//     delay(5000);

//     ledcSetup(MOTOR_1_1_CHANNEL, freq, resolution);
//     ledcSetup(MOTOR_1_2_CHANNEL, freq, resolution);
//     ledcSetup(MOTOR_2_1_CHANNEL, freq, resolution);
//     ledcSetup(MOTOR_2_2_CHANNEL, freq, resolution);

//     ledcAttachPin(motor1Pin1, MOTOR_1_1_CHANNEL);
//     ledcAttachPin(motor1Pin2, MOTOR_1_2_CHANNEL);
//     ledcAttachPin(motor2Pin1, MOTOR_2_1_CHANNEL);
//     ledcAttachPin(motor2Pin2, MOTOR_2_2_CHANNEL);
// }

// bool acceptTiltDirection(int direction) {
//   if (panStop && lastTiltDirection == direction) {
//     return false;
//   } else {
//     return true;
//   }
// }

// void loop() {
//   panStop = !digitalRead(titlStopSignal);
//   tiltCurrentSpeed = joystickData.turbo ? panTiltSpeedMax : tiltSpeedMin;
//   panCurrentSpeed = joystickData.turbo ? panTiltSpeedMax : panSpeedMin;
//   if (joystickData.direction == 1 && acceptTiltDirection(1)) { // up
//     ledcWrite(MOTOR_2_2_CHANNEL, tiltCurrentSpeed);
//     lastTiltDirection = 1;
//   } else {
//     ledcWrite(MOTOR_2_2_CHANNEL, 0);
//   }
  
//   if (joystickData.direction == 3 && acceptTiltDirection(3)) { // bottom
//     ledcWrite(MOTOR_2_1_CHANNEL, tiltCurrentSpeed);
//     lastTiltDirection = 3;
//   } else {
//     ledcWrite(MOTOR_2_1_CHANNEL, 0);
//   }

//   if (joystickData.direction == 2) { // right
//     ledcWrite(MOTOR_1_1_CHANNEL, panCurrentSpeed);
//   } else if (joystickData.direction == 4) { // left
//     ledcWrite(MOTOR_1_2_CHANNEL, panCurrentSpeed);
//   }
  
//   if (joystickData.direction == 0) {
//     ledcWrite(MOTOR_1_1_CHANNEL, 0);
//     ledcWrite(MOTOR_1_2_CHANNEL, 0);
//     ledcWrite(MOTOR_2_1_CHANNEL, 0);
//     ledcWrite(MOTOR_2_2_CHANNEL, 0);
//   }
  
//   if (joystickData.zoom == 4 || joystickData.zoom == 5) {
//     if (joystickData.turbo) {
//       lancCommand(ZOOM_IN_6);
//     } else {
//       lancCommand(ZOOM_IN_2);
//     }
//   }

//   if (joystickData.zoom > 5) {
//     if (joystickData.turbo) {
//         lancCommand(ZOOM_IN_7);
//     } else {
//         lancCommand(ZOOM_IN_3);
//     }
//   }

//   if (joystickData.zoom == -3 || joystickData.zoom == -4) {
//     if (joystickData.turbo) {
//         lancCommand(ZOOM_OUT_6);
//     } else {
//         lancCommand(ZOOM_OUT_2);
//     }
//   }

//   if (joystickData.zoom < -4) {
//     if (joystickData.turbo) {
//         lancCommand(ZOOM_OUT_7);
//     } else {
//         lancCommand(ZOOM_OUT_3);
//     }
//   }
//   if (panStop) {
//     if (panCurrentSpeed == panSpeedMin) {
//       delay(2000);
//     } else {
//       delay(500);
//     }
//   }
// }