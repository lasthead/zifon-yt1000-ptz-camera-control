#include <Arduino.h>
#include <SoftwareSerial.h>
#include <PS4Controller.h>
// Ps3.begin("A0:AB:51:5D:E9:1B");
#include "arduino-mti/handler.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

int player = 0;

#define cmdPin 18
#define lancPin 19

int ledStatus = 4;

boolean ZOOM_IN_2[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,LOW,LOW,HIGH,LOW,LOW};   // 28 04
boolean ZOOM_IN_3[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,LOW,LOW,HIGH,HIGH,LOW};  // 28 06
boolean ZOOM_IN_4[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,LOW,HIGH,LOW,LOW,LOW};   // 28 08
boolean ZOOM_IN_6[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,LOW,HIGH,HIGH,LOW,LOW};  // 28 0C
boolean ZOOM_IN_7[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,LOW,HIGH,HIGH,HIGH,LOW}; // 28 0E
boolean ZOOM_OUT_2[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,HIGH,LOW,HIGH,LOW,LOW};   // 28 14
boolean ZOOM_OUT_3[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,HIGH,LOW,HIGH,HIGH,LOW};  // 28 16
boolean ZOOM_OUT_4[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,HIGH,HIGH,LOW,LOW,LOW};   // 28 18
boolean ZOOM_OUT_6[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,HIGH,HIGH,HIGH,LOW,LOW};  // 28 1C
boolean ZOOM_OUT_7[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,LOW,LOW,HIGH,HIGH,HIGH,HIGH,LOW}; // 28 1E

boolean SEARCHING[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   HIGH,LOW,HIGH,HIGH,LOW,HIGH,HIGH,HIGH};

boolean POWER_OFF[] = {LOW,LOW,LOW,HIGH,HIGH,LOW,LOW,LOW,   LOW,HIGH,LOW,HIGH,HIGH,HIGH,HIGH,LOW}; //18 5E

boolean MENU[] = {LOW,LOW,LOW,HIGH,HIGH,LOW,LOW,LOW,   HIGH,LOW,LOW,HIGH,HIGH,LOW,HIGH,LOW};
boolean MENU_RIGHT[] = {LOW,LOW,LOW,HIGH,HIGH,LOW,LOW,LOW,   HIGH,HIGH,LOW,LOW,LOW,LOW,HIGH,LOW};
boolean MENU_LEFT[] = {LOW,LOW,LOW,HIGH,HIGH,LOW,LOW,LOW,   HIGH,HIGH,LOW,LOW,LOW,HIGH,LOW,LOW};

boolean MENU_UP[] = {LOW,LOW,LOW,HIGH,HIGH,LOW,LOW,LOW,   HIGH,LOW,LOW,LOW,LOW,HIGH,LOW,LOW};
boolean MENU_DOWN[] = {LOW,LOW,LOW,HIGH,HIGH,LOW,LOW,LOW,   HIGH,LOW,LOW,LOW,LOW,HIGH,HIGH,LOW};

boolean SET_ENTER[] = {LOW,LOW,LOW,HIGH,HIGH,LOW,LOW,LOW,   HIGH,LOW,HIGH,LOW,LOW,LOW,HIGH,LOW};

boolean WHITE_BALANCE_MENU[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   HIGH,LOW,HIGH,HIGH,LOW,HIGH,LOW,HIGH};
boolean EXPOSURE_MENU[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   HIGH,LOW,HIGH,HIGH,LOW,HIGH,HIGH,HIGH};

//Focus control. Camera must be switched to manual focus
boolean FOCUS_NEAR[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,HIGH,LOW,LOW,LOW,HIGH,HIGH,HIGH}; //28 47
boolean FOCUS_FAR[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,HIGH,LOW,LOW,LOW,HIGH,LOW,HIGH}; //28 45

boolean FOCUS_AUTO[] = {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW,LOW,   LOW,HIGH,LOW,LOW,LOW,LOW,LOW,HIGH}; //28 41

int cmdRepeatCount;
int bitDuration = 104; // Duration of one LANC bit in microseconds.

void printDeviceAddress() {

  const uint8_t* point = esp_bt_dev_get_address();

  for (int i = 0; i < 6; i++) {
    char str[3];
    sprintf(str, "%02X", (int)point[i]);
    Serial.print(str);

    if (i < 5) {
      Serial.print(":");
    }
  }
}



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

void setup()
{
    Serial.begin(9600);

    PS4.begin();

    pinMode(lancPin, INPUT_PULLUP);
    pinMode(cmdPin, OUTPUT); //writes to the LANC line
    digitalWrite(cmdPin, LOW);
    delay(5000);

    Serial.print("ESP Board MAC Address:  ");
    printDeviceAddress();

    ArduinoMTI::init();
}

void indicateBatteryLevel() {
  if (PS4.Battery() >= 3 && ledStatus != 3) {
    PS4.setLed(0, 100, 0);
    delay(200);
    PS4.sendToController();
    ledStatus = 3;
  } else if (PS4.Battery() == 2 && ledStatus != 2){
    PS4.setLed(100, 79, 2);
    delay(200);
    PS4.sendToController();
    ledStatus = 2;
  } else if (PS4.Battery() < 2 && ledStatus != 1){
    PS4.setLed(100, 0, 0);
    delay(200);
    PS4.sendToController();
    ledStatus = 1;
  }
}

boolean menuContext = false;
unsigned long lastButtonPressTime = 0;  // Время последнего нажатия кнопки
const unsigned long debounceDelay = 200; // Задержка для устранения дребезга (в миллисекундах)


void loop()
{
  if (PS4.isConnected()) {
    unsigned long currentTime = millis();
    //  Serial.println(menuContext);
    if (PS4.L1() && PS4.L2() && PS4.R1() && PS4.R2()) {
      if (currentTime - lastButtonPressTime > debounceDelay) {
        menuContext = !menuContext;
        lastButtonPressTime = currentTime;
      }
    }

    if (PS4.Options() && PS4.L1()) {
      if (currentTime - lastButtonPressTime > debounceDelay) {
        lancCommand(WHITE_BALANCE_MENU);
        menuContext = !menuContext;
        lastButtonPressTime = currentTime;
      }
    } else if (PS4.Options() && PS4.L2()) {
      if (currentTime - lastButtonPressTime > debounceDelay) {
        lancCommand(EXPOSURE_MENU);
        menuContext = !menuContext;
        lastButtonPressTime = currentTime;
      }
    } else if (PS4.Options()) {
      if (currentTime - lastButtonPressTime > debounceDelay) {
        lancCommand(MENU);
        menuContext = !menuContext;
        Serial.println(menuContext);
        lastButtonPressTime = currentTime;
      }
    }

    if (PS4.PSButton() && PS4.Options()) {
      Serial.println("PS Button OFF");
      lancCommand(POWER_OFF);
      delay(200);
      PS4.end();
      btStop();
    } else if (PS4.PSButton()) {
      Serial.printf("Battery Level : %d\n", PS4.Battery());
      Serial.printf("ledStatus : %d\n", ledStatus);
      delay(200);
    }

    indicateBatteryLevel();

    if (PS4.Right()) {
      if (menuContext) {
        lancCommand(MENU_RIGHT);
      } else {
        ArduinoMTI::processButton(ArduinoMTI::DIRECTIONS::RIGHT);
      }
    } else if (PS4.Left()) {
      if (menuContext) {
        Serial.println("MENU_LEFT");
        lancCommand(MENU_LEFT);
      } else {
        ArduinoMTI::processButton(ArduinoMTI::DIRECTIONS::LEFT);
      }
    } else if (PS4.Up()) {
      if (menuContext) {
        lancCommand(MENU_UP);
      } else {
        ArduinoMTI::processButton(ArduinoMTI::DIRECTIONS::UP);
      }
    } else if (PS4.Down()) {
      if (menuContext) {
        lancCommand(MENU_DOWN);
      } else {
        ArduinoMTI::processButton(ArduinoMTI::DIRECTIONS::DOWN);
      }
    } else if (PS4.UpLeft()) {
      ArduinoMTI::processButton(ArduinoMTI::DIRECTIONS::LEFT);
      ArduinoMTI::processButton(ArduinoMTI::DIRECTIONS::UP);
    } else if (PS4.UpRight()) {
      ArduinoMTI::processButton(ArduinoMTI::DIRECTIONS::RIGHT);
      ArduinoMTI::processButton(ArduinoMTI::DIRECTIONS::UP);
    } else if (PS4.DownLeft()) {
      ArduinoMTI::processButton(ArduinoMTI::DIRECTIONS::LEFT);
      ArduinoMTI::processButton(ArduinoMTI::DIRECTIONS::DOWN);
    } else if (PS4.DownRight()) {
      ArduinoMTI::processButton(ArduinoMTI::DIRECTIONS::RIGHT);
      ArduinoMTI::processButton(ArduinoMTI::DIRECTIONS::DOWN);
    } else {
      ArduinoMTI::processAxis(PS4.LStickX(), PS4.LStickY());
    }

    if (PS4.Circle()) {
      lancCommand(FOCUS_FAR);
    }

    if (PS4.Cross()) {
      if (menuContext) {
        lancCommand(SET_ENTER);
      } else {
        lancCommand(FOCUS_NEAR);
      }
    }

    if (PS4.Triangle()) {
      lancCommand(FOCUS_AUTO);
    }
    int stickY = PS4.RStickY();

    if (stickY >= 12 && stickY <= 126) {
      lancCommand(ZOOM_IN_3);
    } else if (stickY > 126){
      lancCommand(ZOOM_IN_7);
    }
    if (stickY <= -12 && stickY >= -126 ) {
      lancCommand(ZOOM_OUT_3);
    } else if (stickY < -126){
      lancCommand(ZOOM_OUT_7);
    }
  }
}