#include <Arduino.h>
#include <SoftwareSerial.h>
#include <PS4Controller.h>
// Ps3.begin("A0:AB:51:5D:E9:1B");
#include "arduino-mti/handler.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "arduino-lanc/lanc_commands.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

SemaphoreHandle_t ps4Semaphore;
int player = 0;

const int cmdPin = 18;
const int lancPin = 19;

int ledStatus = 4;

TaskHandle_t batteryTaskHandle;

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

void indicateBatteryLevelTask(void *parameter) {
  Serial.println("indicateBatteryLevelTask started");
  for (;;) {
    if (xSemaphoreTake(ps4Semaphore, portMAX_DELAY)) {
      if (PS4.Battery() >= 3 && ledStatus != 3) {
        PS4.setLed(0, 100, 0);
        PS4.sendToController();
        ledStatus = 3;
      } else if (PS4.Battery() == 2 && ledStatus != 2) {
        PS4.setLed(100, 79, 2);
        PS4.sendToController();
        ledStatus = 2;
      } else if (PS4.Battery() < 2 && ledStatus != 1) {
        PS4.setLed(100, 0, 0);
        PS4.sendToController();
        ledStatus = 1;
      }
      xSemaphoreGive(ps4Semaphore);
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

void setup()
{
    Serial.begin(9600);

    PS4.begin();

    ps4Semaphore = xSemaphoreCreateMutex();

    pinMode(lancPin, INPUT_PULLUP);
    pinMode(cmdPin, OUTPUT); //writes to the LANC line
    digitalWrite(cmdPin, LOW);
    delay(5000);

    Serial.print("ESP Board MAC Address:  ");
    printDeviceAddress();
    ArduinoMTI::init();

    xTaskCreatePinnedToCore(
      indicateBatteryLevelTask,
      "Battery Task",
      4096,
      NULL,
      1,
      &batteryTaskHandle,
      0
    );
}

void loop()
{
  if (PS4.isConnected()) {
    if (xSemaphoreTake(ps4Semaphore, portMAX_DELAY)) { // Захватываем мьютекс
      if (PS4.Options() && PS4.L1()) {
        lancCommand(WHITE_BALANCE_MENU);
      } else if (PS4.Options() && PS4.L2()) {
        lancCommand(EXPOSURE_MENU);
      } else if (PS4.Options()) {
        lancCommand(MENU);
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

      if (PS4.Right()) {
        if (PS4.L2()) {
          ArduinoMTI::processButton(ArduinoMTI::DIRECTIONS::RIGHT);
        } else {
          lancCommand(MENU_RIGHT);
        }
      } else if (PS4.Left()) {
        if (PS4.L2()) {
          ArduinoMTI::processButton(ArduinoMTI::DIRECTIONS::LEFT);
        } else {
          lancCommand(MENU_LEFT);
        }
      } else if (PS4.Up()) {
        if (PS4.L2()) {
          ArduinoMTI::processButton(ArduinoMTI::DIRECTIONS::UP);
        } else {
          lancCommand(MENU_UP);
        }
      } else if (PS4.Down()) {
        if (PS4.L2()) {
          ArduinoMTI::processButton(ArduinoMTI::DIRECTIONS::DOWN);
        } else {
          lancCommand(MENU_DOWN);
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
        if (PS4.L2()) {
          lancCommand(FOCUS_NEAR);
        } else {
          lancCommand(SET_ENTER);
        }
      }

      if (PS4.Triangle()) {
        lancCommand(FOCUS_AUTO);
      }
      int stickY = PS4.RStickY();

      if (stickY >= 12 && stickY <= 126) {
        Serial.println("ZOOM_IN_3");
        lancCommand(ZOOM_IN_3);
      } else if (stickY > 126){
        Serial.println("ZOOM_IN_7");
        lancCommand(ZOOM_IN_7);
      }
      if (stickY <= -12 && stickY >= -126 ) {
        Serial.println("ZOOM_OUT_3");
        lancCommand(ZOOM_OUT_3);
      } else if (stickY < -126){
        Serial.println("ZOOM_OUT_3");
        lancCommand(ZOOM_OUT_7);
      }

      xSemaphoreGive(ps4Semaphore); // Освобождаем мьютекс
    }
  }
}