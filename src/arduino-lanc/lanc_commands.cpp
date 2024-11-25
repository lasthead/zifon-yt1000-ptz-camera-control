#include "lanc_commands.h"

boolean ZOOM_IN_2[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, LOW, LOW};
boolean ZOOM_IN_3[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, LOW};
boolean ZOOM_IN_4[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, LOW, LOW, LOW};
boolean ZOOM_IN_6[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, LOW, LOW};
boolean ZOOM_IN_7[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, HIGH, LOW};
boolean ZOOM_OUT_2[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, LOW, HIGH, LOW, LOW};
boolean ZOOM_OUT_3[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, LOW, HIGH, HIGH, LOW};
boolean ZOOM_OUT_4[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, LOW, LOW, LOW};
boolean ZOOM_OUT_6[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, HIGH, LOW, LOW};
boolean ZOOM_OUT_7[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, HIGH, HIGH, LOW};
boolean SEARCHING[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, HIGH, LOW, HIGH, HIGH, LOW, HIGH, HIGH, HIGH};
boolean POWER_OFF[] = {LOW, LOW, LOW, HIGH, HIGH, LOW, LOW, LOW, LOW, HIGH, LOW, HIGH, HIGH, HIGH, HIGH, LOW};
boolean MENU[] = {LOW, LOW, LOW, HIGH, HIGH, LOW, LOW, LOW, HIGH, LOW, LOW, HIGH, HIGH, LOW, HIGH, LOW};
boolean MENU_RIGHT[] = {LOW, LOW, LOW, HIGH, HIGH, LOW, LOW, LOW, HIGH, HIGH, LOW, LOW, LOW, LOW, HIGH, LOW};
boolean MENU_LEFT[] = {LOW, LOW, LOW, HIGH, HIGH, LOW, LOW, LOW, HIGH, HIGH, LOW, LOW, LOW, HIGH, LOW, LOW};
boolean MENU_UP[] = {LOW, LOW, LOW, HIGH, HIGH, LOW, LOW, LOW, HIGH, LOW, LOW, LOW, LOW, HIGH, LOW, LOW};
boolean MENU_DOWN[] = {LOW, LOW, LOW, HIGH, HIGH, LOW, LOW, LOW, HIGH, LOW, LOW, LOW, LOW, HIGH, HIGH, LOW};
boolean SET_ENTER[] = {LOW, LOW, LOW, HIGH, HIGH, LOW, LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, HIGH, LOW};
boolean WHITE_BALANCE_MENU[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, HIGH, LOW, HIGH, HIGH, LOW, HIGH, LOW, HIGH};
boolean EXPOSURE_MENU[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, HIGH, LOW, HIGH, HIGH, LOW, HIGH, HIGH, HIGH};
boolean FOCUS_NEAR[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, HIGH, LOW, LOW, LOW, HIGH, HIGH, HIGH};
boolean FOCUS_FAR[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, HIGH, LOW, LOW, LOW, HIGH, LOW, HIGH};
boolean FOCUS_AUTO[] = {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, LOW, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, HIGH};

int cmdRepeatCount = 0;
int bitDuration = 104; // duration of one bit in microseconds

void lancCommand(boolean lancBit[]) {
        cmdRepeatCount = 0;

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
