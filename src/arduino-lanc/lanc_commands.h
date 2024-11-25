#ifndef LANC_COMMANDS_H
#define LANC_COMMANDS_H

#include <Arduino.h>

extern boolean ZOOM_IN_2[];
extern boolean ZOOM_IN_3[];
extern boolean ZOOM_IN_4[];
extern boolean ZOOM_IN_6[];
extern boolean ZOOM_IN_7[];
extern boolean ZOOM_OUT_2[];
extern boolean ZOOM_OUT_3[];
extern boolean ZOOM_OUT_4[];
extern boolean ZOOM_OUT_6[];
extern boolean ZOOM_OUT_7[];
extern boolean SEARCHING[];
extern boolean POWER_OFF[];
extern boolean MENU[];
extern boolean MENU_RIGHT[];
extern boolean MENU_LEFT[];
extern boolean MENU_UP[];
extern boolean MENU_DOWN[];
extern boolean SET_ENTER[];
extern boolean WHITE_BALANCE_MENU[];
extern boolean EXPOSURE_MENU[];
extern boolean FOCUS_NEAR[];
extern boolean FOCUS_FAR[];
extern boolean FOCUS_AUTO[];

extern const int lancPin;
extern const int cmdPin;

void lancCommand(boolean lancBit[]);

#endif // LANC_COMMANDS_H