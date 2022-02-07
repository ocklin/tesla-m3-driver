#ifndef PinMode_PRJ_H_INCLUDED
#define PinMode_PRJ_H_INCLUDED

#include "device.h"

#define DIG_IO_LIST \
    DIG_IO_ENTRY(vtg_out,              0,                  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(launchpad_gate,     124,                  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(led_out,            DEVICE_GPIO_PIN_LED1, PinMode::OUTPUT)

#endif // PinMode_PRJ_H_INCLUDED
