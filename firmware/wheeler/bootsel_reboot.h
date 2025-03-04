
extern "C" {
  #include "pico.h"
  #include "pico/time.h"
  #include "pico/bootrom.h"
}

#ifndef BOOTSEL_PIN
#include "pin_defines.h"
#endif

void checkReboot() {
  if (digitalRead(0) == LOW) {
    reset_usb_boot(1<<LED_BUILTIN,0); //invokes reset into bootloader mode
  }
}

#define USE_BOOTSEL_REBOOT_PIN 1
