#include <Arduino.h>
#include "Adafruit_TinyUSB.h"
#include "quad_encoder.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "pin_defines.h"
#include "bootsel_reboot.h"

#define DEBUG 1

#define USB_POLL_INTERVAL_RATE_FAST 2
#define USB_POLL_INTERVAL_RATE_SLOW 20
#define USB_POLL_INTERVAL_RATE_VERY_SLOW 200

// HID report descriptor using TinyUSB's template
// Single Report (no ID) descriptor
uint8_t const desc_hid_report[] = {
  TUD_HID_REPORT_DESC_GAMEPAD()
};

#pragma region PIO_Setup
PIO pio = pio0;
const uint sm = 0;
int new_value, delta, old_value = 0;
int last_value = -1, last_delta = -1;
// CLK = A, B is next pin up from A
const uint PIN_AB = ENCODER_CLK;
#pragma endregion PIO - Setup

Adafruit_USBD_HID usb_hid;
hid_gamepad_report_t gp;

#pragma region state_variables
int lastClk = HIGH;
int rotation = 0;
float rotationReport = 0.0f;

int rotationMin, rotationMax = 0;

float range = 0;
int accMin = 512;
int accMax = 512;
int brakeMin = 512;
int brakeMax = 512;

pin_size_t buttons[] = { BTN_A, BTN_B, BTN_X, BTN_Y, BTN_LB, BTN_RB, ACC, BRAKE, BTN_START, BTN_BACK, BTN_GUIDE };
pin_size_t hat[] = { HAT_UP, HAT_DOWN, HAT_LEFT, HAT_RIGHT };
#pragma endregion state_variables

/// @brief Mostly just setting pin modes and starting HID device
void setup() {
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  Serial.begin(115200);  //super high to avoid slow down with USB polling

  // pinMode(ENCODER_CLK, INPUT_PULLUP);
  // pinMode(ENCODER_DT, INPUT_PULLUP);

  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_X, INPUT_PULLUP);
  pinMode(BTN_Y, INPUT_PULLUP);
  pinMode(BTN_LB, INPUT_PULLUP);
  pinMode(BTN_RB, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_BACK, INPUT_PULLUP);
  pinMode(BTN_GUIDE, INPUT_PULLUP);

  pinMode(L_X, INPUT);
  pinMode(L_Y, INPUT);

  pinMode(HAT_LEFT, INPUT_PULLUP);
  pinMode(HAT_RIGHT, INPUT_PULLUP);
  pinMode(HAT_UP, INPUT_PULLUP);
  pinMode(HAT_DOWN, INPUT_PULLUP);

  pinMode(ACC, INPUT_PULLUP);
  pinMode(BRAKE, INPUT_PULLUP);

  pinMode(ENCODER_RESET, INPUT_PULLUP);
  pinMode(DEBUG_PIN, INPUT_PULLUP);
  pinMode(RANGE_SELECT_PIN, INPUT_PULLUP);
  pinMode(RSTICK_SWAP, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(0, INPUT_PULLUP);

  digitalWrite(LED_BUILTIN, HIGH);


  uint8_t rate;
  #if DEBUG
  rate = USB_POLL_INTERVAL_RATE_VERY_SLOW;
  #else
  rate = USB_POLL_INTERVAL_RATE_FAST;
  #endif
  usb_hid.setPollInterval(rate);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();

  // If already enumerated, additional class driver begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  pio_add_program(pio, &quadrature_encoder_program);
  quadrature_encoder_program_init(pio, sm, PIN_AB, 0);
}

/// @brief Get rotation amount from PIO
void checkRotation() {
  new_value = quadrature_encoder_get_count(pio, sm);
  delta = new_value - old_value;
  old_value = new_value;
  if (rotationMin > new_value) {
    rotationMin = new_value;
  }
  if (rotationMax < new_value) {
    rotationMax = new_value;
  }

  if (new_value != last_value || delta != last_delta) {
    last_value = new_value;
    last_delta = delta;
  }

  int mapped = map(new_value, rotationMin, rotationMax, -127, 127);
  rotation = mapped;
}

/// @brief Track the minimum and maximum seen readings from potentiometer and use these as an updated baseline for comparison
/// @param storedMin
/// @param storedMax
/// @param value
void normalizePotRange(int *storedMin, int *storedMax, int value) {
  *storedMin = min(*storedMin, value);
  *storedMax = max(*storedMax, value);
}

/// @brief Check each of the buttons from defined buttons array and set as a 32-bit mask
/// @param out
void readButtons(uint32_t *out) {
  PinStatus r;
  for (int i = 0; i <= 10; i++) {
    r = digitalRead(buttons[i]);
    if (r == LOW) {
      *out |= (1UL << i);
    }
  }
}

/// @brief Map combination of button presses to HID enumeration for Button HATs
/// @param out
void readDPad(uint8_t *out) {
  bool up = digitalRead(HAT_UP) == LOW;
  bool down = digitalRead(HAT_DOWN) == LOW;
  bool left = digitalRead(HAT_LEFT) == LOW;
  bool right = digitalRead(HAT_RIGHT) == LOW;

  if (up) {
    if (right) {
      *out = GAMEPAD_HAT_UP_RIGHT;
    } else if (left) {
      *out = GAMEPAD_HAT_UP_LEFT;
    } else {
      *out = GAMEPAD_HAT_UP;
    }
    return;
  }

  if (down) {
    if (right) {
      *out = GAMEPAD_HAT_DOWN_RIGHT;
    } else if (left) {
      *out = GAMEPAD_HAT_DOWN_LEFT;
    } else {
      *out = GAMEPAD_HAT_DOWN;
    }
    return;
  }

  if (right) {
    *out = GAMEPAD_HAT_RIGHT;
  } else if (left) {
    *out = GAMEPAD_HAT_LEFT;
  } else {
    *out = GAMEPAD_HAT_CENTERED;
  }
}

/// @brief Main update loop
void loop() {
#if USE_BOOTSEL_REBOOT_PIN
  checkReboot();
#endif

  digitalWrite(LED_BUILTIN, LOW);
#ifdef TINYUSB_NEED_POLLING_TASK
  TinyUSBDevice.task();
#endif

  if (!TinyUSBDevice.mounted()) {
    return;
  }

  checkRotation();

  uint32_t b = 0;
  readButtons(&b);
  uint8_t dpad = 0;
  readDPad(&dpad);

  if (!usb_hid.ready()) return;
  digitalWrite(LED_BUILTIN, HIGH);

  gp.buttons = b;

  long x_a = analogRead(L_X);
  long mapped_x = map(x_a, 0, 1024, -127, 127);
  gp.y = mapped_x;

  long y_a = analogRead(L_Y);
  long mapped_y = map(y_a, 0, 1024, -127, 127);
  gp.z = mapped_y;
  
  gp.rz = 0;
  gp.ry = 0;
  gp.hat = dpad;

  gp.x = (int)rotation;

  PinStatus r;
  r = digitalRead(ENCODER_RESET);
  if (r == LOW) { rotation = 0; }

  int on = 127;
  int off = -127;
  gp.rx = digitalRead(ACC) == LOW ? on : off;
  gp.ry = digitalRead(BRAKE) == LOW ? on : off;

  usb_hid.sendReport(0, &gp, sizeof(gp));

  digitalWrite(LED_BUILTIN, LOW);

  //Monkey debug dump

  if (DEBUG) {
    Serial.print("rz: ");
    Serial.print(gp.rz);
    Serial.print(", ry: ");
    Serial.print(gp.ry);
    Serial.print(", buttons: "),
      Serial.print(gp.buttons);
    Serial.print(", hat: ");
    Serial.print(gp.hat);
    Serial.print(", x: ");
    Serial.print(gp.x);
    Serial.print(", rotation: ");
    Serial.println(rotation);
  }
}
