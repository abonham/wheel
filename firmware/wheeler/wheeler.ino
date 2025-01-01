#include <Arduino.h>
#include "Adafruit_TinyUSB.h"


#define ENCODER_CLK 16 //WHITE
#define ENCODER_DT  17 //GREEN

#define HAT_LEFT 15
#define HAT_DOWN 14
#define HAT_RIGHT 13
#define HAT_UP 12

#define BTN_A 18
#define BTN_B 19
#define BTN_X 20
#define BTN_Y 21
#define BTN_LB 6
#define BTN_RB 7

#define BTN_START 8
#define BTN_BACK 9
#define BTN_GUIDE 10

#define BRAKE 22
#define ACC 28
#define L_X 27
#define L_Y 26

#define ENCODER_RESET 2
#define DEBUG_PIN 3
#define RANGE_SELECT_PIN 4

#define RSTICK_SWAP 5

#define DEBUG 1

// HID report descriptor using TinyUSB's template
// Single Report (no ID) descriptor
uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_GAMEPAD()
};

Adafruit_USBD_HID usb_hid;
hid_gamepad_report_t gp;

int lastClk = HIGH;
int rotation = 0;
float rotationReport = 0.0f;
float range = 0;
int accMin = 512;
int accMax = 512;
int brakeMin = 512;
int brakeMax = 512;

pin_size_t buttons[] = {BTN_A, BTN_B, BTN_X, BTN_Y, BTN_LB, BTN_RB, ACC, BRAKE, BTN_START, BTN_BACK, BTN_GUIDE};
pin_size_t hat[] = {HAT_UP, HAT_DOWN, HAT_LEFT, HAT_RIGHT};

/// @brief Verify the mid-point in a potentiometer analogue range, as compared to ADC_REF and GND_REF.
/// @note may well help in cases we don't have a linear pot.
void midPoint() {
  pinMode(L_Y, INPUT_PULLDOWN);
  delay(100);
  int low_y = analogRead(L_Y);
  pinMode(L_Y, INPUT_PULLUP);
  delay(100);
  int high_y = analogRead(L_Y);
  Serial.print("\n\nlow_y: ");
  Serial.print(low_y);
  Serial.print("\nhigh_y: ");
  Serial.print(high_y);
  Serial.println("\n");
  delay(1000);
}

/// @brief Mostly just setting pin modes and starting HID device
void setup() {
  // Manual begin() is required on core without built-in support e.g. mbed rp2040
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);

  }

  Serial.begin(250000);

  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);

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

  digitalWrite(LED_BUILTIN, HIGH);

  usb_hid.setPollInterval(16);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();

  // If already enumerated, additional class driver begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
}

/// @brief Brute force quadrature signal processing. Lazy but pretty effective for the amount of effort
void checkRotation() {
  int newClk = digitalRead(ENCODER_CLK);

  if (newClk != lastClk) {

    // There was a change on the CLK pin
    lastClk = newClk;

    int dtValue = digitalRead(ENCODER_DT);
    if (newClk == LOW && dtValue == HIGH) {
        rotation++;
    }
    if (newClk == LOW && dtValue == LOW) {
      rotation--;
    }
  }
  rotation = min(600, max(-600, rotation));
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


extern "C" {
  #include "pico.h"
  #include "pico/time.h"
  #include "pico/bootrom.h"
}

void checkReboot() {
  if (digitalRead(0) == LOW) {
    reset_usb_boot(1<<LED_BUILTIN,0); //invokes reset into bootloader mode
  }
}

/// @brief Main update loop
void loop() {
  checkReboot();
  digitalWrite(LED_BUILTIN, LOW);
  #ifdef TINYUSB_NEED_POLLING_TASK
  // Manual call tud_task since it isn't called by Core's background
  TinyUSBDevice.task();
  #endif

  // not enumerated()/mounted() yet: nothing to do
  if (!TinyUSBDevice.mounted()) {
    return;
  }

  // Update rotary encoder - TODO: move to ISR
  checkRotation();

  range = digitalRead(RANGE_SELECT_PIN) == LOW ? 450.0f : 600.0f;

  rotationReport = (float)rotation * (127.0f / range) * 1.1;
  rotationReport = max(min(127, rotationReport), -127);

  uint32_t b = 0;
  readButtons(&b);
  uint8_t dpad = 0;
  readDPad(&dpad);

  if (!usb_hid.ready()) return;
  digitalWrite(LED_BUILTIN, HIGH);

  gp.buttons = b;

  long x_a = analogRead(L_X);
  long mapped_x = map(x_a, 0, 1024, -127, 127);

  long y_a = analogRead(L_Y);
  long mapped_y = map(y_a, 0, 1024, -127, 127);
  gp.y = mapped_x;
  gp.z = mapped_y;
  gp.rz = 0;
  gp.ry = 0;
  gp.hat = dpad;

  gp.x = (int)rotationReport;

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
