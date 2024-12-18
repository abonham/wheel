#include <Arduino.h>
#include "Adafruit_TinyUSB.h"

#define ENCODER_CLK 16
#define ENCODER_DT  17

#define HAT_LEFT 15
#define HAT_DOWN 14
#define HAT_RIGHT 13
#define HAT_UP 12

#define BTN_A 18
#define BTN_B 19
#define BTN_X 20
#define BTN_Y 21
#define BTN_LB 9
#define BTN_RB 8

#define L_X 11
#define L_Y 10

#define BTN_START 26 
#define BTN_BACK 27
#define BTN_GUIDE 7

#define ACC 28
#define BRAKE 27

#define DEBUG_PIN 3
#define RANGE_SELECT_PIN 4

#define DEBUG 1

// HID report descriptor using TinyUSB's template
// Single Report (no ID) descriptor
uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_GAMEPAD()
};

// USB HID object
Adafruit_USBD_HID usb_hid;

// Report payload defined in src/class/hid/hid.h
// - For Gamepad Button Bit Mask see  hid_gamepad_button_bm_t
// - For Gamepad Hat    Bit Mask see  hid_gamepad_hat_t
hid_gamepad_report_t gp;

int lastClk = HIGH;
int rotation = 0;
float rotationReport = 0.0f;
float range = 0;
int accMin = 512;
int accMax = 512;
int brakeMin = 512;
int brakeMax = 512;
pin_size_t buttons[] = {BTN_A, BTN_B, BTN_X, BTN_Y, BTN_LB, BTN_RB, L_X, L_Y, BTN_START, BTN_BACK, BTN_GUIDE};
pin_size_t hat[] = {HAT_UP, HAT_DOWN, HAT_LEFT, HAT_RIGHT};

void setup() {
  // Manual begin() is required on core without built-in support e.g. mbed rp2040
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);

  }

  Serial.begin(115200);
  
  // Setup encoder pins
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

  pinMode(L_X, INPUT_PULLUP);
  pinMode(L_Y, INPUT_PULLUP);

  pinMode(HAT_LEFT, INPUT_PULLUP);
  pinMode(HAT_RIGHT, INPUT_PULLUP);
  pinMode(HAT_UP, INPUT_PULLUP);
  pinMode(HAT_DOWN, INPUT_PULLUP);

  pinMode(ACC, INPUT_PULLUP);
  pinMode(BRAKE, INPUT_PULLUP);

  pinMode(2, INPUT_PULLUP);
  pinMode(DEBUG_PIN, INPUT_PULLUP);
  pinMode(RANGE_SELECT_PIN, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);
  
  // Setup HID
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
}

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

void normalizePotRange(int *storedMin, int *storedMax, int value) {
  *storedMin = min(*storedMin, value);
  *storedMax = max(*storedMax, value);
}

void checkJoystick() {
  int x = digitalRead(L_X);
  int y = digitalRead(L_Y);

  gp.rx = x == LOW ? 127 : -127;
  gp.ry = y == LOW ? 127 : -127;
}

void readButtons(uint32_t *out) {
  PinStatus r;
  for (int i = 0; i <= 10; i++) {
    r = digitalRead(buttons[i]);
    if (r == LOW) {
      *out |= (1UL << i);
    }
  }
}

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

void loop() {


  #ifdef TINYUSB_NEED_POLLING_TASK
  // Manual call tud_task since it isn't called by Core's background
  TinyUSBDevice.task();
  #endif

  // not enumerated()/mounted() yet: nothing to do
  if (!TinyUSBDevice.mounted()) {
    return;
  }

//  // Remote wakeup
//  if ( TinyUSBDevice.suspended() && btn )
//  {
//    // Wake up host if we are in suspend mode
//    // and REMOTE_WAKEUP feature is enabled by host
//    TinyUSBDevice.remoteWakeup();
//  }


  // Update rotary encoder - TODO: move to ISR
  checkRotation();

  range = digitalRead(RANGE_SELECT_PIN) == LOW ? 450.0f : 600.0f;
  range = 450.0f;
  
  rotationReport = (float)rotation * (127.0f / range);
  rotationReport = max(min(127, rotationReport), -127);
  
  uint32_t b = 0;
  readButtons(&b);
  uint8_t dpad = 0;
  readDPad(&dpad);

  if (!usb_hid.ready()) return;

  gp.buttons = b;
  gp.y = 0;
  gp.z = 0;
  gp.rz = 0;
  gp.ry = 0;
  gp.hat = dpad;

  // update state if ready for another HID poll
  gp.x = (int)rotationReport;
  
  // gp.buttons = buttonState;

  checkJoystick();
  
  PinStatus r;
  r = digitalRead(2);
  if (r == LOW) { rotation = 0; }
  

  // int acc = analogRead(ACC);
  // int brake = analogRead(BRAKE);
  // normalizePotRange(&accMin, &accMax, acc);
  // normalizePotRange(&brakeMin, &brakeMax, brake);
  // gp.rz = (acc / 4) - 127;
  // gp.ry = (brake / 4) - 127;
  
  int on = 127;
  int off = -127;
  gp.rz = digitalRead(ACC) == LOW ? on : off;
  gp.ry = digitalRead(BRAKE) == LOW ? on : off;

  usb_hid.sendReport(0, &gp, sizeof(gp));


  if (DEBUG) {
    Serial.print(gp.rz);
    Serial.print(",");
    Serial.print(gp.ry);
    Serial.print(","),
    Serial.print(gp.buttons);
    Serial.print(",");
    Serial.print(gp.hat);
    Serial.print(",");
    Serial.print(gp.x);
    Serial.print(",");
    Serial.println(rotation);
  }
}