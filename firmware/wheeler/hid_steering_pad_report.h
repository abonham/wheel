#pragma once

typedef struct TU_ATTR_PACKED
{
  int8_t  steering;
  int8_t  accelerator;
  int8_t  brake;
  int8_t  x;         ///< Delta x  movement of left analog-stick
  int8_t  y;         ///< Delta y  movement of left analog-stick
  int8_t  z;         ///< Delta z  movement of right analog-joystick
  int8_t  rx;        ///< Delta Rx movement of analog left trigger
  int8_t  ry;        ///< Delta Ry movement of analog right trigger
  int8_t  rz;        ///< Delta Rz movement of right analog-joystick
  uint16_t buttons;  ///< Buttons mask for currently pressed buttons
}hid_combind_controls_report_t;
/// HID Gamepad Protocol Report.
typedef struct TU_ATTR_PACKED
{
  int8_t  x;         ///< Delta x  movement of left analog-stick
  int8_t  y;         ///< Delta y  movement of left analog-stick
  int8_t  z;         ///< Delta z  movement of right analog-joystick
  int8_t  rx;        ///< Delta Rx movement of analog left trigger
  int8_t  ry;        ///< Delta Ry movement of analog right trigger
  int8_t  rz;        ///< Delta Rz movement of right analog-joystick
  uint8_t hat;
  uint16_t buttons;  ///< Buttons mask for currently pressed buttons
}hid_game_pad_report_t;

typedef struct TU_ATTR_PACKED
{
  int8_t  steering;
  int8_t  accelerator;
  int8_t  brake;
}hid_steering_pad_report_t;
