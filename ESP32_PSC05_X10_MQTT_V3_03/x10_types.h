#pragma once
// Type definitions for the X10 IOT Gateway sketch.
// They live in their own tab because Arduino IDE inserts automatic function
// prototypes before the first function in the .ino; every type used in a
// function signature must already be declared at that point.
#include <Arduino.h>

enum DevType : uint8_t { T_DIMMER = 0, T_LIGHT = 1, T_SWITCH = 2, T_SENSOR = 3, T_MOMENTARY = 4, T_COUNT = 5 };

struct ModelInfo {
  const char *model;
  const char *manufacturer;
  const char *description;
  DevType type;              // default HA entity type for this model
  const char *device_class;  // binary_sensor device_class (SENSOR only) or nullptr
};

struct Device {
  String address, name;
  String model;  // "" = generic (registered without a model, e.g. V2.24)
  DevType type;
  uint8_t state; // 0 unknown, 1 off, 2 on
  int brightness; // -1 unknown (DIMMER only)
  bool publish_pending; // deferred MQTT state publish requested
  bool momentary_armed = false;   // MOMENTARY: ON shown, auto-OFF pending
  uint32_t momentary_since = 0;   // MOMENTARY: millis() when ON was seen
};

struct SeenDevice {
  String address;
  uint8_t state;        // 0 unknown, 1 off, 2 on
  String last_fn;       // last X10 function applied
  bool publish_pending;
  bool discovery_pending;
};

struct WorkItem {
  String command;
  String source;
  bool brightness;
  int level;
  uint32_t last_update_ms;
};

enum WorkPhase : uint8_t {
  WORK_IDLE, WORK_COMMAND, WORK_CAL_OFF, WORK_CAL_ON,
  WORK_CAL_ADJUST, WORK_ADJUST
};
