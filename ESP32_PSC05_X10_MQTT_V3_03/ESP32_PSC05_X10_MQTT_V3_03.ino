/*
 * X10 IOT Gateway - V3.03
 * ESP32 + X10 PSC05 (TW523-compatible) + MQTT + Home Assistant
 *
 * Target: Arduino IDE 1.8.19 / ESP32 Arduino core 1.0.6
 * Library dependency: PubSubClient by Nick O'Leary (2.8)
 * X10 engine: x10_psc05_v2_17_backoff_diagnostic_esp32_core_1_0_6.h (V2.17.1,
 *             hardware validated; UNCHANGED in this release).
 *
 * IMPORTANT: Edit x10_secrets.h (Wi-Fi + broker) before uploading.
 *
 * MQTT topics (all V2.24 topics are kept):
 *   x10/command          inbound: A1 ON, A1 OFF, C7 DIM 5, A ALL_OFF ...
 *   x10/event            outbound: QUEUED, TX_OK, RX, SEEN_NEW, ERROR ...
 *   x10/status           outbound engine statistics
 *   x10/availability     retained: online/offline (LWT)
 *   x10/manage/set       inbound registry management (see below)
 *   x10/manage/result    outbound management result (also an HA sensor)
 *   x10/device/<A1>/...  registered device state / brightness
 *   x10/seen/<A1>/...    unregistered (seen) device state / attributes
 *   x10/register/...     Home Assistant registration panel entities
 *
 * Management (x10/manage/set):
 *   ADD|C3|Living Room Lamp|LM465            model sets the HA entity type
 *   ADD|C3|Living Room Lamp|HD501|DIMMER     model + explicit type override
 *   ADD|C3|Living Room Lamp|DIMMER           legacy V2.24 form (no model)
 *   RENAME|C3|New Name
 *   REMOVE|C3
 *   FORGET|C3                                drop an unregistered "seen" entry
 *   LIST
 *   Types: DIMMER (HA light + brightness), LIGHT (HA light, on/off),
 *          SWITCH (HA switch), SENSOR (HA binary_sensor, receive only),
 *          MOMENTARY (HA switch that returns to OFF ~2.5 s after ON, for
 *          modules such as the UM506 in momentary mode).
 *
 * V3.03:
 *   1. Deleting a registered device in Home Assistant (MQTT integration ->
 *      device -> Delete) also removes it from the gateway's registry, so it
 *      can be registered again. HA clears the device's retained discovery
 *      config when it deletes it; the gateway watches each registered
 *      device's config topic for that empty message.
 *
 * V3.02:
 *   1. HA registration panel gains "House Code" (A-P) and "Unit Code" (1-16)
 *      selects, so ANY address can be registered from Home Assistant without
 *      another controller or a computer. Choosing a Seen Address fills them in.
 *
 * V3.01:
 *   1. New MOMENTARY type: HA switch that shows ON for MOMENTARY_RESET_MS
 *      after an ON (from HA or another controller), then reverts to OFF
 *      without sending anything on the power line.
 *   2. Model table in alphabetical order; HA model select shows
 *      "MODEL - Name" labels (plain model numbers are still accepted).
 *
 * V3.00 (over V2.24):
 *   1. Device registry stores an X10 model (HD466, HD465, AMT-103, 7246,
 *      AMT-101, PMS02, WS12A, WS467, HD501, SR227, UM506, LM15A-C, TM751,
 *      TM523, HD11A, LM465). The model selects the HA entity type, which can
 *      be overridden. Registry raised to 32 devices.
 *   2. X10 traffic from other controllers that addresses an UNREGISTERED
 *      unit creates a "seen" entry (persisted, max 32) that is published to
 *      HA as a binary_sensor tracking its ON/OFF state.
 *   3. HA registration panel (MQTT discovery, on the controller device):
 *      seen-address select, model select, type select, name text,
 *      Register / Remove / Forget buttons and a result sensor.
 *      Registering a seen address removes its sensor, creates the
 *      light/switch/binary_sensor and carries the last known state over.
 *   4. Republish discovery when Home Assistant restarts (homeassistant/status).
 *   5. SENSOR type devices only follow explicitly addressed ON/OFF; ALL_*
 *      broadcasts are applied to lamps/switches only, as real modules do.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <string>
#include "x10_secrets.h"
#include "x10_types.h"
#include "x10_psc05_v2_17_backoff_diagnostic_esp32_core_1_0_6.h"

static const char *FW_VERSION = "3.03";

static const char *TOPIC_COMMAND = "x10/command";
static const char *TOPIC_EVENT = "x10/event";
static const char *TOPIC_STATUS = "x10/status";
static const char *TOPIC_AVAILABILITY = "x10/availability";
static const char *TOPIC_MANAGE = "x10/manage/set";
static const char *TOPIC_MANAGE_RESULT = "x10/manage/result";
static const char *TOPIC_HA_STATUS = "homeassistant/status";
static const char *TOPIC_REGISTER_PREFIX = "x10/register/";

// ---------------------------------------------------------------------------
// Device types and supported models
// ---------------------------------------------------------------------------
static const char *const TYPE_NAMES[T_COUNT] = {"DIMMER", "LIGHT", "SWITCH", "SENSOR", "MOMENTARY"};
// Used for the HA device name of registrations without a model (V2.24 style).
static const char *const TYPE_LABELS[T_COUNT] = {"Lamp", "Light", "Switch", "Sensor", "Momentary"};
// MOMENTARY devices (e.g. UM506 in momentary mode, contacts close ~1.5-2.5 s)
// show ON this long after an ON, then revert to OFF.
static const uint32_t MOMENTARY_RESET_MS = 2500;


// Default types are best-effort; verify against your hardware. The HA
// registration panel "Type" select (or ADD|..|MODEL|TYPE) overrides them.
// Alphabetical by model number. "name" is shown next to the model number in
// the HA model select ("LM465 - Lamp Module"). Names for LM15A, LM465, SR227,
// TM751, UM506, WS12A and WS467 are from thex10shop.com's manual index;
// HD465/HD466 are confirmed IBM modules; the rest are descriptive only.
static const ModelInfo MODELS[] = {
  {"7246",    "X10 compatible",    "Appliance Module",            T_SWITCH, nullptr},
  {"AMT-101", "X10 compatible",    "Appliance Module",            T_SWITCH, nullptr},
  {"AMT-103", "X10 compatible",    "Appliance Module",            T_SWITCH, nullptr},
  {"HD11A",   "IBM Home Director", "Appliance Module",            T_SWITCH, nullptr},
  {"HD465",   "IBM",               "Lamp Module",                 T_DIMMER, nullptr},
  {"HD466",   "IBM",               "3-Pin Appliance Module",      T_SWITCH, nullptr},
  {"HD501",   "IBM Home Director", "Wall Switch",                 T_DIMMER, nullptr},
  {"LM15A-C", "X10",               "Socket Rocket Lamp Module",   T_LIGHT,  nullptr},
  {"LM465",   "X10",               "Lamp Module",                 T_DIMMER, nullptr},
  {"PMS02",   "X10 compatible",    "Motion Sensor",               T_SENSOR, "motion"},
  {"SR227",   "X10",               "SuperSocket Outlet",          T_SWITCH, nullptr},
  {"TM523",   "X10 compatible",    "Transceiver Module",          T_SWITCH, nullptr},
  {"TM751",   "X10",               "Transceiver Module",          T_SWITCH, nullptr},
  {"UM506",   "X10",               "Universal Module",            T_SWITCH, nullptr},
  {"WS12A",   "X10",               "Decorator Dimmer Switch",     T_DIMMER, nullptr},
  {"WS467",   "X10",               "Wall Switch Module",          T_DIMMER, nullptr},
};
static const uint8_t MODEL_COUNT = sizeof(MODELS) / sizeof(MODELS[0]);

// Accepts "LM465" or a select label "LM465 - Lamp Module".
static int find_model(String m) {
  m.trim();
  int sp = m.indexOf(' ');
  if (sp > 0) m = m.substring(0, sp);
  m.toUpperCase();
  for (uint8_t i = 0; i < MODEL_COUNT; ++i)
    if (m == MODELS[i].model) return i;
  return -1;
}
static String model_label(uint8_t i) {
  return String(MODELS[i].model) + " - " + MODELS[i].description;
}
static int parse_type(String s) {
  s.trim(); s.toUpperCase();
  for (uint8_t i = 0; i < T_COUNT; ++i)
    if (s == TYPE_NAMES[i]) return i;
  return -1;
}

// ---------------------------------------------------------------------------
// Registry of registered devices (NVS namespace "x10-reg")
// ---------------------------------------------------------------------------
static const uint8_t MAX_DEVICES = 32;
static Device devices[MAX_DEVICES];
static uint8_t device_count = 0;

static bool is_dimmer(const Device &d) { return d.type == T_DIMMER; }
static bool is_lamp(const Device &d) { return d.type == T_DIMMER || d.type == T_LIGHT; }
// MOMENTARY: an ON (re)starts the auto-OFF timer.
static void note_momentary_on(Device &d) {
  if (d.type != T_MOMENTARY || d.state != 2) return;
  d.momentary_armed = true;
  d.momentary_since = millis();
}

// ---------------------------------------------------------------------------
// Unregistered addresses seen on the power line (NVS namespace "x10-seen")
// ---------------------------------------------------------------------------
static const uint8_t MAX_SEEN = 32;
static SeenDevice seen[MAX_SEEN];
static uint8_t seen_count = 0;
static bool seen_save_pending = false;
static bool seen_full_reported = false;

// ---------------------------------------------------------------------------
// HA registration panel state
// ---------------------------------------------------------------------------
static const char *PANEL_NONE = "(none)";
static const char *MODEL_NONE = "(choose)";
static const char *TYPE_DEFAULT = "Model default";
static String reg_address = PANEL_NONE;   // seen-address shortcut
static String reg_house = MODEL_NONE;     // "(choose)" or A..P
static String reg_unit = MODEL_NONE;      // "(choose)" or 1..16
static String reg_model = MODEL_NONE;
static String reg_type = TYPE_DEFAULT;
static String reg_name = "";
static String rm_target = PANEL_NONE;
static bool panel_refresh_pending = false;  // option lists changed

static Preferences prefs;
static WiFiClient wifi_client;
static PubSubClient mqtt(wifi_client);
static String serial_line;
static uint32_t last_wifi_attempt_ms = 0;
static uint32_t last_mqtt_attempt_ms = 0;
static uint32_t last_stats_ms = 0;
static bool discovery_pending = false;
static uint16_t discovery_index = 0;
static uint32_t brightness_debounce_ms = 500;

// Application-level FIFO: one X10 engine transaction is submitted at a time.
// A brightness request occupies one FIFO slot for its entire calibration.
static const uint8_t WORK_QUEUE_SIZE = 16;
static WorkItem work_queue[WORK_QUEUE_SIZE];
static uint8_t work_head = 0, work_tail = 0, work_count = 0;

static WorkPhase work_phase = WORK_IDLE;
static WorkItem current_work;
static String expected_command;
static int work_start_level = -1;
static int work_steps = 0;
static bool work_brighter = false;
static bool work_waiting_result = false;

static const uint32_t WORK_TIMEOUT_MS = 60000UL;   // watchdog on one engine command
static uint32_t work_deadline_ms = 0;
static bool work_retry_seen = false;      // engine retried/collided during this command
static bool work_disturbed = false;       // external X10 traffic touched the lamp being adjusted
static const uint8_t MAX_ABANDONED = 4;
static String abandoned_cmds[MAX_ABANDONED];  // commands whose terminal event we still expect
static uint8_t abandoned_count = 0;
static uint16_t rx_unit_mask[16];         // per-house addressed units (X10 address set)
static bool rx_after_function[16];        // next address starts a new set
static const uint8_t MANAGE_PENDING_MAX = 4;
static String manage_pending[MANAGE_PENDING_MAX];
static uint8_t manage_pending_count = 0;
static int pending_debounce_ms = -1;

// True while an X10 transaction (or calibration sequence) owns the line.
static bool tx_active() { return work_phase != WORK_IDLE || work_waiting_result; }
// True when registry changes must wait (also while commands are queued).
static bool manage_busy() { return tx_active() || work_count != 0; }

static bool valid_address(String addr) {
  addr.trim(); addr.toUpperCase();
  if (addr.length() < 2 || addr.length() > 3) return false;
  if (addr[0] < 'A' || addr[0] > 'P') return false;
  for (unsigned int i=1;i<addr.length();++i)
    if (addr[i] < '0' || addr[i] > '9') return false;
  int unit = addr.substring(1).toInt();
  return unit >= 1 && unit <= 16 && addr.substring(1) == String(unit);
}
static String make_address(uint8_t house, uint8_t unit) {
  return String((char)('A' + house)) + String(unit);
}
static int find_device(const String &addr) {
  String normalized=addr;
  normalized.trim();
  normalized.toUpperCase();
  for (uint8_t i=0;i<device_count;++i) {
    if (devices[i].address==normalized) return i;
  }
  return -1;
}
static int find_seen(const String &addr) {
  String normalized=addr;
  normalized.trim();
  normalized.toUpperCase();
  for (uint8_t i=0;i<seen_count;++i) {
    if (seen[i].address==normalized) return i;
  }
  return -1;
}

static String state_topic(const Device &d) { return "x10/device/"+d.address+"/state"; }
static String bright_set_topic(const Device &d) { return "x10/device/"+d.address+"/brightness/set"; }
static String bright_state_topic(const Device &d) { return "x10/device/"+d.address+"/brightness/state"; }
static const char *ha_component(DevType t) {
  switch (t) {
    case T_DIMMER: case T_LIGHT: return "light";
    case T_SWITCH: case T_MOMENTARY: return "switch";
    default: return "binary_sensor";
  }
}
static String discovery_topic(const Device &d) {
  return String("homeassistant/") + ha_component(d.type) + "/esp32_psc05_" + d.address + "/config";
}
// Watch a registered device's own discovery topic: an empty retained message
// there means Home Assistant deleted the device.
static void watch_discovery(const Device &d, bool on) {
  if (!mqtt.connected()) return;
  String t=discovery_topic(d);
  if (on) mqtt.subscribe(t.c_str()); else mqtt.unsubscribe(t.c_str());
}
static String unique_id(const Device &d) {
  return String("esp32_psc05_x10_") + d.address;
}
static String seen_state_topic(const String &a) { return "x10/seen/"+a+"/state"; }
static String seen_attr_topic(const String &a) { return "x10/seen/"+a+"/attributes"; }
static String seen_discovery_topic(const String &a) {
  return "homeassistant/binary_sensor/esp32_psc05_seen_"+a+"/config";
}

// ---------------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------------
static void save_registry() {
  prefs.begin("x10-reg",false);
  prefs.putUChar("count",device_count);
  for (uint8_t i=0;i<device_count;++i) {
    String k=String(i);
    prefs.putString(("a"+k).c_str(),devices[i].address);
    prefs.putString(("n"+k).c_str(),devices[i].name);
    prefs.putBool(("d"+k).c_str(),is_dimmer(devices[i]));   // V2.24 compatibility
    prefs.putString(("m"+k).c_str(),devices[i].model);
    prefs.putUChar(("t"+k).c_str(),(uint8_t)devices[i].type);
  }
  prefs.end();
}
static void load_registry() {
  prefs.begin("x10-reg",true);
  bool initialized=prefs.getBool("init",false);
  if (initialized) {
    uint8_t n=prefs.getUChar("count",0);
    for (uint8_t i=0;i<n && device_count<MAX_DEVICES;++i) {
      String k=String(i);
      String a=prefs.getString(("a"+k).c_str(),"");
      if (!valid_address(a) || find_device(a)>=0) continue;
      Device &d=devices[device_count++];
      d.address=a;d.name=prefs.getString(("n"+k).c_str(),a);
      uint8_t t=prefs.getUChar(("t"+k).c_str(),0xFF);
      if (t>=T_COUNT) {
        // Registered by V2.24: only the dimmer flag exists.
        t=prefs.getBool(("d"+k).c_str(),false)?T_DIMMER:T_SWITCH;
        d.model="";
      } else {
        d.model=prefs.getString(("m"+k).c_str(),"");
        if (d.model.length() && find_model(d.model)<0) d.model="";
      }
      d.type=(DevType)t;
    }
  }
  prefs.end();
  if (!initialized) {
    devices[0].address="A1";devices[0].name="X10 A1";devices[0].type=T_SWITCH;devices[0].model="";
    devices[1].address="B1";devices[1].name="X10 B1";devices[1].type=T_DIMMER;devices[1].model="";
    device_count=2;
    save_registry();
    prefs.begin("x10-reg",false);prefs.putBool("init",true);prefs.end();
  }
  for (uint8_t i=0;i<device_count;++i) {
    devices[i].state=0;
    devices[i].brightness=-1;
    devices[i].publish_pending=false;
    devices[i].momentary_armed=false;
  }
}
static void save_seen() {
  prefs.begin("x10-seen",false);
  prefs.putUChar("count",seen_count);
  for (uint8_t i=0;i<seen_count;++i)
    prefs.putString(("s"+String(i)).c_str(),seen[i].address);
  prefs.end();
  seen_save_pending=false;
}
static void load_seen() {
  prefs.begin("x10-seen",true);
  uint8_t n=prefs.getUChar("count",0);
  for (uint8_t i=0;i<n && seen_count<MAX_SEEN;++i) {
    String a=prefs.getString(("s"+String(i)).c_str(),"");
    if (!valid_address(a) || find_device(a)>=0 || find_seen(a)>=0) continue;
    SeenDevice &s=seen[seen_count++];
    s.address=a; s.state=0; s.last_fn="";
    s.publish_pending=false; s.discovery_pending=false;  // full discovery covers it
  }
  prefs.end();
}

static void result(const String &s) {
  Serial.println("[MANAGE] "+s);
  if (mqtt.connected()) mqtt.publish(TOPIC_MANAGE_RESULT,s.c_str(),false);
}
static void event(const String &s) {
  Serial.println("[EVENT] "+s);
  if (mqtt.connected()) mqtt.publish(TOPIC_EVENT,s.c_str(),false);
}

// Escape MQTT discovery JSON strings (names may contain quotes/backslashes).
static String json_escape(const String &input) {
  String out;
  for (unsigned int i = 0; i < input.length(); ++i) {
    char c = input[i];
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else if ((uint8_t)c < 0x20) out += ' ';
    else out += c;
  }
  return out;
}

// ---------------------------------------------------------------------------
// Registered device state + discovery
// ---------------------------------------------------------------------------
static void publish_state(Device &d) {
  if (!mqtt.connected() || !d.state) return;
  // MQTT light (default schema) state must match payload_on/off, e.g. "B1 ON".
  String on_off=d.state==2?"ON":"OFF";
  String state=is_lamp(d) ? d.address+" "+on_off : on_off;
  String topic=state_topic(d);
  mqtt.publish(topic.c_str(),state.c_str(),true);
  if (is_dimmer(d)) {
    topic=bright_state_topic(d);
    if (d.brightness>=0) {
      String b=String(d.brightness);
      mqtt.publish(topic.c_str(),b.c_str(),true);
    } else {
      // Remove any stale retained brightness estimate.
      mqtt.publish(topic.c_str(),"",true);
    }
  }
}

static void publish_unknown(Device &d) {
  if (!mqtt.connected()) return;
  String t = state_topic(d);
  mqtt.publish(t.c_str(), "", true);
  if (is_dimmer(d)) {
    t = bright_state_topic(d);
    mqtt.publish(t.c_str(), "", true);
  }
}

static String controller_device_json() {
  return String("\"device\":{\"identifiers\":[\"esp32_psc05_controller\"],"
                "\"name\":\"ESP32 PSC05 X10 Controller\","
                "\"manufacturer\":\"ESP32 / X10\","
                "\"model\":\"PSC05 Interface\",\"sw_version\":\"") + FW_VERSION + "\"}";
}

static String device_json(const Device &d) {
  int mi=find_model(d.model);
  String label = mi>=0 ? String(MODELS[mi].model) : String(TYPE_LABELS[d.type]);
  String model = mi>=0 ? String(MODELS[mi].model)+" "+MODELS[mi].description : String("Powerline Module");
  String manufacturer = mi>=0 ? String(MODELS[mi].manufacturer) : String("X10");
  // Each X10 address gets a separate Home Assistant device registry entry.
  // Stable identifiers preserve the association across renames and reboots.
  return "\"device\":{\"identifiers\":[\"esp32_psc05_x10_" + d.address +
    "\"],\"name\":\"" + json_escape("X10 " + label + " " + d.address) +
    "\",\"manufacturer\":\"" + json_escape(manufacturer) +
    "\",\"model\":\"" + json_escape(model) +
    "\",\"via_device\":\"esp32_psc05_controller\"}";
}

static bool publish_discovery_device(Device &d) {
  if (!mqtt.connected()) return false;
  String a=d.address;
  String payload="{\"name\":\""+json_escape(d.name)+"\",\"unique_id\":\""+unique_id(d)+
    "\",\"state_topic\":\""+state_topic(d)+"\",";
  if (is_lamp(d)) {
    payload+="\"command_topic\":\"x10/command\",\"payload_on\":\""+a+
      " ON\",\"payload_off\":\""+a+" OFF\",";
    if (is_dimmer(d)) {
      payload+="\"brightness_command_topic\":\""+bright_set_topic(d)+
        "\",\"brightness_state_topic\":\""+bright_state_topic(d)+
        "\",\"brightness_scale\":255,";
    }
  } else if (d.type==T_SWITCH || d.type==T_MOMENTARY) {
    payload+="\"command_topic\":\"x10/command\",\"payload_on\":\""+a+
      " ON\",\"payload_off\":\""+a+" OFF\",\"state_on\":\"ON\",\"state_off\":\"OFF\",";
  } else {
    // SENSOR: receive-only, follows ON/OFF transmitted to its address.
    payload+="\"payload_on\":\"ON\",\"payload_off\":\"OFF\",";
    int mi=find_model(d.model);
    if (mi>=0 && MODELS[mi].device_class)
      payload+=String("\"device_class\":\"")+MODELS[mi].device_class+"\",";
  }
  payload += "\"availability_topic\":\"x10/availability\",";
  payload += device_json(d) + "}";
  String topic=discovery_topic(d);
  bool ok=mqtt.publish(topic.c_str(),payload.c_str(),true);
  if (ok) {
    if (d.state==0) {
      // Device state is unknown: clear any previously retained MQTT state.
      String stateTopic=state_topic(d);
      if (!mqtt.publish(stateTopic.c_str(),"",true)) return false;
      if (is_dimmer(d)) {
        String brightnessTopic=bright_state_topic(d);
        if (!mqtt.publish(brightnessTopic.c_str(),"",true)) return false;
      }
    } else {
      publish_state(d);
    }
  }
  return ok;
}

// ---------------------------------------------------------------------------
// Seen (unregistered) device state + discovery
// ---------------------------------------------------------------------------
static void publish_seen_state(SeenDevice &s) {
  if (!mqtt.connected()) return;
  if (s.state) {
    String t=seen_state_topic(s.address);
    mqtt.publish(t.c_str(), s.state==2?"ON":"OFF", true);
  }
  String attr="{\"address\":\""+s.address+"\",\"house\":\""+String(s.address[0])+
    "\",\"unit\":"+s.address.substring(1)+",\"last_function\":\""+json_escape(s.last_fn)+
    "\",\"registered\":false}";
  String t=seen_attr_topic(s.address);
  mqtt.publish(t.c_str(), attr.c_str(), true);
}

static bool publish_seen_discovery(SeenDevice &s) {
  if (!mqtt.connected()) return false;
  const String &a=s.address;
  String payload="{\"name\":\"X10 Unregistered "+a+"\",\"unique_id\":\"esp32_psc05_seen_"+a+
    "\",\"state_topic\":\""+seen_state_topic(a)+
    "\",\"json_attributes_topic\":\""+seen_attr_topic(a)+
    "\",\"payload_on\":\"ON\",\"payload_off\":\"OFF\",\"icon\":\"mdi:help-network\""
    ",\"availability_topic\":\""+TOPIC_AVAILABILITY+"\","+controller_device_json()+"}";
  String topic=seen_discovery_topic(a);
  bool ok=mqtt.publish(topic.c_str(),payload.c_str(),true);
  if (ok) {
    s.discovery_pending=false;
    // After reboot the state is unknown locally; the broker keeps the last
    // retained ON/OFF, which is the best information available.
    if (s.state || s.last_fn.length()) publish_seen_state(s);
  }
  return ok;
}

static void clear_seen_topics(const String &a) {
  if (!mqtt.connected()) return;
  String t=seen_discovery_topic(a); mqtt.publish(t.c_str(),"",true);
  t=seen_state_topic(a); mqtt.publish(t.c_str(),"",true);
  t=seen_attr_topic(a); mqtt.publish(t.c_str(),"",true);
}

static void remove_seen_at(uint8_t i) {
  if (i>=seen_count) return;
  String a=seen[i].address;
  clear_seen_topics(a);
  for (uint8_t j=i;j+1<seen_count;++j) seen[j]=seen[j+1];
  --seen_count;
  if (reg_address==a) reg_address=PANEL_NONE;
  seen_save_pending=true;
  panel_refresh_pending=true;
}

// Find or (for external traffic) create a seen entry for an unregistered unit.
static SeenDevice *note_seen(const String &addr, bool create) {
  int i=find_seen(addr);
  if (i>=0) return &seen[i];
  if (!create) return nullptr;
  if (seen_count>=MAX_SEEN) {
    if (!seen_full_reported) { result("ERROR SEEN_LIST_FULL " + addr); seen_full_reported=true; }
    return nullptr;
  }
  SeenDevice &s=seen[seen_count++];
  s.address=addr; s.state=0; s.last_fn="";
  s.publish_pending=false; s.discovery_pending=true;
  seen_save_pending=true;
  panel_refresh_pending=true;
  event("SEEN_NEW "+addr);
  return &s;
}

// ---------------------------------------------------------------------------
// HA registration panel (select/text/button/sensor on the controller device)
// ---------------------------------------------------------------------------
static String seen_options_json() {
  String s=String("[\"")+PANEL_NONE+"\"";
  for (uint8_t i=0;i<seen_count;++i) s+=",\""+seen[i].address+"\"";
  return s+"]";
}
static String remove_option(const Device &d) { return d.address+" - "+d.name; }
static String registered_options_json() {
  String s=String("[\"")+PANEL_NONE+"\"";
  for (uint8_t i=0;i<device_count;++i) s+=",\""+json_escape(remove_option(devices[i]))+"\"";
  return s+"]";
}
static String model_options_json() {
  String s=String("[\"")+MODEL_NONE+"\"";
  for (uint8_t i=0;i<MODEL_COUNT;++i) s+=",\""+json_escape(model_label(i))+"\"";
  return s+"]";
}
static String type_options_json() {
  String s=String("[\"")+TYPE_DEFAULT+"\"";
  for (uint8_t i=0;i<T_COUNT;++i) s+=String(",\"")+TYPE_NAMES[i]+"\"";
  return s+"]";
}
static String panel_common(const char *name, const char *uid, const char *category) {
  return String("\"name\":\"")+name+"\",\"unique_id\":\""+uid+
    "\",\"availability_topic\":\""+TOPIC_AVAILABILITY+
    "\",\"entity_category\":\""+category+"\","+controller_device_json();
}

static bool publish_debounce_discovery() {
  if (!mqtt.connected()) return false;
  const char *topic =
    "homeassistant/number/esp32_psc05_brightness_debounce/config";
  String payload = "{";
  payload += "\"name\":\"X10 Brightness Debounce\",";
  payload += "\"unique_id\":\"esp32_psc05_brightness_debounce\",";
  payload += "\"command_topic\":\"x10/settings/brightness_debounce/set\",";
  payload += "\"state_topic\":\"x10/settings/brightness_debounce/state\",";
  payload += "\"availability_topic\":\"";
  payload += TOPIC_AVAILABILITY;
  payload += "\",";
  payload += "\"min\":500,";
  payload += "\"max\":2000,";
  payload += "\"step\":100,";
  payload += "\"unit_of_measurement\":\"ms\",";
  payload += "\"mode\":\"slider\",";
  payload += "\"icon\":\"mdi:timer-cog\",";
  payload += controller_device_json();
  payload += "}";
  return mqtt.publish(topic,payload.c_str(),true);
}

// Address entered in the panel (House Code + Unit Code), or "" if incomplete.
static String panel_address() {
  if (reg_house==MODEL_NONE || reg_unit==MODEL_NONE) return "";
  return reg_house+reg_unit;
}
static String house_options_json() {
  String s=String("[\"")+MODEL_NONE+"\"";
  for (char h='A';h<='P';++h) s+=String(",\"")+h+"\"";
  return s+"]";
}
static String unit_options_json() {
  String s=String("[\"")+MODEL_NONE+"\"";
  for (uint8_t u=1;u<=16;++u) s+=",\""+String(u)+"\"";
  return s+"]";
}

// Make sure select states are always one of the published options.
static void sanitize_panel() {
  if (reg_address!=PANEL_NONE && find_seen(reg_address)<0) reg_address=PANEL_NONE;
  if (rm_target!=PANEL_NONE) {
    bool found=false;
    for (uint8_t i=0;i<device_count && !found;++i) found=(remove_option(devices[i])==rm_target);
    if (!found) rm_target=PANEL_NONE;
  }
}

static void publish_panel_state(uint8_t item) {
  if (!mqtt.connected()) return;
  sanitize_panel();
  switch (item) {
    case 0: mqtt.publish("x10/register/address/state",reg_address.c_str(),true); break;
    case 1: mqtt.publish("x10/register/model/state",reg_model.c_str(),true); break;
    case 2: mqtt.publish("x10/register/type/state",reg_type.c_str(),true); break;
    case 3: mqtt.publish("x10/register/name/state",reg_name.c_str(),false); break;
    case 5: mqtt.publish("x10/register/remove_target/state",rm_target.c_str(),true); break;
    case 10: mqtt.publish("x10/register/house/state",reg_house.c_str(),true); break;
    case 11: mqtt.publish("x10/register/unit/state",reg_unit.c_str(),true); break;
    default: break;
  }
}

static const uint8_t PANEL_ITEMS = 12;
static bool publish_panel_item(uint8_t item) {
  if (!mqtt.connected()) return false;
  String topic, payload;
  switch (item) {
    case 0:
      topic="homeassistant/select/esp32_psc05_reg_address/config";
      payload="{"+panel_common("X10 Register: Seen Address","esp32_psc05_reg_address","config")+
        ",\"command_topic\":\"x10/register/address/set\",\"state_topic\":\"x10/register/address/state\""
        ",\"options\":"+seen_options_json()+",\"icon\":\"mdi:map-marker-question\"}";
      break;
    case 1:
      topic="homeassistant/select/esp32_psc05_reg_model/config";
      payload="{"+panel_common("X10 Register: Model","esp32_psc05_reg_model","config")+
        ",\"command_topic\":\"x10/register/model/set\",\"state_topic\":\"x10/register/model/state\""
        ",\"options\":"+model_options_json()+",\"icon\":\"mdi:package-variant\"}";
      break;
    case 2:
      topic="homeassistant/select/esp32_psc05_reg_type/config";
      payload="{"+panel_common("X10 Register: Type","esp32_psc05_reg_type","config")+
        ",\"command_topic\":\"x10/register/type/set\",\"state_topic\":\"x10/register/type/state\""
        ",\"options\":"+type_options_json()+",\"icon\":\"mdi:shape\"}";
      break;
    case 3:
      topic="homeassistant/text/esp32_psc05_reg_name/config";
      payload="{"+panel_common("X10 Register: Name","esp32_psc05_reg_name","config")+
        ",\"command_topic\":\"x10/register/name/set\",\"state_topic\":\"x10/register/name/state\""
        ",\"min\":0,\"max\":40,\"mode\":\"text\",\"icon\":\"mdi:rename\"}";
      break;
    case 4:
      topic="homeassistant/button/esp32_psc05_reg_submit/config";
      payload="{"+panel_common("X10 Register Device","esp32_psc05_reg_submit","config")+
        ",\"command_topic\":\"x10/register/submit/press\",\"payload_press\":\"PRESS\",\"icon\":\"mdi:plus-network\"}";
      break;
    case 5:
      topic="homeassistant/select/esp32_psc05_reg_remove_target/config";
      payload="{"+panel_common("X10 Remove: Registered Device","esp32_psc05_reg_remove_target","config")+
        ",\"command_topic\":\"x10/register/remove_target/set\",\"state_topic\":\"x10/register/remove_target/state\""
        ",\"options\":"+registered_options_json()+",\"icon\":\"mdi:format-list-bulleted\"}";
      break;
    case 6:
      topic="homeassistant/button/esp32_psc05_reg_remove/config";
      payload="{"+panel_common("X10 Remove Device","esp32_psc05_reg_remove","config")+
        ",\"command_topic\":\"x10/register/remove/press\",\"payload_press\":\"PRESS\",\"icon\":\"mdi:minus-network\"}";
      break;
    case 7:
      topic="homeassistant/button/esp32_psc05_reg_forget/config";
      payload="{"+panel_common("X10 Forget Seen Address","esp32_psc05_reg_forget","config")+
        ",\"command_topic\":\"x10/register/forget/press\",\"payload_press\":\"PRESS\",\"icon\":\"mdi:delete-sweep\"}";
      break;
    case 8:
      topic="homeassistant/sensor/esp32_psc05_manage_result/config";
      payload="{"+panel_common("X10 Registration Result","esp32_psc05_manage_result","diagnostic")+
        ",\"state_topic\":\""+TOPIC_MANAGE_RESULT+"\",\"icon\":\"mdi:message-text\"}";
      break;
    case 9:
      return publish_debounce_discovery();
    case 10:
      topic="homeassistant/select/esp32_psc05_reg_house/config";
      payload="{"+panel_common("X10 Register: House Code","esp32_psc05_reg_house","config")+
        ",\"command_topic\":\"x10/register/house/set\",\"state_topic\":\"x10/register/house/state\""
        ",\"options\":"+house_options_json()+",\"icon\":\"mdi:home-alphabet\"}";
      break;
    case 11:
      topic="homeassistant/select/esp32_psc05_reg_unit/config";
      payload="{"+panel_common("X10 Register: Unit Code","esp32_psc05_reg_unit","config")+
        ",\"command_topic\":\"x10/register/unit/set\",\"state_topic\":\"x10/register/unit/state\""
        ",\"options\":"+unit_options_json()+",\"icon\":\"mdi:numeric\"}";
      break;
    default:
      return true;
  }
  bool ok=mqtt.publish(topic.c_str(),payload.c_str(),true);
  if (ok) publish_panel_state(item);
  return ok;
}

static void request_full_discovery() {
  discovery_index=0;
  discovery_pending=true;
}

// ---------------------------------------------------------------------------
// Deferred publishing
// ---------------------------------------------------------------------------
// Publish at most one deferred state per loop pass, outside any engine callback.
static void service_deferred_publish() {
  if (!mqtt.connected()) return;
  for (uint8_t i = 0; i < device_count; ++i) {
    if (!devices[i].publish_pending) continue;
    devices[i].publish_pending = false;
    if (devices[i].state == 0) publish_unknown(devices[i]);
    else publish_state(devices[i]);
    return;
  }
  for (uint8_t i = 0; i < seen_count; ++i) {
    if (!seen[i].publish_pending || seen[i].discovery_pending) continue;
    seen[i].publish_pending = false;
    publish_seen_state(seen[i]);
    return;
  }
}

static bool work_targets(const Device &d) {
  if (work_phase == WORK_IDLE) return false;
  return current_work.command == d.address ||
         current_work.command.startsWith(d.address + " ");
}

// Unregistered units: addressed ON/OFF/DIM/BRIGHT from another controller
// creates/updates a seen entry; ALL_UNITS_OFF turns known seen units off.
// ALL_LIGHTS_* are not applied: the module type of a seen unit is unknown.
static void apply_function_unregistered(uint8_t house, const String &fn,
                                        uint16_t unit_mask, bool external) {
  if (fn == "ALL_OFF") {
    for (uint8_t i = 0; i < seen_count; ++i) {
      if ((uint8_t)(seen[i].address[0] - 'A') != house) continue;
      seen[i].state = 1; seen[i].last_fn = fn; seen[i].publish_pending = true;
    }
    return;
  }
  if (!(fn == "ON" || fn == "OFF" || fn == "DIM" || fn == "BRIGHT")) return;
  for (uint8_t u = 1; u <= 16; ++u) {
    if (!(unit_mask & (uint16_t)(1u << (u - 1)))) continue;
    String addr = make_address(house, u);
    if (find_device(addr) >= 0) continue;
    SeenDevice *s = note_seen(addr, external);
    if (!s) continue;
    // DIM/BRIGHT switch an X10 lamp module on.
    s->state = (fn == "OFF") ? 1 : 2;
    s->last_fn = fn;
    s->publish_pending = true;
  }
}

// Apply an X10 function (own broadcast or observed external traffic) to the
// estimated state of registered and seen devices. fn: ON OFF DIM BRIGHT
// ALL_OFF ALL_ON ALL_LIGHTS_OFF.
static void apply_function(uint8_t house, const String &fn,
                           uint16_t unit_mask, bool external) {
  const bool all_off = (fn == "ALL_OFF");
  const bool lights  = (fn == "ALL_ON" || fn == "ALL_LIGHTS_OFF");
  const bool single  = (fn == "ON" || fn == "OFF" || fn == "DIM" || fn == "BRIGHT");
  if (!all_off && !lights && !single) return;
  for (uint8_t i = 0; i < device_count; ++i) {
    Device &d = devices[i];
    if ((uint8_t)(d.address[0] - 'A') != house) continue;
    int u = d.address.substring(1).toInt();
    bool addressed = single && (u >= 1 && u <= 16) &&
                     (unit_mask & (uint16_t)(1u << (u - 1)));
    bool hit;
    if (d.type == T_SENSOR) hit = addressed && (fn == "ON" || fn == "OFF");
    else if (all_off) hit = true;
    else if (lights) hit = is_lamp(d);
    else hit = addressed;
    if (!hit) continue;
    const bool off = (fn == "OFF" || all_off || fn == "ALL_LIGHTS_OFF");
    const bool on  = (fn == "ON" || fn == "ALL_ON");
    if (off) {
      d.state = 1; d.brightness = 0;
    } else if (on) {
      d.state = 2;
      if (d.brightness == 0) d.brightness = -1;
      note_momentary_on(d);
    } else if (is_dimmer(d) && d.state == 2 && d.brightness >= 0) {
      d.brightness = -1;           // DIM/BRIGHT seen: level now uncertain
    } else {
      continue;
    }
    if (external && work_targets(d)) work_disturbed = true;
    d.publish_pending = true;
  }
  apply_function_unregistered(house, fn, unit_mask, external);
}

// Mirror how real modules see the line: our own successful command leaves
// the addressed unit as the current address set for that house.
static void note_own_tx(const String &address) {
  if (address.length() < 2) return;
  int h = address[0] - 'A';
  int u = address.substring(1).toInt();
  if (h < 0 || h > 15 || u < 1 || u > 16) return;
  rx_unit_mask[h] = (uint16_t)(1u << (u - 1));
  rx_after_function[h] = true;
}

// Engine events for traffic from other X10 controllers look like:
//   "RX B1"        (address)      "RX B ON" / "RX B DIM" / "RX B ALL_OFF" ...
static void track_external_rx(const std::string &message) {
  if (message.compare(0, 3, "RX ") != 0) return;
  String t(message.c_str() + 3);
  t.trim(); t.toUpperCase();
  if (t.length() < 2) return;
  char hc = t[0];
  if (hc < 'A' || hc > 'P') return;
  uint8_t h = (uint8_t)(hc - 'A');
  if (t[1] >= '0' && t[1] <= '9') {
    int unit = t.substring(1).toInt();
    if (unit < 1 || unit > 16) return;
    if (rx_after_function[h]) { rx_unit_mask[h] = 0; rx_after_function[h] = false; }
    rx_unit_mask[h] |= (uint16_t)(1u << (unit - 1));
    return;
  }
  if (t[1] != ' ') return;
  String fn = t.substring(2);
  fn.trim();
  if (fn == "ADDRESS" || fn == "FUNCTION") return;
  apply_function(h, fn, rx_unit_mask[h], true);
  rx_after_function[h] = true;
}

static void note_engine_event(const std::string &message) {
  if (!work_waiting_result) return;
  if (message.compare(0, 6, "RETRY ") == 0 ||
      message.compare(0, 10, "COLLISION ") == 0 ||
      message.compare(0, 15, "TX_VERIFY_FAIL ") == 0)
    work_retry_seen = true;
}

// Mark one device (cmd "B1 ON") or a whole house (cmd "A ALL_OFF") unknown.
static void mark_unknown(const String &cmd) {
  int sp = cmd.indexOf(' ');
  String a = sp > 0 ? cmd.substring(0, sp) : cmd;
  for (uint8_t i = 0; i < device_count; ++i) {
    Device &d = devices[i];
    bool hit = (a.length() == 1) ? (d.address[0] == a[0]) : (d.address == a);
    if (!hit) continue;
    d.state = 0; d.brightness = -1; d.publish_pending = true;
  }
}

static void remember_abandoned(const String &cmd) {
  if (abandoned_count >= MAX_ABANDONED) {
    for (uint8_t i = 1; i < MAX_ABANDONED; ++i) abandoned_cmds[i - 1] = abandoned_cmds[i];
    --abandoned_count;
  }
  abandoned_cmds[abandoned_count++] = cmd;
}

// The engine is FIFO, so the first terminal event matching an abandoned
// command belongs to it and must not complete the current work item.
static bool consume_abandoned(const String &completed) {
  if (abandoned_count == 0 || completed != abandoned_cmds[0]) return false;
  for (uint8_t i = 1; i < abandoned_count; ++i) abandoned_cmds[i - 1] = abandoned_cmds[i];
  --abandoned_count;
  return true;
}

// Only a completed TX_OK or TX_FAIL releases an engine command.
// TX_OK means PSC05 RX-loopback verification, not physical module action.
static void publish_confirmed_state(const std::string &event_text) {
  bool ok = event_text.compare(0, 6, "TX_OK ") == 0;
  bool fail = event_text.compare(0, 8, "TX_FAIL ") == 0;
  if (!ok && !fail) return;

  String completed(event_text.c_str() + (ok ? 6 : 8));
  completed.trim(); completed.toUpperCase();
  // Late terminal event of a command abandoned by the watchdog.
  if (consume_abandoned(completed)) return;
  if (!work_waiting_result) return;
  if (completed != expected_command) return;
  work_waiting_result = false;

  String address = current_work.brightness ? current_work.command : "";
  if (!current_work.brightness) {
    int sp = current_work.command.indexOf(' ');
    if (sp > 0) address = current_work.command.substring(0, sp);
  }
  int idx = find_device(address);
  Device *d = idx >= 0 ? &devices[idx] : nullptr;
  if (ok && address.length() >= 2) note_own_tx(address);

  if (work_phase == WORK_COMMAND) {
    if (address.length() == 1) {
      // House broadcast (ALL_OFF / ALL_ON / ALL_LIGHTS_OFF).
      if (ok) {
        String fn = current_work.command.substring(address.length());
        fn.trim();
        apply_function((uint8_t)(address[0] - 'A'), fn, 0, false);
        rx_after_function[address[0] - 'A'] = true;
      }
      work_phase = WORK_IDLE;
      return;
    }
    String op = current_work.command.substring(address.length());
    op.trim(); op.toUpperCase();
    if (ok && d) {
      if (op == "ON" || op == "OFF") {
        d->state = (op == "ON") ? 2 : 1;
        if (op == "OFF") d->brightness = 0;
        else if (d->brightness == 0) d->brightness = -1;
        note_momentary_on(*d);
        publish_state(*d);
      } else if (is_dimmer(*d) &&
                 (op.startsWith("DIM ") || op.startsWith("BRIGHT "))) {
        // Raw DIM/BRIGHT has no reliable starting reference.
        d->brightness = -1;
        publish_state(*d);
      }
    } else if (ok && !d && (op == "ON" || op == "OFF")) {
      // Raw command to an unregistered address that is already being tracked.
      SeenDevice *s = note_seen(address, false);
      if (s) { s->state = (op == "ON") ? 2 : 1; s->last_fn = op; s->publish_pending = true; }
    }
    work_phase = WORK_IDLE;
    return;
  }

  if (!d) { work_phase = WORK_IDLE; return; }
  if (work_phase == WORK_CAL_OFF) {
    if (!ok) { work_phase = WORK_IDLE; return; }
    d->state = 1;
    d->brightness = 0;
    publish_state(*d);
    work_phase = WORK_CAL_ON;
    return;
  }
  if (work_phase == WORK_CAL_ON) {
    if (!ok) { work_phase = WORK_IDLE; return; }
    d->state = 2;
    d->brightness = 255; // Full-brightness reference after verified ON following OFF.
    publish_state(*d);
    if (current_work.level == 255) work_phase = WORK_IDLE;
    else work_phase = WORK_CAL_ADJUST;
    return;
  }
  if (work_phase == WORK_CAL_ADJUST || work_phase == WORK_ADJUST) {
    if (ok && (work_retry_seen || work_disturbed)) {
      // A retried DIM/BRIGHT may have been applied more than once,
      // and external traffic may have changed the lamp mid-sequence.
      d->brightness = -1;
      if (!work_disturbed) d->state = 2;
    } else if (ok) {
      int estimate = work_start_level +
                     (work_brighter ? work_steps * 16 : -work_steps * 16);
      d->brightness = constrain(estimate, 0, 255);
      // DIM to zero is NOT an explicit OFF.
      d->state = 2;
    } else {
      // A failed verification leaves brightness uncertain, not OFF.
      d->brightness = -1;
    }
    publish_state(*d);
    work_phase = WORK_IDLE;
  }
}

static bool enqueue_work(const WorkItem &item) {
  if (work_count >= WORK_QUEUE_SIZE) {
    result("ERROR APPLICATION_QUEUE_FULL");
    return false;
  }
  work_queue[work_tail] = item;
  work_tail = (work_tail + 1) % WORK_QUEUE_SIZE;
  ++work_count;
  return true;
}

// Normalize command strings to the engine's TX_OK/TX_FAIL event format.
// Reject malformed requests before accepting them into the application FIFO.
static void submit_command(const String &incoming, const char *source) {
  String line = incoming;
  line.trim(); line.toUpperCase();
  int first = line.indexOf(' ');
  if (first <= 0) { result("ERROR BAD_COMMAND"); return; }
  String addr = line.substring(0, first);
  String rest = line.substring(first + 1);
  rest.trim();
  int second = rest.indexOf(' ');
  String op = second < 0 ? rest : rest.substring(0, second);
  String arg = second < 0 ? "" : rest.substring(second + 1);
  arg.trim();
  String canonical;
  if (addr.length() == 1 && addr[0] >= 'A' && addr[0] <= 'P') {
    if (arg.length()) { result("ERROR BAD_COMMAND"); return; }
    if (op == "ALL_UNITS_OFF" || op == "ALL_OFF") op = "ALL_OFF";
    else if (op == "ALL_LIGHTS_ON" || op == "ALL_ON") op = "ALL_ON";
    else if (op != "ALL_LIGHTS_OFF") {
      result("ERROR BAD_COMMAND"); return;
    }
    canonical = addr + " " + op;
  } else if (valid_address(addr)) {
    if (op == "ON" || op == "OFF") {
      if (arg.length()) { result("ERROR BAD_COMMAND"); return; }
      canonical = addr + " " + op;
    } else if (op == "DIM" || op == "BRIGHT") {
      if (!arg.length()) arg = "1";
      for (unsigned int i = 0; i < arg.length(); ++i) {
        if (arg[i] < '0' || arg[i] > '9') {
          result("ERROR BAD_REPEAT"); return;
        }
      }
      int repeats = arg.toInt();
      if (repeats < 1 || repeats > 16) {
        result("ERROR BAD_REPEAT"); return;
      }
      canonical = addr + " " + op + " " + String(repeats);
    } else { result("ERROR BAD_COMMAND"); return; }
  } else { result("ERROR BAD_ADDRESS"); return; }

  WorkItem item;
  item.command = canonical;
  item.source = String(source);
  item.brightness = false;
  item.level = -1;
  item.last_update_ms = millis();
  enqueue_work(item);
}

static void submit_brightness(const String &address, int level) {
  // Coalesce only adjacent, still-queued slider events for the SAME lamp.
  // Never move a request across another user's command.
  if (work_count > 0) {
    uint8_t last = (work_tail + WORK_QUEUE_SIZE - 1) % WORK_QUEUE_SIZE;
    WorkItem &tail = work_queue[last];
    if (tail.brightness && tail.command == address) {
      tail.level = level;
      tail.last_update_ms = millis();
      return;
    }
  }
  WorkItem item;
  item.command = address;
  item.source = "HA";
  item.brightness = true;
  item.level = level;
  item.last_update_ms = millis();
  enqueue_work(item);
}

static bool issue_work_command(const String &command) {
  work_retry_seen = false;
  work_disturbed = false;
  work_deadline_ms = millis() + WORK_TIMEOUT_MS;
  expected_command = command;
  expected_command.trim(); expected_command.toUpperCase();
  bool accepted = x10_psc05.command(std::string(expected_command.c_str()));
  if (!accepted) {
    result("ERROR X10_COMMAND_REJECTED " + expected_command);
    work_waiting_result = false;
    work_phase = WORK_IDLE;
    return false;
  }
  work_waiting_result = true;
  return true;
}

// Watchdog: give up on an engine command that never reported TX_OK/TX_FAIL.
// Affected devices become "unknown"; a late terminal event for the command
// is swallowed by consume_abandoned().
static void abandon_current_work() {
  result("ERROR WORK_TIMEOUT " + expected_command);
  remember_abandoned(expected_command);
  mark_unknown(expected_command);
  work_waiting_result = false;
  work_phase = WORK_IDLE;
}

static void service_work_queue(uint32_t now) {
  if (work_waiting_result) {
    if ((int32_t)(now - work_deadline_ms) >= 0) abandon_current_work();
    return;
  }
  if (work_phase == WORK_CAL_ON) {
    issue_work_command(current_work.command + " ON");
    return;
  }
  if (work_phase == WORK_CAL_ADJUST || work_phase == WORK_ADJUST) {
    int idx = find_device(current_work.command);
    if (idx < 0) { work_phase = WORK_IDLE; return; }
    Device &d = devices[idx];
    // External traffic changed the lamp since calibration; redo it.
    if (d.state != 2 || d.brightness < 0) {
      work_phase = WORK_CAL_OFF;
      issue_work_command(d.address + " OFF");
      return;
    }
    int difference = current_work.level - d.brightness;
    if (difference == 0) { work_phase = WORK_IDLE; return; }
    work_brighter = difference > 0;
    work_steps = (abs(difference) + 8) / 16;
    // The validated X10 engine transmits a minimum of two repetitions.
    if (work_steps < 2) work_steps = 2;
    if (work_steps > 16) work_steps = 16;
    work_start_level = d.brightness;
    issue_work_command(d.address + (work_brighter ? " BRIGHT " : " DIM ") +
                       String(work_steps));
    return;
  }
  if (work_phase != WORK_IDLE || work_count == 0) return;
  WorkItem &head = work_queue[work_head];
  if (head.brightness &&
      (uint32_t)(now - head.last_update_ms) < brightness_debounce_ms) return;
  current_work = head;
  work_head = (work_head + 1) % WORK_QUEUE_SIZE;
  --work_count;

  if (!current_work.brightness) {
    work_phase = WORK_COMMAND;
    issue_work_command(current_work.command);
    return;
  }
  int idx = find_device(current_work.command);
  if (idx < 0 || !is_dimmer(devices[idx])) {
    result("ERROR BRIGHTNESS_DEVICE_NOT_FOUND " + current_work.command);
    return;
  }
  Device &d = devices[idx];
  if (current_work.level == 0) {
    // An explicit zero brightness request becomes an OFF command.
    current_work.brightness = false;
    current_work.command = d.address + " OFF";
    work_phase = WORK_COMMAND;
    issue_work_command(current_work.command);
    return;
  }
  if (d.state != 2 || d.brightness < 0) {
    work_phase = WORK_CAL_OFF;
    issue_work_command(d.address + " OFF");
    return;
  }
  work_phase = WORK_ADJUST;
  // The next loop iteration will calculate and issue the adjustment.
}

// ---------------------------------------------------------------------------
// Registry management
// ---------------------------------------------------------------------------
static void reset_registration_form() {
  reg_address=PANEL_NONE; reg_model=MODEL_NONE; reg_type=TYPE_DEFAULT; reg_name="";
  reg_house=MODEL_NONE; reg_unit=MODEL_NONE;
  for (uint8_t i=0;i<=3;++i) publish_panel_state(i);
  publish_panel_state(10); publish_panel_state(11);
}

static void manage_command(String line) {
  line.trim();
  if (line=="LIST") {
    for (uint8_t i=0;i<device_count;++i)
      result("DEVICE|"+devices[i].address+"|"+devices[i].name+"|"+
             (devices[i].model.length()?devices[i].model:String("-"))+"|"+TYPE_NAMES[devices[i].type]);
    for (uint8_t i=0;i<seen_count;++i)
      result("SEEN|"+seen[i].address+"|"+(seen[i].state==2?"ON":seen[i].state==1?"OFF":"UNKNOWN"));
    result("OK LIST "+String(device_count)+" SEEN "+String(seen_count));return;
  }
  int p=line.indexOf('|');
  if (p<0) {result("ERROR BAD_FORMAT");return;}
  String verb=line.substring(0,p);
  verb.trim(); verb.toUpperCase();
  String rest=line.substring(p+1);
  int q=rest.indexOf('|');
  String addr=q<0?rest:rest.substring(0,q);
  addr.trim();addr.toUpperCase();
  if (!valid_address(addr)) {result("ERROR INVALID_ADDRESS");return;}
  int idx=find_device(addr);
  if (verb=="REMOVE") {
    if (work_phase != WORK_IDLE || work_count != 0) {
      result("ERROR REMOVE_WHILE_COMMANDS_PENDING"); return;
    }
    if (idx<0) {result("ERROR NOT_FOUND");return;}
    Device old=devices[idx];
    String topic=discovery_topic(old);
    watch_discovery(old,false);   // our own empty config must not echo back
    if (!mqtt.publish(topic.c_str(),"",true)) {result("ERROR MQTT_REMOVE_FAILED");return;}
    publish_unknown(old);  // clear retained state/brightness
    for (int i=idx;i<(int)device_count-1;++i) devices[i]=devices[i+1];
    --device_count;save_registry();
    panel_refresh_pending=true;
    result("OK REMOVED "+addr);return;
  }
  if (verb=="FORGET") {
    int si=find_seen(addr);
    if (si<0) {result("ERROR NOT_SEEN");return;}
    remove_seen_at((uint8_t)si);
    seen_full_reported=false;
    result("OK FORGOTTEN "+addr);return;
  }
  if (verb=="RENAME") {
    if (idx<0 || q<0) {result("ERROR NOT_FOUND_OR_FORMAT");return;}
    String name=rest.substring(q+1);name.trim();
    if (!name.length() || name.length()>40 || name.indexOf('|')>=0) {result("ERROR INVALID_NAME");return;}
    devices[idx].name=name;save_registry();
    panel_refresh_pending=true;
    bool ok=publish_discovery_device(devices[idx]);
    result(ok?"OK RENAMED "+addr:"ERROR DISCOVERY_PUBLISH_FAILED");return;
  }
  if (verb=="ADD") {
    if (idx>=0) {result("ERROR DUPLICATE_ADDRESS");return;}
    if (device_count>=MAX_DEVICES || q<0) {result("ERROR CAPACITY_OR_FORMAT");return;}
    // tail = name|MODEL_OR_TYPE[|TYPE]
    String tail=rest.substring(q+1);
    int r=tail.indexOf('|');
    if (r<0) {result("ERROR BAD_FORMAT");return;}
    String name=tail.substring(0,r), spec=tail.substring(r+1), type_s="";
    int s=spec.indexOf('|');
    if (s>=0) { type_s=spec.substring(s+1); spec=spec.substring(0,s); }
    name.trim(); spec.trim(); spec.toUpperCase(); type_s.trim(); type_s.toUpperCase();
    if (!name.length() || name.length()>40) {result("ERROR INVALID_NAME");return;}
    String model="";
    int type=-1;
    int mi=find_model(spec);
    if (mi>=0) { model=MODELS[mi].model; type=MODELS[mi].type; }
    else {
      type=parse_type(spec);   // legacy V2.24: DIMMER / SWITCH (and LIGHT / SENSOR)
      if (type<0) {result("ERROR UNKNOWN_MODEL_OR_TYPE "+spec);return;}
    }
    if (type_s.length() && type_s!="MODEL DEFAULT") {
      int t=parse_type(type_s);
      if (t<0) {result("ERROR UNKNOWN_TYPE "+type_s);return;}
      type=t;
    }

    Device &d=devices[device_count];
    d.address=addr;
    d.name=name;
    d.model=model;
    d.type=(DevType)type;
    d.state=0;
    d.brightness=-1;
    d.publish_pending=false;
    d.momentary_armed=false;

    // A seen address carries its last known ON/OFF state into the registration.
    const bool from_form=(panel_address()==addr);  // registered from the HA panel
    int si=find_seen(addr);
    if (si>=0) {
      d.state=seen[si].state;
      d.brightness=(d.state==1)?0:-1;
      note_momentary_on(d);
      remove_seen_at((uint8_t)si);
      seen_full_reported=false;
    }

    ++device_count;
    save_registry();
    panel_refresh_pending=true;

    bool ok=publish_discovery_device(d);
    watch_discovery(d,true);
    result(ok?"OK ADDED "+addr+" "+(model.length()?model:String("-"))+" "+TYPE_NAMES[type]
             :"ERROR DISCOVERY_PUBLISH_FAILED");
    if (ok && from_form) reset_registration_form();
    return;
  }
  result("ERROR UNKNOWN_ACTION");
}

// Registry changes write NVS and publish discovery; hold them back while an
// X10 transaction is in progress.
static void manage_dispatch(String line) {
  line.trim();
  if (line=="LIST") {manage_command(line);return;}
  if (manage_pending_count==0 && !manage_busy()) {manage_command(line);return;}
  if (manage_pending_count>=MANAGE_PENDING_MAX) {result("ERROR MANAGE_QUEUE_FULL");return;}
  manage_pending[manage_pending_count++]=line;
  result("QUEUED MANAGE");
}

static void register_from_panel() {
  String addr=panel_address();
  if (!addr.length()) {result("ERROR SELECT_HOUSE_AND_UNIT");return;}
  if (find_device(addr)>=0) {result("ERROR DUPLICATE_ADDRESS "+addr);return;}
  if (reg_model==MODEL_NONE) {result("ERROR SELECT_MODEL");return;}
  String name=reg_name; name.trim();
  int mi=find_model(reg_model);
  if (mi<0) {result("ERROR SELECT_MODEL");return;}
  if (!name.length()) name="X10 "+String(MODELS[mi].model)+" "+addr;
  String line="ADD|"+addr+"|"+name+"|"+MODELS[mi].model;
  if (reg_type!=TYPE_DEFAULT) line+="|"+reg_type;
  manage_dispatch(line);
}

// x10/register/<item>/set and x10/register/<item>/press from Home Assistant.
static void handle_register_topic(const String &topic, String value) {
  value.trim();
  if (topic=="x10/register/address/set") {
    if (value==PANEL_NONE || find_seen(value)>=0) reg_address=value;
    else result("ERROR NOT_SEEN "+value);
    if (reg_address!=PANEL_NONE) {
      // Fill House Code / Unit Code from the seen address.
      reg_house=reg_address.substring(0,1);
      reg_unit=reg_address.substring(1);
      publish_panel_state(10); publish_panel_state(11);
    }
    publish_panel_state(0); return;
  }
  if (topic=="x10/register/house/set" || topic=="x10/register/unit/set") {
    bool house=(topic=="x10/register/house/set");
    if (value==MODEL_NONE) {
      if (house) reg_house=MODEL_NONE; else reg_unit=MODEL_NONE;
    } else if (house) {
      String v=value; v.toUpperCase();
      if (v.length()==1 && v[0]>='A' && v[0]<='P') reg_house=v;
    } else {
      int u=value.toInt();
      if (u>=1 && u<=16 && value==String(u)) reg_unit=value;
    }
    // Keep the Seen Address shortcut in step with the typed address.
    String a=panel_address();
    reg_address=(a.length() && find_seen(a)>=0)?a:String(PANEL_NONE);
    publish_panel_state(house?10:11); publish_panel_state(0); return;
  }
  if (topic=="x10/register/model/set") {
    int mi=find_model(value);
    if (value==MODEL_NONE) reg_model=value;
    else if (mi>=0) reg_model=model_label(mi);
    publish_panel_state(1); return;
  }
  if (topic=="x10/register/type/set") {
    if (value==TYPE_DEFAULT || parse_type(value)>=0) reg_type=value;
    publish_panel_state(2); return;
  }
  if (topic=="x10/register/name/set") {
    if (value.length()<=40 && value.indexOf('|')<0) reg_name=value;
    else result("ERROR INVALID_NAME");
    publish_panel_state(3); return;
  }
  if (topic=="x10/register/remove_target/set") {
    rm_target=value;          // sanitize_panel() rejects unknown values
    publish_panel_state(5); return;
  }
  if (topic=="x10/register/submit/press") { register_from_panel(); return; }
  if (topic=="x10/register/remove/press") {
    sanitize_panel();
    if (rm_target==PANEL_NONE) {result("ERROR SELECT_DEVICE_TO_REMOVE");return;}
    int sp=rm_target.indexOf(' ');
    manage_dispatch("REMOVE|"+(sp>0?rm_target.substring(0,sp):rm_target));
    return;
  }
  if (topic=="x10/register/forget/press") {
    if (reg_address==PANEL_NONE) {result("ERROR SELECT_SEEN_ADDRESS");return;}
    manage_dispatch("FORGET|"+reg_address);
    return;
  }
  // Our own .../state publications also arrive here; ignore them.
}

static void apply_debounce(int value) {
  brightness_debounce_ms=(uint32_t)value;
  prefs.begin("x10-settings",false);
  prefs.putUInt("debounce",brightness_debounce_ms);
  prefs.end();
  if (mqtt.connected())
    mqtt.publish("x10/settings/brightness_debounce/state",
                 String(brightness_debounce_ms).c_str(),true);
  result("OK DEBOUNCE "+String(brightness_debounce_ms));
}

static void service_deferred_work() {
  if (pending_debounce_ms>=0 && !tx_active()) {
    int v=pending_debounce_ms;pending_debounce_ms=-1;
    apply_debounce(v);
  }
  if (manage_pending_count>0 && !manage_busy()) {
    String line=manage_pending[0];
    for (uint8_t i=1;i<manage_pending_count;++i) manage_pending[i-1]=manage_pending[i];
    --manage_pending_count;
    manage_command(line);
  }
  // NVS writes only while the line is idle.
  if (seen_save_pending && !manage_busy()) save_seen();
  service_deferred_publish();
}

static void on_mqtt_message(char *topic,byte *payload,unsigned int length) {
  if (length>180) return;
  // Copy topic and payload first: publishing from here reuses the client buffer.
  char buf[181];memcpy(buf,payload,length);buf[length]=0;
  String t(topic);

  if (t==TOPIC_HA_STATUS) {
    if (strcmp(buf,"online")==0) {
      // Home Assistant restarted: republish discovery and every known state.
      request_full_discovery();
      for (uint8_t i=0;i<device_count;++i) devices[i].publish_pending=true;
      for (uint8_t i=0;i<seen_count;++i) seen[i].publish_pending=true;
    }
    return;
  }
  if (t.startsWith("homeassistant/") && t.endsWith("/config")) {
    // Only an EMPTY payload matters: HA deleted the device. Our own retained
    // configs (non-empty) echo back here and are ignored.
    if (length==0) {
      for (uint8_t i=0;i<device_count;++i) {
        if (t!=discovery_topic(devices[i])) continue;
        String a=devices[i].address;
        Serial.println("[MANAGE] Deleted in Home Assistant: "+a);
        manage_dispatch("REMOVE|"+a);
        break;
      }
    }
    return;
  }
  if (t.startsWith(TOPIC_REGISTER_PREFIX)) {
    if (t.endsWith("/set") || t.endsWith("/press")) handle_register_topic(t,String(buf));
    return;
  }
  if (length==0) return;
  if (t==TOPIC_MANAGE) {manage_dispatch(String(buf));return;}
  if (t==TOPIC_COMMAND) {
    submit_command(String(buf), "MQTT");
    return;
  }

  // Adjustable brightness debounce setting
  if (t=="x10/settings/brightness_debounce/set") {
    if (length<3 || length>4) return;
    for (unsigned int k=0;k<length;++k) {
      if (buf[k]<'0' || buf[k]>'9') return;
    }
    int value=atoi(buf);
    if (value<500 || value>2000 || value%100!=0) {
      result("ERROR INVALID_DEBOUNCE");
      return;
    }
    // Applied from loop() when no X10 transaction is active.
    pending_debounce_ms=value;
    return;
  }

  for (uint8_t i=0;i<device_count;++i) {
    Device &d=devices[i];
    if (!is_dimmer(d) || t!=bright_set_topic(d)) continue;
    if (length>3) return;
    for (unsigned int k=0;k<length;++k) {
      if (buf[k]<'0' || buf[k]>'9') return;
    }
    int target=atoi(buf);
    if (target<0 || target>255) return;
    submit_brightness(d.address, target);
    return;
  }
}

// One discovery packet per loop pass so X10 timing is never blocked for long:
// registration panel, then registered devices, then seen devices.
static void service_discovery() {
  if (tx_active()) return;
  if (discovery_pending) {
    uint16_t i=discovery_index;
    bool ok;
    if (i<PANEL_ITEMS) ok=publish_panel_item((uint8_t)i);
    else if (i<PANEL_ITEMS+device_count) ok=publish_discovery_device(devices[i-PANEL_ITEMS]);
    else if (i<PANEL_ITEMS+device_count+seen_count) ok=publish_seen_discovery(seen[i-PANEL_ITEMS-device_count]);
    else { discovery_pending=false; panel_refresh_pending=false; return; }
    if (ok) ++discovery_index;
    return;
  }
  if (panel_refresh_pending) {
    // Option lists changed (seen/registered devices added or removed).
    if (publish_panel_item(0) && publish_panel_item(5)) panel_refresh_pending=false;
    return;
  }
  for (uint8_t i=0;i<seen_count;++i) {
    if (seen[i].discovery_pending) { publish_seen_discovery(seen[i]); return; }
  }
}

static void service_network() {
  uint32_t now=millis();
  if (WiFi.status()!=WL_CONNECTED) {
    if ((uint32_t)(now-last_wifi_attempt_ms)>=10000U || last_wifi_attempt_ms==0) {
      last_wifi_attempt_ms=now;WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
    }
    return;
  }
  if (!mqtt.connected()) {
    // mqtt.connect() can block for seconds; never during a transaction.
    if (tx_active()) return;
    if ((uint32_t)(now-last_mqtt_attempt_ms)>=5000U || last_mqtt_attempt_ms==0) {
      last_mqtt_attempt_ms=now;
      String id="esp32-x10-"+String((uint32_t)(ESP.getEfuseMac()&0xFFFFFF),HEX);
      bool ok=strlen(MQTT_USER)?mqtt.connect(id.c_str(),MQTT_USER,MQTT_PASSWORD,
                         TOPIC_AVAILABILITY,0,true,"offline"):
                     mqtt.connect(id.c_str(),TOPIC_AVAILABILITY,0,true,"offline");
      if (ok) {
        mqtt.publish(TOPIC_AVAILABILITY,"online",true);
        String debounce_state=String(brightness_debounce_ms);
        mqtt.publish("x10/settings/brightness_debounce/state",debounce_state.c_str(),true);
        mqtt.subscribe(TOPIC_COMMAND);
        mqtt.subscribe(TOPIC_MANAGE);
        mqtt.subscribe("x10/device/+/brightness/set");
        mqtt.subscribe("x10/settings/brightness_debounce/set");
        mqtt.subscribe("x10/register/#");
        mqtt.subscribe(TOPIC_HA_STATUS);
        for (uint8_t i=0;i<device_count;++i) watch_discovery(devices[i],true);
        request_full_discovery();
        result("OK CONNECTED");
      } else {Serial.print("[MQTT] Connect failed ");Serial.println(mqtt.state());}
    }
    return;
  }

  mqtt.loop();
  service_discovery();
}

// MOMENTARY devices: show OFF again once the module's contact closure is over.
// Only the HA state changes; nothing is sent on the power line.
static void service_momentary(uint32_t now) {
  for (uint8_t i=0;i<device_count;++i) {
    Device &d=devices[i];
    if (!d.momentary_armed) continue;
    if (d.type!=T_MOMENTARY || d.state!=2) { d.momentary_armed=false; continue; }
    if ((uint32_t)(now-d.momentary_since)<MOMENTARY_RESET_MS) continue;
    d.momentary_armed=false;
    d.state=1;
    d.publish_pending=true;
  }
}

static void load_brightness_settings() {
  prefs.begin("x10-settings",true);
  uint32_t saved=prefs.getUInt("debounce",500);
  prefs.end();
  if (saved>=500 && saved<=2000 && saved%100==0) {
    brightness_debounce_ms=saved;
  } else {
    brightness_debounce_ms=500;
  }
  Serial.print("[X10] Brightness debounce: ");
  Serial.print(brightness_debounce_ms);
  Serial.println(" ms");
}

void setup() {
  Serial.begin(115200);delay(500);
  Serial.print("ESP32 PSC05 X10 IOT Gateway V");Serial.println(FW_VERSION);
  load_registry();
  load_seen();
  load_brightness_settings();
  x10_psc05.set_event_callback([](const std::string &message) {
    Serial.print("[X10] ");Serial.println(message.c_str());
    note_engine_event(message);
    track_external_rx(message);
    publish_confirmed_state(message);
    if (mqtt.connected() && message.compare(0,4,"RAW ")!=0)
      mqtt.publish(TOPIC_EVENT,message.c_str());
  });
  x10_psc05.set_stats_callback([](const std::string &message) {
    Serial.print("[STAT] ");Serial.println(message.c_str());
    // Skip the (potentially blocking) publish mid-transaction.
    if (mqtt.connected() && !tx_active()) mqtt.publish(TOPIC_STATUS,message.c_str());
  });
  x10_psc05.begin();
  WiFi.mode(WIFI_STA);WiFi.setAutoReconnect(true);
  // Discovery payloads with option lists (up to 32 registered names) need room.
  mqtt.setBufferSize(3072);
  mqtt.setServer(MQTT_HOST,MQTT_PORT);
  mqtt.setCallback(on_mqtt_message);
  mqtt.setSocketTimeout(1);mqtt.setKeepAlive(15);
}

void loop() {
  x10_psc05.loop();
  uint32_t now=millis();
  if ((uint32_t)(now-last_stats_ms)>=1000U) {
    last_stats_ms=now;x10_psc05.request_stats();
  }
  while (Serial.available()>0) {
    char c=(char)Serial.read();
    if (c=='\r'||c=='\n') {
      if (serial_line.length()) {
        // Serial console: "MANAGE ADD|C3|Lamp|LM465" or an X10 command "A1 ON".
        if (serial_line.startsWith("MANAGE ")) manage_dispatch(serial_line.substring(7));
        else submit_command(serial_line,"SERIAL");
        serial_line="";
      }
    } else if (c==8||c==127) {
      if (serial_line.length()) serial_line.remove(serial_line.length()-1);
    } else if (serial_line.length()<79) serial_line+=c;
  }
  service_network();

  // Deferred management / debounce / NVS / state publishes (idle only).
  service_deferred_work();

  // Process settled brightness requests.
  service_work_queue(millis());

  // MOMENTARY auto-OFF.
  service_momentary(millis());
}
