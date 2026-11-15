# X10 IOT Gateway (V3.03)

This gateway connects an ESP32 and an X10 **PSC05** (or TW523) power-line interface to **Home Assistant** over MQTT.

* **Registered modules:** Home Assistant controls them as lights, switches or binary sensors. The ESP32 tracks their ON/OFF state and brightness and keeps Home Assistant up to date. Commands sent by other X10 controllers also update these states.
* **Unregistered modules:** when another X10 controller sends commands to an address that isn't registered, the gateway creates a **binary sensor** in Home Assistant ("X10 Unregistered C3") that follows that address's ON/OFF state.
* **Registration:** Home Assistant gets a registration panel. Pick one of the unregistered addresses, choose its model and press **Register**. The sensor is then replaced by a controllable entity, and the last known state is carried over.

The sketch is built on the hardware-validated **V2.17.1 X10 engine**. The engine's **code is unchanged**, so PSC05 loopback verification, collision backoff, TX polarity and the core 1.0.6 timers behave exactly as before. Only its TX-driver comments were corrected to describe the 2N7000 + NTE129 driver. The application layer evolved from the V2.24 sketch, and every V2.24 topic and management command still works.

## Files

| Path | Purpose |
|---|---|
| `ESP32_PSC05_X10_MQTT_V3_03/ESP32_PSC05_X10_MQTT_V3_03.ino` | Main sketch |
| `ESP32_PSC05_X10_MQTT_V3_03/x10_psc05_v2_17_backoff_diagnostic_esp32_core_1_0_6.h` | X10 engine V2.17.1 (code unchanged; TX-driver comments corrected) |
| `ESP32_PSC05_X10_MQTT_V3_03/x10_types.h` | Type definitions (kept in a separate tab so Arduino IDE's automatic prototypes compile) |
| `ESP32_PSC05_X10_MQTT_V3_03/x10_secrets.example.h` | Template. On a first install, copy it to `x10_secrets.h` in the same folder and fill in your Wi-Fi and MQTT details. When upgrading, copy your existing `x10_secrets.h` in instead. |
| `homeassistant/packages/x10_manager.yaml` | Your HA package, updated for models, plus a "new address seen" notification |
| `homeassistant/x10_registration_card.yaml` | Dashboard card for the registration panel |

## Build

**First install:** copy `x10_secrets.example.h` to `x10_secrets.h` in the sketch folder and fill in your Wi-Fi and MQTT details. `x10_secrets.h` is listed in `.gitignore`, so it is never committed.

* Arduino IDE 1.8.19 with **ESP32 Arduino core 1.0.6** (the engine uses the 1.0.x timer API)
* Library: **PubSubClient 2.8** (Nick O'Leary)
* Board: ESP32 Dev Module (DevKit V1)

## Wiring (unchanged from V2.17.1)

| PSC05 RJ11 pin | Signal | ESP32 |
|---|---|---|
| 1 | Zero crossing (open collector) | GPIO34, external 4.7 k pull-up to 3.3 V |
| 2 | Common | GND |
| 3 | X10 RX (open collector, active low) | GPIO35, external 4.7 k pull-up to 3.3 V |
| 4 | X10 TX input | GPIO25 through the 5 V driver below. **Do not drive it directly from 3.3 V**: the TW523 note asks for a logic "1" of at least 4 V. |

GPIO34 and GPIO35 are input-only and have no internal pull-ups.

### TX driver

The PSC05's TX input (pin 4) needs a 5 V logic level: at least 4 V for a "1", per the [PL513 / TW523 Technical Note](https://cdn.shopify.com/s/files/1/2279/4329/files/PL513_and_TW523_technical_notes.pdf?13630872876378945203). This build uses the note's page 11 "Typical O.E.M. Connection Diagram" transmit circuit (a PNP stage, here an NTE129). The ESP32's 3.3 V GPIO25 can't switch that 5 V PNP stage directly, so a **2N7000 N-channel MOSFET** is added in front of it to interface the 3.3 V GPIO to the 5 V O.E.M. transmit line:

```
GPIO25 ──300Ω──┬── 2N7000 gate          2N7000 source ── GND
               └─100k── GND             2N7000 drain ──10k── NTE129 (PNP) base
                                                  5 V ──10k── NTE129 base
                                        NTE129 emitter ── 5 V
                                        NTE129 collector ──100Ω── PSC05 pin 4
                                                          └─10k── GND
```

| GPIO25 | 2N7000 | NTE129 | PSC05 pin 4 | Power line |
|---|---|---|---|---|
| HIGH | on | on | ~5 V | 120 kHz carrier on |
| LOW | off | off | 0 V (10 k pull-down) | silent |

Each stage inverts, so the complete path doesn't: GPIO25 HIGH means carrier on. That matches the engine settings `TX_ACTIVE_LEVEL = HIGH` and `TX_IDLE_LEVEL = LOW`.

The driver is fail-safe. During boot, reset or flashing, or with the ESP32 unpowered, the 100 k pull-down holds the 2N7000 off, so the PNP stays off and nothing is transmitted.

If you build a driver that inverts (for example, a single transistor pulling pin 4 low), swap the two engine settings. Be aware that such a driver transmits continuous carrier whenever the ESP32 isn't driving GPIO25.

## Device models and Home Assistant entity types

| Type | HA entity | Behaviour |
|---|---|---|
| `DIMMER` | `light` with brightness | ON/OFF, brightness calibration (V2.24 algorithm), ALL_LIGHTS_* |
| `LIGHT` | `light`, on/off only | ON/OFF, ALL_LIGHTS_* |
| `SWITCH` | `switch` | ON/OFF, ALL_UNITS_OFF |
| `SENSOR` | `binary_sensor` | Receive only. Follows ON/OFF sent to its address and ignores ALL_* broadcasts. |
| `MOMENTARY` | `switch` | For modules that switch back off by themselves, such as a UM506 in momentary mode (contacts close for about 1.5–2.5 s). Shows ON for 2.5 s after an ON from HA or another controller, then reverts to OFF. Nothing is sent on the power line when it reverts. Responds to ALL_UNITS_OFF. |

Names marked † come from the manual index at thex10shop.com. Names marked ‡ are IBM modules confirmed on real hardware. The other names are only descriptive.

| Model | Name | Default type | Notes |
|---|---|---|---|
| 7246 | Appliance Module | SWITCH | Verify |
| AMT-101 | Appliance Module | SWITCH | Verify |
| AMT-103 | Appliance Module | SWITCH | Verify |
| HD11A | Appliance Module | SWITCH | Verify |
| HD465 | Lamp Module ‡ | DIMMER | IBM. Brightness reference used for dimmer calibration since V2.24. |
| HD466 | 3-Pin Appliance Module ‡ | SWITCH | IBM |
| HD501 | Wall Switch | DIMMER | Verify |
| LM15A-C | Socket Rocket Lamp Module † | LIGHT | No dimming |
| LM465 | Lamp Module † | DIMMER | |
| PMS02 | Motion Sensor | SENSOR (motion) | Verify |
| SR227 | SuperSocket Outlet † | SWITCH | |
| TM523 | Transceiver Module | SWITCH | Verify |
| TM751 | Transceiver Module † | SWITCH | Transceiver with appliance relay |
| UM506 | Universal Module † | SWITCH | Use MOMENTARY when the module's slide switch is set to Momentary |
| WS12A | Decorator Dimmer Switch † | DIMMER | |
| WS467 | Wall Switch Module † | DIMMER | |

I couldn't confirm the exact product type for the models marked "verify". If a default is wrong, pick the correct **Type** when you register the device. To change the defaults permanently, edit the `MODELS[]` table in the sketch.

## Registering a device in Home Assistant

Everything is done on the **ESP32 PSC05 X10 Controller** device page (Settings → Devices & services → MQTT), under *Configuration*. No computer needs to be connected to the ESP32.

1. Enter the module's address. Either:
   - set **X10 Register: House Code** (A–P) and **X10 Register: Unit Code** (1–16) to match the dials on the module, or
   - if another controller has already used the address, pick it in **X10 Register: Seen Address**. House Code and Unit Code fill in automatically.
2. Set **X10 Register: Model**, for example "LM465 - Lamp Module".
3. Optional:
   - **X10 Register: Type**: leave on `Model default`, or override it (for example MOMENTARY for a UM506 in momentary mode).
   - **X10 Register: Name**: the default is `X10 <model> <address>`.
4. Press **X10 Register Device**.

**X10 Registration Result** (under *Diagnostic*) shows `OK ADDED D7 UM506 MOMENTARY` or an error:
- `ERROR SELECT_HOUSE_AND_UNIT`: House Code or Unit Code isn't set.
- `ERROR SELECT_MODEL`: no model chosen.
- `ERROR DUPLICATE_ADDRESS D7`: that address is already registered. Remove it first.

If the address was in the seen list, its "X10 Unregistered" sensor is replaced and its last known state carries over. The form clears after a successful registration.

### Removing a device

Either way works and keeps the gateway and Home Assistant in step:
- On the controller device, choose it in **X10 Remove: Registered Device** and press **X10 Remove Device**, or
- delete the device itself in Home Assistant (Settings → Devices & services → MQTT → the device → ⋮ → **Delete**). HA clears the device's discovery message, the gateway sees that and removes it from its registry, and **X10 Registration Result** shows `OK REMOVED B1`.

You can then register the address again.

**X10 Forget Seen Address** drops an unregistered sensor; it comes back if that address is used again.

Limits: 32 registered devices and 32 unregistered (seen) addresses. Both lists are stored in NVS and survive reboots. Registry changes and NVS writes wait until no X10 transmission is in progress, as in V2.24.

## MQTT reference

| Topic | Direction | Payload |
|---|---|---|
| `x10/command` | in | `A1 ON`, `A1 OFF`, `C7 DIM 5`, `C7 BRIGHT 3`, `A ALL_OFF`, `A ALL_ON`, `A ALL_LIGHTS_OFF` |
| `x10/manage/set` | in | `ADD\|C3\|Name\|LM465`, `ADD\|C3\|Name\|HD501\|DIMMER`, `ADD\|A1\|Name\|UM506\|MOMENTARY`, `ADD\|C3\|Name\|DIMMER` (V2.24 form), `RENAME\|C3\|Name`, `REMOVE\|C3`, `FORGET\|C3`, `LIST` |
| `x10/manage/result` | out | `OK ADDED C3 LM465 DIMMER`, `DEVICE\|…`, `SEEN\|…`, `ERROR …` |
| `x10/event` | out | Engine events (`TX_OK`, `RX`, `COLLISION`, …) and `SEEN_NEW C3` |
| `x10/status` | out | Engine statistics |
| `x10/availability` | out, retained | `online` / `offline` |
| `x10/device/<addr>/state` | out, retained | Lights: `C3 ON`. Switches and sensors: `ON` / `OFF`. |
| `x10/device/<addr>/brightness/set` and `/state` | in / out | 0–255 (DIMMER only) |
| `x10/seen/<addr>/state` and `/attributes` | out, retained | `ON` / `OFF`, plus JSON with the last function |
| `x10/register/<item>/set` and `/press` | in | Registration panel (HA discovery entities) |

On the serial console, type an X10 command (`A1 ON`) or `MANAGE ADD|C3|Lamp|LM465`.

## State tracking rules

* Address frames build a per-house address set. A function frame applies to that set, and the next address frame after a function starts a new set (standard X10 behaviour, same as V2.24).
* `ALL_UNITS_OFF` turns off every registered lamp and switch in the house, plus the house's unregistered addresses. `ALL_LIGHTS_ON` and `ALL_LIGHTS_OFF` apply only to `DIMMER` and `LIGHT` devices, because the type of an unregistered address is unknown.
* DIM/BRIGHT from another controller marks a dimmer's brightness as unknown. The PSC05 reports only the first dim code and every third one after it, so the dim amount can't be measured. On an unregistered address, DIM/BRIGHT counts as ON.
* After a reboot, registered devices show *unknown* until a command is confirmed (V2.24 behaviour). Unregistered sensors keep the last value retained on the broker.
* When Home Assistant restarts (`homeassistant/status` = `online`), the gateway republishes all discovery and state messages.

## Upgrading from V3.02

Same as below: unzip, copy your `x10_secrets.h` into `ESP32_PSC05_X10_MQTT_V3_03/`, build and upload. Registered devices are kept.

If you deleted a device in HA while running V3.02, the gateway may still have it. Remove it once with **X10 Remove Device** (or delete it in HA again after upgrading, if HA still shows it).

## Upgrading from V3.01

Unzip into a new folder, then copy your existing `x10_secrets.h` into the new sketch folder. The zip deliberately has no `x10_secrets.h`, so it can't overwrite your credentials. If you forget to copy it, the build stops with `x10_secrets.h: No such file or directory`. Registered devices are kept.

## Upgrading from V3.00

The sketch folder and `.ino` were renamed `ESP32_PSC05_X10_MQTT_V3_01` (Arduino IDE needs the folder and sketch names to match). Copy your filled-in `x10_secrets.h` into the new folder before building. Registered devices and seen addresses are kept, because they're stored in the ESP32's flash, not in the sketch.

To switch an existing UM506 to MOMENTARY, remove it and add it again, for example `MANAGE REMOVE|A1` then `MANAGE ADD|A1|Name|UM506|MOMENTARY`.

## Upgrading from V2.24

Flash V3.00 and your NVS registry migrates automatically. Existing entries keep their type (dimmer or switch), with no model set. Their unique IDs and discovery topics are unchanged, so the Home Assistant entities stay the same. To add a model to an existing entry, remove it and register it again.

V2.24 cleared `homeassistant/switch/esp32_psc05_B1/config` on every connect as a one-time migration. V3.00 drops that step.

## References

- [X10 PL513 / TW523 Power Line Interface Technical Note](https://cdn.shopify.com/s/files/1/2279/4329/files/PL513_and_TW523_technical_notes.pdf?13630872876378945203) (Rev 2.4): X10 code format, transmit and receive timing, RJ11 pin-out, electrical characteristics and the page 11 O.E.M. connection diagram. The PSC05 is the current TW523-compatible two-way interface.
- [X10 product manuals](https://thex10shop.com/pages/x10-instruction-manuals) (TheX10Shop): source of the module names marked † in the model table.

## License

MIT. See [LICENSE](LICENSE).
