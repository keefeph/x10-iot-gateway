/*
 * V2.17.1 diagnostic: TX polarity for the 2N7000 + NTE129 (PNP) driver.
 * GPIO25 HIGH = active; GPIO25 LOW = idle X10 envelope.
 * (V3.00 note: comments corrected only; no code changed.)
 * RX timing and V2.15 verified bit order are unchanged.
 */
#pragma once

#include <Arduino.h>
#include <functional>
#include <string>
#include <cstring>
#include <cctype>
#include <cstdio>
#include <cstdlib>

/*
 * ESP32 DevKit V1 + X10 PSC05 controller
 *
 * PSC05 RJ11/OEM interface:
 *   Pin 1 = ZERO CROSSING -> GPIO34
 *   Pin 2 = COMMON       -> ESP32 GND
 *   Pin 3 = X10 RX       -> GPIO35, external 4.7K pull-up to 3.3V
 *   Pin 4 = X10 TX       <- GPIO25 through a 5V driver: 300R -> 2N7000 gate
 *                           (100K gate to GND); drain -> TW523 note page 11
 *                           NTE129 PNP stage -> 100R -> pin 4
 *
 * IMPORTANT:
 *   PSC05 pin 4 requires a logic-high level of at least about 4V.
 *   Do NOT drive it directly from the 3.3V ESP32 GPIO.
 *
 * GPIO34/GPIO35 are input-only and do not have internal pull-ups.
 *
 * V2.15 changes:
 *   - PSC05-aware RX processing; no raw 120 kHz decoding.
 *   - Explicit TX state machine.
 *   - 512 half-cycle waveform buffer.
 *   - Correct DIM/BRIGHT RX verification: first code + every third code.
 *   - One-shot 1 ms TX envelope timer.
 *   - TX timeout derived from waveform length.
 *   - Separate retry count from DIM/BRIGHT repeat count.
 *   - Collision verification against PSC05's validated RX codes.
 *   - External traffic is not automatically called a collision after TX.
 *   - RX decoder rewritten as one chronological half-cycle stream.
 *   - PSC05 Start Code 1110 is used as RX framing.
 *   - RX data bits are decoded from true/complement pairs.
 *   - Expected RX values contain house+key data only; Start is framing.
 *   - RX is sampled as a level 600 us after every zero crossing.
 *   - RX falling-edge interrupt removed; ZERO CROSS is the RX clock.
 *   - Dedicated RX one-shot timer is separate from the 1 ms TX timer.
 *   - ISR-to-loop RX handoff uses a fixed ring buffer (head/tail).
 *   - Correct X10 wire order: H1,H2,H4,H8 then D1,D2,D4,D8,D16.
 *   - TX emits LSB-first; RX places chronological bits explicitly.
 *   - Invalid RX true/complement pairs reset RX-only decoding.
 *   - During TX_TRANSMIT, an invalid pair is a collision and aborts TX for retry/backoff.
 *   - Retries keep ownership of current_; failed commands are not requeued.
 *   - retry_count_ persists across attempts; every retry gets fresh random backoff.
 *   - TX_TRANSMIT processes collision before normal waveform completion.
 *   - Zero-cross ISR latches TX LOW whenever tx_collision_ is set.
 *   - Valid wrong RX code is a collision only during TX_TRANSMIT.
 *   - TX_WAIT_RX mismatches are external/unverified traffic, not collisions.
 *   - Line arbitration now uses raw sampled PSC05 carrier activity.
 *   - Arbitration separates WAIT_QUIET from randomized BACKOFF.
 */

class X10PSC05 {
 public:
  static constexpr uint8_t ZC_PIN = 34;
  static constexpr uint8_t RX_PIN = 35;
  static constexpr uint8_t TX_PIN = 25;
  // TX driver = 2N7000 + NTE129 (PNP) stage of the TW523 note, page 11.
  // Each stage inverts, so the complete path is NON-inverting:
  // GPIO HIGH -> 2N7000 on  -> NTE129 on  -> PSC05 TX ~5V -> carrier ON
  // GPIO LOW  -> 2N7000 off -> NTE129 off -> PSC05 TX 0V  -> carrier OFF (idle)
  // Fail-safe: the 100K gate pull-down keeps the carrier off during boot/reset.
  static constexpr uint8_t TX_IDLE_LEVEL = LOW;
  static constexpr uint8_t TX_ACTIVE_LEVEL = HIGH;

  static constexpr uint8_t QUEUE_SIZE = 16;

  // Maximum waveform:
  // address pair = 50 half-cycles
  // 16 DIM/BRIGHT codes = 352 half-cycles
  // final gap = 6 half-cycles
  // 50 + 352 + 6 = 408
  static constexpr uint16_t MAX_WAVE_BITS = 512;

  static constexpr uint8_t MAX_RX_CODES = 32;
  static constexpr uint8_t MAX_RETRIES = 5;

  // One X10 half-cycle is approximately 8.333 ms at 60 Hz.
  static constexpr uint16_t HALF_CYCLE_MS = 9;
  static constexpr uint16_t RX_SAMPLE_US = 600;
  static constexpr uint8_t RX_QUEUE_SIZE = 64;

  enum Op : uint8_t {
    OP_ON,
    OP_OFF,
    OP_DIM,
    OP_BRIGHT,
    OP_ALL_UNITS_OFF,
    OP_ALL_LIGHTS_ON,
    OP_ALL_LIGHTS_OFF
  };

  struct Command {
    uint8_t house = 0;
    uint8_t unit = 0;
    uint8_t repeat = 1;  // DIM/BRIGHT count; normal commands use 1.
    Op op = OP_OFF;
  };

  enum TxState : uint8_t {
    TX_IDLE,
    TX_WAIT_QUIET,
    TX_BACKOFF,
    TX_TRANSMIT,
    TX_WAIT_RX
  };

  X10PSC05() = default;

  void set_event_callback(std::function<void(const std::string &)> cb) {
    event_cb_ = cb;
  }

  void set_stats_callback(std::function<void(const std::string &)> cb) {
    stats_cb_ = cb;
  }

  void request_stats() {
    pending_stats_ = true;
  }

  void begin() {
    instance_ = this;

    pinMode(ZC_PIN, INPUT);
    pinMode(RX_PIN, INPUT);
    pinMode(TX_PIN, OUTPUT);

    digitalWrite(TX_PIN, TX_IDLE_LEVEL);

    /*
     * Two independent 1 MHz one-shot timers are used:
     *
     *   tx_timer_ - terminates each PSC05 TX envelope after 1000 us.
     *   rx_timer_ - samples PSC05 RX exactly RX_SAMPLE_US after
     *               each zero-cross boundary.
     *
     * Keeping the timers separate prevents an RX sample from changing
     * the duration of a TX envelope, or vice versa.
     */
    tx_timer_ = timerBegin(0, 80, true);
    if (tx_timer_) {
      timerAttachInterrupt(tx_timer_, &X10PSC05::tx_timer_isr, true);
      timerAlarmWrite(tx_timer_, 1000, false);
      timerAlarmDisable(tx_timer_);
    }

    rx_timer_ = timerBegin(1, 80, true);
    if (rx_timer_) {
      timerAttachInterrupt(rx_timer_, &X10PSC05::rx_sample_isr, true);
      timerAlarmWrite(rx_timer_, RX_SAMPLE_US, false);
      timerAlarmDisable(rx_timer_);
    }

    /*
     * Both edges of PSC05 ZERO CROSS represent X10 half-cycle
     * boundaries.  RX itself is not interrupt-driven.  ZERO CROSS is
     * the receive clock; RX is sampled as a level 600 us later.
     */
    attachInterrupt(digitalPinToInterrupt(ZC_PIN), &X10PSC05::zc_isr, CHANGE);

    randomSeed((uint32_t)ESP.getEfuseMac() ^ micros());

    publish_event("READY");
    pending_stats_ = true;
  }

  void loop() {
    process_rx_events();
    service_tx_state();

    if (pending_stats_) {
      pending_stats_ = false;
      publish_stats();
    }
  }

  /*
   * Accepted MQTT command strings:
   *
   *   A1 ON
   *   A1 OFF
   *   B4 ON
   *   B4 OFF
   *   C7 DIM 5
   *   C7 BRIGHT 3
   *   A ALL_OFF
   *   A ALL_ON
   *   A ALL_LIGHTS_OFF
   */
  bool command(const std::string &input) {
    char s[80];

    size_t n = input.size();
    if (n >= sizeof(s)) {
      n = sizeof(s) - 1;
    }

    memcpy(s, input.data(), n);
    s[n] = '\0';

    for (size_t i = 0; i < n; i++) {
      if (s[i] == '\t' || s[i] == '\r' || s[i] == '\n') {
        s[i] = ' ';
      }
      s[i] = (char)toupper((unsigned char)s[i]);
    }

    char *tok1 = strtok(s, " ");
    char *tok2 = strtok(nullptr, " ");
    char *tok3 = strtok(nullptr, " ");

    if (!tok1 || !tok2) {
      publish_event("ERROR BAD_COMMAND");
      return false;
    }

    Command c{};

    /*
     * Broadcast commands have a house code but no unit address.
     */
    if (strcmp(tok2, "ALL_OFF") == 0 ||
        strcmp(tok2, "ALL_UNITS_OFF") == 0) {

      if (!parse_house(tok1, c.house)) {
        publish_event("ERROR BAD_HOUSE");
        return false;
      }

      c.op = OP_ALL_UNITS_OFF;
      c.unit = 0;
      c.repeat = 1;

    } else if (strcmp(tok2, "ALL_ON") == 0 ||
               strcmp(tok2, "ALL_LIGHTS_ON") == 0) {

      if (!parse_house(tok1, c.house)) {
        publish_event("ERROR BAD_HOUSE");
        return false;
      }

      c.op = OP_ALL_LIGHTS_ON;
      c.unit = 0;
      c.repeat = 1;

    } else if (strcmp(tok2, "ALL_LIGHTS_OFF") == 0) {

      if (!parse_house(tok1, c.house)) {
        publish_event("ERROR BAD_HOUSE");
        return false;
      }

      c.op = OP_ALL_LIGHTS_OFF;
      c.unit = 0;
      c.repeat = 1;

    } else {

      if (!parse_address(tok1, c.house, c.unit)) {
        publish_event("ERROR BAD_ADDRESS");
        return false;
      }

      if (strcmp(tok2, "ON") == 0) {
        c.op = OP_ON;
        c.repeat = 1;

      } else if (strcmp(tok2, "OFF") == 0) {
        c.op = OP_OFF;
        c.repeat = 1;

      } else if (strcmp(tok2, "DIM") == 0) {
        c.op = OP_DIM;
        c.repeat = parse_repeat(tok3);

      } else if (strcmp(tok2, "BRIGHT") == 0) {
        c.op = OP_BRIGHT;
        c.repeat = parse_repeat(tok3);

      } else {
        publish_event("ERROR BAD_FUNCTION");
        return false;
      }
    }

    if (queue_count_ >= QUEUE_SIZE) {
      publish_event("ERROR QUEUE_FULL");
      return false;
    }

    queue_[queue_tail_] = c;
    queue_tail_ = (queue_tail_ + 1) % QUEUE_SIZE;
    queue_count_++;

    pending_stats_ = true;

    char msg[64];
    format_command(c, msg, sizeof(msg));

    char out[80];
    snprintf(out, sizeof(out), "QUEUED %s Q=%u", msg, queue_count_);
    publish_event(out);

    return true;
  }

 private:

  /*
   * Standard X10 house codes.
   * Bit order is H1 H2 H4 H8.
   */
  static constexpr uint8_t HOUSE[16] = {
    0b0110, // A
    0b1110, // B
    0b0010, // C
    0b1010, // D
    0b0001, // E
    0b1001, // F
    0b0101, // G
    0b1101, // H
    0b0111, // I
    0b1111, // J
    0b0011, // K
    0b1011, // L
    0b0000, // M
    0b1000, // N
    0b0100, // O
    0b1100  // P
  };

  /*
   * X10 unit/address codes.
   *
   * Bit order is D1 D2 D4 D8 D16.
   * The final D16 bit is zero for an address.
   */
  static constexpr uint8_t UNIT[16] = {
    0b01100, // 1
    0b11100, // 2
    0b00100, // 3
    0b10100, // 4
    0b00010, // 5
    0b10010, // 6
    0b01010, // 7
    0b11010, // 8
    0b01110, // 9
    0b11110, // 10
    0b00110, // 11
    0b10110, // 12
    0b00000, // 13
    0b10000, // 14
    0b01000, // 15
    0b11000  // 16
  };

  /*
   * X10 function codes.
   * Final D16 bit is one for a function.
   */
  static constexpr uint8_t FUNC_ALL_UNITS_OFF  = 0b00001;
  static constexpr uint8_t FUNC_ALL_LIGHTS_ON  = 0b00011;
  static constexpr uint8_t FUNC_ON             = 0b00101;
  static constexpr uint8_t FUNC_OFF            = 0b00111;
  static constexpr uint8_t FUNC_DIM            = 0b01001;
  static constexpr uint8_t FUNC_BRIGHT         = 0b01011;
  static constexpr uint8_t FUNC_ALL_LIGHTS_OFF = 0b01101;

  /*
   * ISR state.
   */
  static X10PSC05 *instance_;
  static hw_timer_t *tx_timer_;
  static hw_timer_t *rx_timer_;

  volatile uint32_t half_cycle_count_{0};
  volatile uint32_t last_carrier_half_{0};
  volatile bool carrier_seen_{false};

  /*
   * One active-low RX level is sampled RX_SAMPLE_US after each
   * zero crossing.  Samples are queued chronologically in a ring buffer for the
   * main-loop decoder.
   */
  /*
   * Single-producer / single-consumer RX ring buffer.
   *
   * rx_sample_isr() is the only writer of rx_head_.
   * process_rx_events() is the only writer of rx_tail_.
   *
   * One slot is intentionally left unused so head == tail always
   * means empty.  If the loop ever falls behind, the newest sample
   * is dropped and rx_overflow_count_ is incremented.
   */
  volatile uint8_t rx_pending_bits_[RX_QUEUE_SIZE]{};
  volatile uint8_t rx_head_{0};
  volatile uint8_t rx_tail_{0};
  volatile uint32_t rx_overflow_count_{0};

  volatile bool tx_high_{false};

  /*
   * Main-loop queue/state.
   */
  Command queue_[QUEUE_SIZE]{};

  uint8_t queue_head_{0};
  uint8_t queue_tail_{0};
  uint8_t queue_count_{0};

  Command current_{};
  bool current_valid_{false};

  TxState tx_state_{TX_IDLE};

  uint8_t tx_wave_[MAX_WAVE_BITS]{};
  uint16_t tx_wave_len_{0};

  /*
   * Codes that PSC05 should return while verifying our transmission.
   */
  uint16_t tx_expected_[MAX_RX_CODES]{};
  uint8_t expected_count_{0};
  uint8_t expected_rx_index_{0};

  volatile bool tx_collision_{false};
  bool tx_seen_rx_{false};

  uint32_t tx_deadline_ms_{0};

  /*
   * Last validated PSC05 RX code.
   */
  uint32_t last_rx_ms_{0};

  /*
   * Backoff is measured in zero-cross half-cycle boundaries.
   */
  uint16_t backoff_half_cycles_{0};
  uint32_t backoff_reference_half_{0};

  /*
   * Retry state is independent from DIM/BRIGHT repeat count.
   */
  uint8_t retry_count_{0};

  /*
   * RX decoder.
   *
   * IMPORTANT:
   *
   * The PSC05 presents the Start Code to the OEM.  The Start Code
   * is 1110 and deliberately violates the normal true/complement
   * data-bit rule.
   *
   * After the four Start half-cycles, the PSC05 presents nine data
   * bits (4 house + 5 key), with each data bit represented by its
   * true value followed chronologically by its complement.
   *
   * Therefore the complete received envelope occupies:
   *
   *     4 start half-cycles
   *   + 18 data half-cycles
   *   = 22 half-cycles
   *
   * We decode the stream chronologically.  We do NOT split it into
   * alternating phase streams because the true/complement pair
   * depends on the order of adjacent half-cycles.
   */
  enum RxState : uint8_t {
    RX_SEARCH_START,
    RX_DATA_TRUE,
    RX_DATA_COMPLEMENT
  };

  RxState rx_state_{RX_SEARCH_START};

  uint8_t rx_start_shift_{0};
  uint8_t rx_start_count_{0};

  uint8_t rx_data_count_{0};
  uint8_t rx_data_value_{0};
  uint16_t rx_decoded_code_{0};
  uint8_t rx_diag_frame_[22]{0};
  uint8_t rx_diag_pos_{0};


  /*
   * Internal representation of a received PSC05 code contains only
   * the 9 data bits:
   *
   *     house[4] << 5 | key[5]
   *
   * The Start Code is framing, not part of the value compared
   * against expected PSC05 RX codes.
   */

  /*
   * Statistics.
   */
  uint32_t tx_ok_{0};
  uint32_t tx_fail_{0};
  uint32_t collisions_{0};
  uint32_t rx_codes_{0};
  uint32_t retries_{0};

  bool pending_stats_{false};

  std::function<void(const std::string &)> event_cb_;
  std::function<void(const std::string &)> stats_cb_;

  /*
   * Zero-crossing ISR.
   *
   * Every transition of the PSC05 zero-cross output represents
   * one 60-Hz half-cycle boundary.
   */
  static void IRAM_ATTR zc_isr();

  /*
   * Sample PSC05 RX as a LEVEL 600 us after ZERO CROSS.
   *
   * PSC05 RX is open-collector and active low:
   *
   *     LOW  = carrier/envelope present = X10 half-cycle value 1
   *     HIGH = no carrier/envelope      = X10 half-cycle value 0
   *
   * This ISR does no X10 decoding.  It only queues one chronological
   * sample for the main loop.
   */
  static void IRAM_ATTR rx_sample_isr();

  /*
   * One-shot TX timer terminates each 1 ms PSC05 TX envelope.
   */
  static void IRAM_ATTR tx_timer_isr();

  static bool parse_house(const char *s, uint8_t &house) {
    if (!s || !s[0]) {
      return false;
    }

    char c = s[0];

    if (c >= 'A' && c <= 'P') {
      house = (uint8_t)(c - 'A');
      return true;
    }

    return false;
  }

  static bool parse_address(
      const char *s,
      uint8_t &house,
      uint8_t &unit) {

    if (!s || strlen(s) < 2) {
      return false;
    }

    if (!parse_house(s, house)) {
      return false;
    }

    char *end = nullptr;
    long u = strtol(s + 1, &end, 10);

    if (*end != '\0' || u < 1 || u > 16) {
      return false;
    }

    unit = (uint8_t)u;
    return true;
  }

  static uint8_t parse_repeat(const char *s) {
    if (!s || !s[0]) {
      return 1;
    }

    long n = strtol(s, nullptr, 10);

    if (n < 1) {
      n = 1;
    }

    if (n > 16) {
      n = 16;
    }

    return (uint8_t)n;
  }

  static void format_command(
      const Command &c,
      char *out,
      size_t len) {

    char h = (char)('A' + c.house);

    switch (c.op) {

      case OP_ALL_UNITS_OFF:
        snprintf(out, len, "%c ALL_OFF", h);
        break;

      case OP_ALL_LIGHTS_ON:
        snprintf(out, len, "%c ALL_ON", h);
        break;

      case OP_ALL_LIGHTS_OFF:
        snprintf(out, len, "%c ALL_LIGHTS_OFF", h);
        break;

      case OP_ON:
        snprintf(out, len, "%c%u ON", h, c.unit);
        break;

      case OP_OFF:
        snprintf(out, len, "%c%u OFF", h, c.unit);
        break;

      case OP_DIM:
        snprintf(out, len, "%c%u DIM %u",
                 h, c.unit, c.repeat);
        break;

      case OP_BRIGHT:
        snprintf(out, len, "%c%u BRIGHT %u",
                 h, c.unit, c.repeat);
        break;
    }
  }

  /*
   * Internal PSC05 RX representation.
   *
   * The Start Code is framing and is not included in the value
   * returned by the RX decoder.  Only the 9 data bits are retained:
   *
   *     house (4 bits) + key/function (5 bits)
   */
  static uint16_t make_code(
      uint8_t house,
      uint8_t key) {

    return (uint16_t)(
        ((HOUSE[house] & 0x0F) << 5) |
        (key & 0x1F));
  }

  /*
   * Append one complete X10 code.
   *
   * 1110                  = 4 half-cycles
   * house, complemented  = 8 half-cycles
   * key, complemented    = 10 half-cycles
   *
   * Total = 22 half-cycles.
   */
  void append_half(uint8_t bit) {
    if (tx_wave_len_ < MAX_WAVE_BITS) {
      tx_wave_[tx_wave_len_++] = bit ? 1U : 0U;
    }
  }

  void append_code(
      uint8_t house,
      uint8_t key) {
    /*
     * X10 chronological wire order:
     *
     *   START: 1 1 1 0
     *   HOUSE: H1,/H1 H2,/H2 H4,/H4 H8,/H8
     *   DATA : D1,/D1 D2,/D2 D4,/D4 D8,/D8 D16,/D16
     *
     * The house and key values are stored conventionally as numeric
     * bit fields, therefore bit 0 is transmitted first.
     */

    append_half(1);
    append_half(1);
    append_half(1);
    append_half(0);

    const uint8_t hb = HOUSE[house] & 0x0F;

    for (int8_t i = 3; i >= 0; --i) {
      const uint8_t b = (hb >> i) & 1U;
      append_half(b);
      append_half(b ^ 1U);
    }

    /*
     * PSC05 chronological data order demonstrated on hardware:
     * D8, D4, D2, D1, D16 = stored key bits 4,3,2,1,0.
     */
    for (int8_t i = 4; i >= 0; --i) {
      const uint8_t b = (key >> i) & 1U;
      append_half(b);
      append_half(b ^ 1U);
    }
  }

  /*
   * Normal X10 code pair:
   *
   *   code
   *   code
   *   3-cycle gap
   *
   * At 60 Hz, 3 cycles = 6 half-cycles.
   */
  void append_pair(
      uint8_t house,
      uint8_t key) {

    append_code(house, key);
    append_code(house, key);

    for (uint8_t i = 0; i < 6; i++) {
      if (tx_wave_len_ < MAX_WAVE_BITS) {
        tx_wave_[tx_wave_len_++] = 0;
      }
    }
  }

  /*
   * Build the exact waveform and the sequence of RX codes expected
   * back from the PSC05.
   */
  bool build_wave(const Command &c) {

    tx_wave_len_ = 0;
    expected_count_ = 0;
    expected_rx_index_ = 0;

    /*
     * Broadcast commands consist of a single function pair.
     */
    if (c.op == OP_ALL_UNITS_OFF ||
        c.op == OP_ALL_LIGHTS_ON ||
        c.op == OP_ALL_LIGHTS_OFF) {

      uint8_t fn = function_code(c.op);

      append_pair(c.house, fn);

      tx_expected_[expected_count_++] =
          make_code(c.house, fn);

      return tx_wave_len_ > 0;
    }

    /*
     * First transmit the unit address pair.
     */
    append_pair(
        c.house,
        UNIT[c.unit - 1]);

    if (expected_count_ < MAX_RX_CODES) {
      tx_expected_[expected_count_++] =
          make_code(
              c.house,
              UNIT[c.unit - 1]);
    }

    /*
     * DIM/BRIGHT are special:
     *
     * X10 sends consecutive function codes rather than normal
     * code-pairs with a 3-cycle gap.
     *
     * PSC05 RX behavior is also special: it returns the first
     * DIM/BRIGHT code and then every third code.
     */
    if (c.op == OP_DIM ||
        c.op == OP_BRIGHT) {

      uint8_t fn =
          function_code(c.op);

      uint8_t count = c.repeat;

      if (count < 2) {
        count = 2;
      }

      if (count > 16) {
        count = 16;
      }

      for (uint8_t i = 0; i < count; i++) {

        append_code(c.house, fn);

        /*
         * PSC05 RX sequence:
         *   code 0
         *   code 3
         *   code 6
         *   code 9
         *   ...
         */
        if ((i == 0 || (i % 3) == 0) &&
            expected_count_ < MAX_RX_CODES) {

          tx_expected_[expected_count_++] =
              make_code(c.house, fn);
        }
      }

      /*
       * End with a 3-cycle gap.
       */
      for (uint8_t i = 0; i < 6; i++) {
        if (tx_wave_len_ < MAX_WAVE_BITS) {
          tx_wave_[tx_wave_len_++] = 0;
        }
      }

    } else {

      /*
       * ON/OFF:
       * address pair followed by function pair.
       */
      uint8_t fn =
          function_code(c.op);

      append_pair(c.house, fn);

      if (expected_count_ < MAX_RX_CODES) {
        tx_expected_[expected_count_++] =
            make_code(c.house, fn);
      }
    }

    return tx_wave_len_ > 0 &&
           tx_wave_len_ < MAX_WAVE_BITS;
  }

  static uint8_t function_code(Op op) {

    switch (op) {

      case OP_ON:
        return FUNC_ON;

      case OP_OFF:
        return FUNC_OFF;

      case OP_DIM:
        return FUNC_DIM;

      case OP_BRIGHT:
        return FUNC_BRIGHT;

      case OP_ALL_UNITS_OFF:
        return FUNC_ALL_UNITS_OFF;

      case OP_ALL_LIGHTS_ON:
        return FUNC_ALL_LIGHTS_ON;

      case OP_ALL_LIGHTS_OFF:
        return FUNC_ALL_LIGHTS_OFF;
    }

    return FUNC_OFF;
  }

  /*
   * Move ISR RX events into local memory so the rest of the
   * decoder never manipulates ISR-owned arrays.
   */
  void process_rx_events() {
    /*
     * Drain samples in exactly the order produced by rx_sample_isr().
     * No array shifting or shared count is required.
     */
    while (rx_tail_ != rx_head_) {
      const uint8_t tail = rx_tail_;
      const uint8_t sample = rx_pending_bits_[tail];

      rx_tail_ =
          static_cast<uint8_t>((tail + 1U) % RX_QUEUE_SIZE);

      process_rx_half(sample != 0);
    }
  }

  /*
   * Decode one chronological PSC05 RX half-cycle.
   *
   * Search:
   *
   *     1 1 1 0
   *
   * The fourth zero is the end of the Start Code and the beginning
   * of the data stream.
   *
   * The following 18 half-cycles are nine true/complement pairs.
   *
   *     1 0 -> logical 1
   *     0 1 -> logical 0
   *
   *     1 1 or 0 0 -> invalid pair
   */
  void reset_rx_decoder() {
    rx_state_ = RX_SEARCH_START;
    rx_start_shift_ = 0;
    rx_start_count_ = 0;
    rx_data_count_ = 0;
    rx_data_value_ = 0;
    rx_decoded_code_ = 0;
  }

  /*
   * A malformed true/complement pair has two meanings:
   *
   *   RX only:
   *     discard the partial receive frame and search for 1110.
   *
   *   While TX_TRANSMIT:
   *     the line no longer contains a legal copy of the frame we
   *     are actively placing on it.  Treat that as a collision,
   *     stop driving TX immediately, and let finish_tx(false)
   *     perform the normal retry/backoff path.
   *
   * TX_WAIT_RX is deliberately NOT considered active monitoring:
   * unrelated malformed traffic after our waveform has completed
   * must not be attributed to our transmission.
   */
  void handle_bad_rx_pair() {
    reset_rx_decoder();

    if (tx_state_ == TX_TRANSMIT) {
      tx_collision_ = true;
      collisions_++;
      pending_stats_ = true;

      digitalWrite(TX_PIN, TX_IDLE_LEVEL);
      tx_high_ = false;

      if (tx_timer_) {
        timerAlarmDisable(tx_timer_);
      }

      /*
       * Prevent any additional waveform bits from being emitted
       * before the main-loop TX state machine observes collision.
       */
      tx_pos_ = tx_len_;
    }
  }

  void process_rx_half(bool bit) {

    switch (rx_state_) {

      case RX_SEARCH_START: {

        rx_start_shift_ =
            (uint8_t)(((rx_start_shift_ << 1) |
                       (bit ? 1U : 0U)) & 0x0F);

        if (rx_start_count_ < 4) {
          rx_start_count_++;
        }

        if (rx_start_count_ >= 4 &&
            rx_start_shift_ == 0b1110) {

          /*
           * The fourth half-cycle was the Start Code's zero.
           * The next half-cycle is the true half of the first
           * house bit.
           */
          rx_state_ = RX_DATA_TRUE;
          rx_data_count_ = 0;
          rx_data_value_ = 0;
          rx_decoded_code_ = 0;

          rx_diag_frame_[0] = 1;
          rx_diag_frame_[1] = 1;
          rx_diag_frame_[2] = 1;
          rx_diag_frame_[3] = 0;
          rx_diag_pos_ = 4;
        }

        break;
      }

      case RX_DATA_TRUE:

        if (rx_diag_pos_ < 22) {
          rx_diag_frame_[rx_diag_pos_++] = bit ? 1U : 0U;
        }

        rx_data_value_ = bit ? 1U : 0U;
        rx_state_ = RX_DATA_COMPLEMENT;
        break;

      case RX_DATA_COMPLEMENT: {

        if (rx_diag_pos_ < 22) {
          rx_diag_frame_[rx_diag_pos_++] = bit ? 1U : 0U;
        }

        const bool complement =
            !rx_data_value_;

        if (bit != complement) {
          handle_bad_rx_pair();
          break;
        }

        /*
         * Shift the recovered logical data bit into the 9-bit
         * house+key value.
         */
        rx_data_value_ =
            (uint8_t)(rx_data_value_ & 1U);

        /*
         * Chronological PSC05/X10 wire order is:
         *
         *   data_count 0..3 : H8, H4, H2, H1
         *   data_count 4..8 : D8, D4, D2, D1, D16
         *
         * Our internal packed representation remains:
         *
         *   bits 8..5 = house numeric value
         *   bits 4..0 = key/data numeric value
         *
         * Therefore place each recovered bit directly into its
         * final packed position rather than shifting the stream.
         */
        if (rx_data_value_) {
          if (rx_data_count_ < 4) {
            /*
             * Chronological house order: H8,H4,H2,H1.
             * Store into packed house bits 8,7,6,5.
             */
            rx_decoded_code_ |=
                (uint16_t)(1U << (8U - rx_data_count_));
          } else {
            /*
             * Chronological data order: D8,D4,D2,D1,D16.
             * Store into packed key bits 4,3,2,1,0.
             */
            rx_decoded_code_ |=
                (uint16_t)(1U << (8U - rx_data_count_));
          }
        }

        rx_data_count_++;

        if (rx_data_count_ >= 9) {

          /*
           * Complete PSC05 code:
           *
           *   bits 8..5 = house
           *   bits 4..0 = key/function
           */
          const uint16_t code =
              rx_decoded_code_ & 0x01FFU;

          const uint8_t house_raw =
              (uint8_t)((code >> 5) & 0x0FU);

          const uint8_t key =
              (uint8_t)(code & 0x1FU);

          if (valid_house(house_raw) &&
              valid_key(key)) {

            handle_received_code(code);

          } else {

            /*
             * The PSC05 normally provides validated traffic, but
             * retain this sanity check so malformed timing cannot
             * become an RX event.
             */
            pending_stats_ = true;
          }

          /*
           * Look for the next Start Code.  This also handles the
           * consecutive DIM/BRIGHT codes correctly because there is
           * no three-cycle gap between them.
           */
          reset_rx_decoder();
        } else {
          rx_state_ = RX_DATA_TRUE;
        }

        break;
      }
    }
  }

  static bool valid_house(uint8_t raw) {

    for (uint8_t i = 0; i < 16; i++) {
      if (HOUSE[i] == raw) {
        return true;
      }
    }

    return false;
  }

  static bool valid_key(uint8_t key) {

    /*
     * Unit/address code.
     */
    if ((key & 1U) == 0) {

      for (uint8_t i = 0; i < 16; i++) {

        if (UNIT[i] == key) {
          return true;
        }
      }

      return false;
    }

    /*
     * Function code.
     */
    switch (key) {

      case FUNC_ALL_UNITS_OFF:
      case FUNC_ALL_LIGHTS_ON:
      case FUNC_ON:
      case FUNC_OFF:
      case FUNC_DIM:
      case FUNC_BRIGHT:
      case FUNC_ALL_LIGHTS_OFF:
        return true;

      default:
        return false;
    }
  }

  void publish_rx_diagnostic(uint16_t code) {
    char raw[23];
    for (uint8_t i = 0; i < 22; ++i) {
      raw[i] = rx_diag_frame_[i] ? '1' : '0';
    }
    raw[22] = '\0';

    char msg[128];
    const uint8_t house_raw = (uint8_t)((code >> 5) & 0x0F);
    const uint8_t key = (uint8_t)(code & 0x1F);

    snprintf(msg, sizeof(msg),
             "RAW %s HOUSE=%u%u%u%u DATA=%u%u%u%u%u H=0x%X D=0x%02X",
             raw,
             (house_raw >> 3) & 1U, (house_raw >> 2) & 1U,
             (house_raw >> 1) & 1U, house_raw & 1U,
             (key >> 4) & 1U, (key >> 3) & 1U,
             (key >> 2) & 1U, (key >> 1) & 1U, key & 1U,
             house_raw, key);
    publish_event(msg);
  }

  /*
   * Handle one complete validated PSC05 RX code.
   */
  void handle_received_code(uint16_t code) {

    rx_codes_++;
    last_rx_ms_ = millis();
    pending_stats_ = true;

    publish_rx_diagnostic(code);

    /*
     * While the waveform is physically being transmitted, the
     * PSC05 return must follow our expected sequence.  A complete,
     * syntactically valid X10 code that is not the next expected
     * echo is therefore a collision, just like a malformed
     * true/complement pair.
     */
    if (tx_state_ == TX_TRANSMIT) {

      tx_seen_rx_ = true;

      if (expected_rx_index_ >= expected_count_ ||
          code != tx_expected_[expected_rx_index_]) {

        tx_collision_ = true;
        collisions_++;
        pending_stats_ = true;

        /*
         * Stop driving immediately.  The zero-cross ISR also checks
         * tx_collision_ as a hard inhibit, so no later half-cycle can
         * reassert TX before the main loop enters retry/backoff.
         */
        digitalWrite(TX_PIN, TX_IDLE_LEVEL);
        tx_high_ = false;

        if (tx_timer_) {
          timerAlarmDisable(tx_timer_);
        }

        tx_pos_ = tx_len_;
        return;
      }

      expected_rx_index_++;
      return;
    }

    /*
     * Once our physical waveform is complete, a different valid
     * frame is not automatically attributable to our transmission.
     * Do not label it a collision.  A matching late PSC05 return may
     * still complete verification; otherwise our existing deadline
     * will produce TX_VERIFY_FAIL and retry normally.
     */
    if (tx_state_ == TX_WAIT_RX) {

      tx_seen_rx_ = true;

      if (expected_rx_index_ < expected_count_ &&
          code == tx_expected_[expected_rx_index_]) {
        expected_rx_index_++;
      } else {
        /*
         * Preserve visibility of legitimate external traffic without
         * allowing it to advance our verification sequence.
         */
        publish_received(code);
      }

      return;
    }

    /*
     * RX traffic while idle/backing off is external traffic.
     */
    publish_received(code);
  }

  void publish_received(uint16_t code) {

    uint8_t house_raw =
        (uint8_t)((code >> 5) & 0x0F);

    uint8_t key =
        (uint8_t)(code & 0x1F);

    uint8_t house_index =
        decode_house(house_raw);

    char h =
        (char)('A' + house_index);

    char msg[80];

    if ((key & 1U) == 0) {

      int unit =
          decode_unit(key);

      if (unit > 0) {

        snprintf(
            msg,
            sizeof(msg),
            "RX %c%u",
            h,
            unit);

      } else {

        snprintf(
            msg,
            sizeof(msg),
            "RX %c ADDRESS",
            h);
      }

    } else {

      const char *fn =
          decode_function(key);

      snprintf(
          msg,
          sizeof(msg),
          "RX %c %s",
          h,
          fn ? fn : "FUNCTION");
    }

    publish_event(msg);
  }

  static uint8_t decode_house(
      uint8_t raw) {

    for (uint8_t i = 0; i < 16; i++) {

      if (HOUSE[i] == raw) {
        return i;
      }
    }

    return 0;
  }

  static int decode_unit(
      uint8_t raw) {

    for (uint8_t i = 0; i < 16; i++) {

      if (UNIT[i] == raw) {
        return i + 1;
      }
    }

    return -1;
  }

  static const char *decode_function(
      uint8_t key) {

    switch (key) {

      case FUNC_ALL_UNITS_OFF:
        return "ALL_OFF";

      case FUNC_ALL_LIGHTS_ON:
        return "ALL_ON";

      case FUNC_ON:
        return "ON";

      case FUNC_OFF:
        return "OFF";

      case FUNC_DIM:
        return "DIM";

      case FUNC_BRIGHT:
        return "BRIGHT";

      case FUNC_ALL_LIGHTS_OFF:
        return "ALL_LIGHTS_OFF";

      default:
        return nullptr;
    }
  }

  /*
   * The PSC05 gives us validated RX traffic, so this is a
   * conservative "quiet line" test rather than raw carrier
   * detection.
   */
  bool line_free() const {

    /*
     * Any sampled 1 proves carrier activity.  A sampled 0 does not
     * prove idle, because zero half-cycles are part of valid X10.
     * Require three complete AC cycles (six half-cycles) with no
     * observed carrier before declaring the line free.
     */
    static constexpr uint32_t QUIET_HALF_CYCLES = 6U;

    if (!carrier_seen_) {
      return true;
    }

    const uint32_t quiet_half_cycles =
        (uint32_t)(half_cycle_count_ - last_carrier_half_);

    return quiet_half_cycles >= QUIET_HALF_CYCLES;
  }

  /*
   * Start the current command after a random X10 half-cycle
   * backoff.
   */
  void wait_for_quiet() {
    /*
     * Do not choose a random delay while the bus is still busy.
     * Once line_free() becomes true, TX_WAIT_QUIET transitions to
     * TX_BACKOFF and chooses exactly one random contention delay.
     */
    tx_state_ = TX_WAIT_QUIET;
    pending_stats_ = true;
  }

  void begin_backoff() {
    /*
     * Randomized collision-avoidance delay measured from the
     * current zero-cross half-cycle count.
     *
     * random(0, 9) returns 0..8, therefore this is 2..10
     * half-cycles inclusive.
     */
    backoff_half_cycles_ =
        (uint16_t)(2 + random(0, 9));

    backoff_reference_half_ =
        half_cycle_count_;

    tx_state_ = TX_BACKOFF;
    pending_stats_ = true;
  }

  bool start_transmission() {

    if (!build_wave(current_)) {

      publish_event(
          "ERROR WAVEFORM_TOO_LARGE");

      return false;
    }

    tx_pos_ = 0;
    tx_len_ = tx_wave_len_;

    expected_rx_index_ = 0;

    tx_collision_ = false;
    tx_seen_rx_ = false;

    /*
     * Calculate a timeout from the actual waveform size.
     *
     * Add generous margin for PSC05 RX verification.
     */
    uint32_t tx_time_ms =
        (uint32_t)tx_len_ *
        HALF_CYCLE_MS;

    tx_deadline_ms_ =
        millis() +
        tx_time_ms +
        1500U;

    tx_state_ =
        TX_TRANSMIT;

    publish_event(
        "TX_START");

    return true;
  }

  /*
   * Main TX state machine.
   */
  void service_tx_state() {

    switch (tx_state_) {

      case TX_IDLE:

        /*
         * Nothing currently executing.
         */
        if (queue_count_ == 0) {
          return;
        }

        /*
         * Take ownership of the next queued command immediately.
         * Arbitration itself is handled by TX_WAIT_QUIET.
         */
        current_ =
            queue_[queue_head_];

        queue_head_ =
            (queue_head_ + 1) %
            QUEUE_SIZE;

        queue_count_--;

        current_valid_ = true;

        retry_count_ = 0;

        /*
         * First establish a truly quiet bus; random backoff is chosen
         * only after the quiet guard has completed.
         */
        wait_for_quiet();

        break;

      case TX_WAIT_QUIET:

        if (!line_free()) {
          return;
        }

        /*
         * Bus has now satisfied the quiet guard.  Choose one random
         * contention delay and begin counting it down.
         */
        begin_backoff();
        break;

      case TX_BACKOFF:

        /*
         * Carrier reappeared during the random countdown.  Cancel
         * this countdown and wait for a complete quiet guard again.
         * A new random delay will be selected once, after quiet.
         */
        if (!line_free()) {
          wait_for_quiet();
          return;
        }

        while (backoff_half_cycles_ > 0) {

          uint32_t elapsed =
              (uint32_t)(
                  half_cycle_count_ -
                  backoff_reference_half_);

          if (elapsed == 0) {
            return;
          }

          uint16_t step =
              (elapsed >
               backoff_half_cycles_)
                  ? backoff_half_cycles_
                  : (uint16_t)elapsed;

          backoff_half_cycles_ -=
              step;

          backoff_reference_half_ +=
              step;
        }

        if (!start_transmission()) {

          current_valid_ = false;
          tx_state_ = TX_IDLE;
        }

        break;

      case TX_TRANSMIT:

        /*
         * Collision has priority over waveform completion.
         *
         * handle_bad_rx_pair() deliberately sets tx_pos_ = tx_len_
         * when aborting an active transmission.  Therefore we must
         * test tx_collision_ first; otherwise an aborted waveform
         * could momentarily be mistaken for a normal completion and
         * enter TX_WAIT_RX.
         */
        if (tx_collision_) {
          finish_tx(false);
          return;
        }

        /*
         * Only a non-collided waveform reaching its end is a normal
         * transmission completion.
         */
        if (tx_pos_ >= tx_len_) {

          digitalWrite(TX_PIN, TX_IDLE_LEVEL);

          tx_high_ = false;

          if (tx_timer_) {
            timerAlarmDisable(tx_timer_);
          }

          tx_state_ =
              TX_WAIT_RX;

          /*
           * For normal commands PSC05 should have returned
           * the expected codes during the transmission.
           * Give it a short final window to deliver the
           * last validated code.
           */
          return;
        }

        if ((int32_t)(
                millis() -
                tx_deadline_ms_) >= 0) {

          finish_tx(false);
        }

        break;

      case TX_WAIT_RX:

        if (tx_collision_) {

          finish_tx(false);
          return;
        }

        if (expected_rx_index_ >=
            expected_count_) {

          finish_tx(true);
          return;
        }

        if ((int32_t)(
                millis() -
                tx_deadline_ms_) >= 0) {

          finish_tx(false);
        }

        break;
    }
  }

  /*
   * Complete or retry the current command.
   */
  void finish_tx(bool success) {

    digitalWrite(TX_PIN, TX_IDLE_LEVEL);
    tx_high_ = false;

    if (tx_timer_) {
      timerAlarmDisable(tx_timer_);
    }

    char cmd[64];
    format_command(current_, cmd, sizeof(cmd));

    if (success &&
        !tx_collision_ &&
        expected_rx_index_ >= expected_count_) {

      tx_ok_++;

      publish_event(
          std::string("TX_OK ") + cmd);

      /*
       * Transaction complete.  Only now release current_ and
       * reset its retry history.
       */
      current_valid_ = false;
      retry_count_ = 0;
      tx_state_ = TX_IDLE;

    } else {

      tx_fail_++;

      if (tx_collision_) {
        publish_event(
            std::string("COLLISION ") + cmd);
      } else {
        publish_event(
            std::string("TX_VERIFY_FAIL ") + cmd);
      }

      if (retry_count_ < MAX_RETRIES) {

        retry_count_++;
        retries_++;

        char retry_msg[80];
        snprintf(
            retry_msg,
            sizeof(retry_msg),
            "RETRY %u/%u",
            retry_count_,
            MAX_RETRIES);

        publish_event(retry_msg);

        /*
         * IMPORTANT:
         *
         * current_ remains valid and is NOT returned to the queue.
         * The same transaction owns current_ until it succeeds or
         * exhausts MAX_RETRIES.  This prevents retry_count_ from
         * being reset by TX_IDLE and avoids needing queue space
         * merely to retry an in-progress transaction.
         *
         * start_transmission() will reset per-attempt state such as
         * tx_collision_ and expected_rx_index_ after this new
         * randomized backoff expires.
         */
        wait_for_quiet();

      } else {

        publish_event(
            std::string("TX_FAIL ") + cmd);

        current_valid_ = false;
        retry_count_ = 0;
        tx_state_ = TX_IDLE;
      }
    }

    pending_stats_ = true;
  }

  void publish_event(
      const std::string &s) {

    if (event_cb_) {
      event_cb_(s);
    }
  }

  void publish_stats() {

    char s[192];

    snprintf(
        s,
        sizeof(s),
        "queue=%u tx_ok=%lu tx_fail=%lu "
        "collisions=%lu retries=%lu rx_codes=%lu state=%u "
        "hc=%lu backoff=%u",
        queue_count_,
        (unsigned long)tx_ok_,
        (unsigned long)tx_fail_,
        (unsigned long)collisions_,
        (unsigned long)retries_,
        (unsigned long)rx_codes_,
        (unsigned)tx_state_,
        (unsigned long)half_cycle_count_,
        (unsigned)backoff_half_cycles_);

    if (stats_cb_) {
      stats_cb_(s);
    }
  }

  /*
   * TX ISR variables.
   */
  volatile uint16_t tx_pos_{0};
  volatile uint16_t tx_len_{0};
};

/* Out-of-class ISR definitions to avoid inline COMDAT IRAM literal placement. */
void IRAM_ATTR X10PSC05::zc_isr() {
    X10PSC05 *p = instance_;
    if (!p) {
      return;
    }

    /*
     * Every ZERO CROSS transition is one X10 half-cycle boundary.
     * Arm the RX timer so GPIO35 is sampled 600 us after this
     * boundary.  Figure 9 permits up to 200 us RX assertion delay
     * and guarantees at least 1 ms active time, putting 600 us
     * comfortably inside the valid window.
     */
    p->half_cycle_count_++;

    if (rx_timer_) {
      timerWrite(rx_timer_, 0);
      timerAlarmWrite(rx_timer_, RX_SAMPLE_US, false);
      timerAlarmEnable(rx_timer_);
    }

    /*
     * TX uses the same zero-cross clock.  For a logical one, assert
     * PSC05 TX at the boundary and use the independent TX timer to
     * remove it after 1 ms.  A logical zero leaves TX inactive.
     */
    if (p->tx_state_ == TX_TRANSMIT) {

      /*
       * Collision is a hard electrical inhibit.
       *
       * The RX decoder runs in the main loop and can latch a
       * collision between zero crossings.  Once latched, no later
       * zero-cross ISR is allowed to assert PSC05 TX again.  A new
       * attempt becomes eligible only after finish_tx(false) enters
       * backoff and start_transmission() explicitly clears
       * tx_collision_.
       */
      if (p->tx_collision_) {
        digitalWrite(TX_PIN, TX_IDLE_LEVEL);
        p->tx_high_ = false;

        if (tx_timer_) {
          timerAlarmDisable(tx_timer_);
        }

        return;
      }

      if (p->tx_pos_ < p->tx_len_) {
        const bool one =
            p->tx_wave_[p->tx_pos_++] != 0;

        if (one) {
          digitalWrite(TX_PIN, TX_ACTIVE_LEVEL);
          p->tx_high_ = true;

          if (tx_timer_) {
            timerWrite(tx_timer_, 0);
            timerAlarmWrite(tx_timer_, 1000, false);
            timerAlarmEnable(tx_timer_);
          }
        } else {
          digitalWrite(TX_PIN, TX_IDLE_LEVEL);
          p->tx_high_ = false;

          if (tx_timer_) {
            timerAlarmDisable(tx_timer_);
          }
        }
      } else {
        digitalWrite(TX_PIN, TX_IDLE_LEVEL);
        p->tx_high_ = false;

        if (tx_timer_) {
          timerAlarmDisable(tx_timer_);
        }
      }
    }
  }

void IRAM_ATTR X10PSC05::rx_sample_isr() {
    X10PSC05 *p = instance_;
    if (!p) {
      return;
    }

    const uint8_t bit =
        (digitalRead(RX_PIN) == LOW) ? 1U : 0U;

    if (bit != 0U) {
      p->last_carrier_half_ = p->half_cycle_count_;
      p->carrier_seen_ = true;
    }

    const uint8_t head = p->rx_head_;
    const uint8_t next =
        static_cast<uint8_t>((head + 1U) % RX_QUEUE_SIZE);

    if (next != p->rx_tail_) {
      p->rx_pending_bits_[head] = bit;
      p->rx_head_ = next;
    } else {
      // Main loop fell behind.  Preserve queued chronological data
      // and drop this newest sample rather than corrupting the frame.
      p->rx_overflow_count_++;
    }

    if (rx_timer_) {
      timerAlarmDisable(rx_timer_);
    }
  }

void IRAM_ATTR X10PSC05::tx_timer_isr() {
    X10PSC05 *p = instance_;
    if (!p) {
      return;
    }

    digitalWrite(TX_PIN, TX_IDLE_LEVEL);
    p->tx_high_ = false;

    if (tx_timer_) {
      timerAlarmDisable(tx_timer_);
    }
  }

/*
 * Static storage.
 *
 * This header is intended to be included by the ESPHome custom
 * component translation unit.  If your build includes this header
 * from multiple independent translation units, change these to
 * C++17 inline static members.
 */
constexpr uint8_t X10PSC05::HOUSE[16];
constexpr uint8_t X10PSC05::UNIT[16];

X10PSC05 *X10PSC05::instance_ = nullptr;
hw_timer_t *X10PSC05::tx_timer_ = nullptr;
hw_timer_t *X10PSC05::rx_timer_ = nullptr;

/*
 * Global component instance used by the ESPHome YAML.
 */
X10PSC05 x10_psc05;
