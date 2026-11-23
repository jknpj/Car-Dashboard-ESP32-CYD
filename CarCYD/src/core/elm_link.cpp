/**
 * @file elm_link.cpp
 * ELM327 protocol state machine. See elm_link.h for the design.
 */
#include "elm_link.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "dtc.h"
#include "log.h"
#include "signals.h"
#include "util.h"
#include "vehicle_state.h"

namespace core {
namespace {

// ---- Timing -------------------------------------------------------------------------
constexpr uint32_t kAtRespTimeoutMs = 4000;  // ATZ / banner can be slow
constexpr uint32_t kPidRespTimeoutMs = 2000; // first request also searches the protocol
constexpr uint32_t kMinGapMs = 25;           // pause between requests
constexpr uint8_t kMaxConsecutiveErrors = 6; // -> Error + reconnect

// Engine speed per km/h for gears 1..6 (index 0 = neutral). Generic table used
// by the demo simulator too; tune per vehicle if the gear estimate is off.
constexpr float kRpmPerKmh[] = {0.0f, 118.0f, 69.0f, 48.0f, 37.0f, 30.0f, 25.0f};
constexpr int kTopGear = 6;

// ---- ELM327 init sequence ------------------------------------------------------------
const char* const kAtSequence[] = {"ATZ", "ATE0", "ATL0", "ATH0", "ATS0", "ATSP0"};
constexpr uint8_t kAtCount = sizeof(kAtSequence) / sizeof(kAtSequence[0]);

// ---- PID poll table (weighted: RPM / speed / coolant repeat more often) ---------------
struct PidDef {
  uint8_t pid;
  SignalId signal;
  float (*convert)(uint32_t value, int n);
};

float cvt_rpm(uint32_t v, int) { return static_cast<float>(v) / 4.0f; }
float cvt_speed(uint32_t v, int) { return static_cast<float>(v & 0xFF); }
float cvt_temp(uint32_t v, int) { return static_cast<float>(v & 0xFF) - 40.0f; }
float cvt_voltage(uint32_t v, int) { return static_cast<float>(v) / 1000.0f; }
float cvt_pct(uint32_t v, int) { return static_cast<float>(v & 0xFF) * 100.0f / 255.0f; }
float cvt_map(uint32_t v, int) { return static_cast<float>(v & 0xFF); }
float cvt_maf(uint32_t v, int) { return static_cast<float>(v) / 100.0f; }
float cvt_timing(uint32_t v, int) { return static_cast<float>(v & 0xFF) / 2.0f - 64.0f; }
float cvt_fuel_rate(uint32_t v, int) { return static_cast<float>(v) / 20.0f; }
float cvt_trim(uint32_t v, int) { return static_cast<float>(v & 0xFF) * 100.0f / 128.0f - 100.0f; }
float cvt_runtime(uint32_t v, int) { return static_cast<float>(v); }

constexpr PidDef kPidTable[] = {
    {0x0C, SignalId::Rpm, cvt_rpm},
    {0x0D, SignalId::Speed, cvt_speed},
    {0x05, SignalId::CoolantTemp, cvt_temp},
    {0x0C, SignalId::Rpm, cvt_rpm},
    {0x0D, SignalId::Speed, cvt_speed},
    {0x04, SignalId::EngineLoad, cvt_pct},
    {0x0C, SignalId::Rpm, cvt_rpm},
    {0x0D, SignalId::Speed, cvt_speed},
    {0x11, SignalId::ThrottlePos, cvt_pct},
    {0x0F, SignalId::IntakeTemp, cvt_temp},
    {0x0C, SignalId::Rpm, cvt_rpm},
    {0x0D, SignalId::Speed, cvt_speed},
    {0x0B, SignalId::IntakePressure, cvt_map},
    {0x10, SignalId::Maf, cvt_maf},
    {0x0C, SignalId::Rpm, cvt_rpm},
    {0x0D, SignalId::Speed, cvt_speed},
    {0x0E, SignalId::TimingAdvance, cvt_timing},
    {0x5E, SignalId::FuelRate, cvt_fuel_rate},
    {0x06, SignalId::ShortFuelTrim, cvt_trim},
    {0x07, SignalId::LongFuelTrim, cvt_trim},
    {0x2F, SignalId::FuelLevel, cvt_pct},
    {0x42, SignalId::BatteryVoltage, cvt_voltage},
    {0x46, SignalId::AmbientTemp, cvt_temp},
    {0x1F, SignalId::RunTime, cvt_runtime},
    {0x5C, SignalId::OilTemp, cvt_temp},
    {0x01, SignalId::Count, nullptr},  // MIL status + DTC count, no signal
};
constexpr uint8_t kPidCount = sizeof(kPidTable) / sizeof(kPidTable[0]);

// ---- Parsing --------------------------------------------------------------------------
int hex_value(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

/** Decodes hex pairs, skipping non-hex characters. Returns the byte count. */
int parse_hex(const char* s, size_t len, uint8_t* out, int max_out) {
  int n = 0;
  for (size_t i = 0; i + 1 < len && n < max_out;) {
    const int hi = hex_value(s[i]);
    const int lo = hex_value(s[i + 1]);
    if (hi < 0 || lo < 0) {
      i++;
      continue;
    }
    out[n++] = static_cast<uint8_t>((hi << 4) | lo);
    i += 2;
  }
  return n;
}

/** True if the response is the ELM327 "no data / keep looking" chatter. */
bool is_chatter(const char* rx, size_t len) {
  (void)len;
  return strstr(rx, "NODATA") != nullptr || strstr(rx, "SEARCHING") != nullptr ||
         strstr(rx, "UNABLETOCONNECT") != nullptr || strstr(rx, "BUSINIT") != nullptr ||
         strstr(rx, "STOPPED") != nullptr || strstr(rx, "BUFFER") != nullptr ||
         strstr(rx, "CANERROR") != nullptr || strstr(rx, "?") != nullptr;
}

}  // namespace

// ---- Link lifecycle -------------------------------------------------------------------

void ElmLink::begin(VehicleState& state, uint32_t now_ms) {
  transport_.begin();
  phase_ = Phase::Connecting;
  waiting_ = false;
  rx_len_ = 0;
  errors_ = 0;
  state.simulated = false;
  state.link_state = LinkState::Connecting;
  state.clock_valid = false;
  state.invalidate_all();
  LOG_I("ELM", "link started over %s", transport_.name());
}

void ElmLink::end(VehicleState& state) {
  (void)state;
  waiting_ = false;
  phase_ = Phase::Idle;
  transport_.reset();
  LOG_I("ELM", "link stopped");
}

void ElmLink::fail_link(VehicleState& state) {
  waiting_ = false;
  rx_len_ = 0;
  errors_ = 0;
  state.link_state = LinkState::Error;
  transport_.reset();
  phase_ = Phase::Connecting;
  LOG_W("ELM", "protocol errors, reconnecting");
}

// ---- Polling --------------------------------------------------------------------------

void ElmLink::send(const char* cmd, uint32_t now_ms) {
  transport_.write(reinterpret_cast<const uint8_t*>(cmd), strlen(cmd));
  static const uint8_t kCr = '\r';
  transport_.write(&kCr, 1);
  waiting_ = true;
  sent_ms_ = now_ms;
  rx_len_ = 0;
}

void ElmLink::send_next_pid(uint32_t now_ms) {
  char cmd[8];
  const PidDef& def = kPidTable[pid_index_];
  pid_index_ = static_cast<uint8_t>((pid_index_ + 1) % kPidCount);
  snprintf(cmd, sizeof(cmd), "01%02X", def.pid);
  send(cmd, now_ms);
}

void ElmLink::poll(VehicleState& state, uint32_t now_ms) {
  transport_.service();

  // Drain whatever the adapter has sent; a '>' prompt closes the response.
  while (transport_.available() > 0 && rx_len_ < sizeof(rx_) - 1) {
    const int b = transport_.read();
    if (b < 0) break;
    if (b == '>') {
      rx_[rx_len_] = '\0';
      waiting_ = false;
      on_response(state, now_ms);
      break;  // one response per tick keeps the state machine simple
    }
    if (b != '\r' && b != '\n' && b != ' ') rx_[rx_len_++] = static_cast<uint8_t>(b);
  }

  // Response timeout.
  const uint32_t timeout = phase_ == Phase::AtInit ? kAtRespTimeoutMs : kPidRespTimeoutMs;
  if (waiting_ && elapsed(now_ms, sent_ms_) >= timeout) {
    waiting_ = false;
    rx_len_ = 0;
    on_timeout(state, now_ms);
  }

  // Drive the state machine forward when idle.
  switch (phase_) {
    case Phase::Connecting:
      if (transport_.connected()) {
        LOG_I("ELM", "adapter connected, initialising");
        at_step_ = 0;
        phase_ = Phase::AtInit;
        send(kAtSequence[at_step_], now_ms);
      }
      break;

    case Phase::Ready:
      if (!waiting_ && elapsed(now_ms, next_tx_ms_) >= kMinGapMs) {
        next_tx_ms_ = now_ms;
        send_next_pid(now_ms);
      }
      break;

    default:
      break;
  }
}

// ---- Responses ------------------------------------------------------------------------

void ElmLink::on_response(VehicleState& state, uint32_t now_ms) {
  const size_t len = rx_len_;
  rx_len_ = 0;

  switch (phase_) {
    case Phase::AtInit: {
      at_step_++;
      if (at_step_ < kAtCount) {
        send(kAtSequence[at_step_], now_ms);
      } else {
        phase_ = Phase::Ready;
        pid_index_ = 0;
        errors_ = 0;
        state.link_state = LinkState::Connected;
        LOG_I("ELM", "ready");
        next_tx_ms_ = now_ms + kMinGapMs;
      }
      break;
    }

    case Phase::Ready: {
      uint8_t data[16];
      const int n = parse_hex(rx_, len, data, static_cast<int>(sizeof(data)));
      const PidDef& def = kPidTable[(pid_index_ + kPidCount - 1) % kPidCount];
      if (def.pid == 0x01) {
        // Mode 01 PID 01: MIL bit and stored DTC count.
        if (n >= 3 && data[0] == 0x41 && data[1] == 0x01) {
          state.dtc.mil_on = (data[2] & 0x80) != 0;
          state.dtc.mil_known = true;
          state.dtc.revision++;
        }
      } else if (n >= 4 && data[0] == 0x41 && data[1] == def.pid) {
        uint32_t value = (static_cast<uint32_t>(data[2]) << 8) | data[3];
        if (n >= 5) value = (value << 8) | data[4];
        if (n >= 6) value = (value << 8) | data[5];
        state.set(def.signal, def.convert(value, n - 2), now_ms);
        errors_ = 0;
      } else if (is_chatter(rx_, len)) {
        // "NO DATA" etc.: leave the signal to go stale, keep polling.
      } else {
        LOG_W("ELM", "unexpected response '%.32s'", rx_);
        if (++errors_ >= kMaxConsecutiveErrors) fail_link(state);
      }
      update_gear(state, now_ms);
      break;
    }

    case Phase::DtcRead: {
      uint8_t data[80];
      const int n = parse_hex(rx_, len, data, static_cast<int>(sizeof(data)));
      switch (dtc_step_) {
        case 0:  // 01 01 -> MIL
          if (n >= 3 && data[0] == 0x41 && data[1] == 0x01) {
            state.dtc.mil_on = (data[2] & 0x80) != 0;
            state.dtc.mil_known = true;
          } else {
            dtc_had_error_ = true;
          }
          break;
        case 1:  // 43 -> stored
        case 2:  // 47 -> pending
        case 3:  // 4A -> permanent
          if (n >= 1 && data[0] == (dtc_step_ == 1 ? 0x43 : dtc_step_ == 2 ? 0x47 : 0x4A)) {
            uint8_t* dst = dtc_step_ == 1 ? dtc_stored_ : dtc_step_ == 2 ? dtc_pending_ : dtc_permanent_;
            const int count = n - 1 < static_cast<int>(kMaxDtcBytes) ? n - 1 : static_cast<int>(kMaxDtcBytes);
            memcpy(dst, data + 1, static_cast<size_t>(count));
            if (dtc_step_ == 1) dtc_stored_count_ = static_cast<uint8_t>(count);
            if (dtc_step_ == 2) dtc_pending_count_ = static_cast<uint8_t>(count);
            if (dtc_step_ == 3) dtc_permanent_count_ = static_cast<uint8_t>(count);
          } else {
            dtc_had_error_ = true;
          }
          break;
        default:
          break;
      }

      if (dtc_step_ == 3) {
        // Finalise: stored, pending, permanent, capped at kMaxCodes.
        uint8_t out = 0;
        const auto add_bucket = [&](const uint8_t* src, uint8_t count, DtcStatus status) {
          for (uint8_t i = 0; i + 1 < count && out < DtcState::kMaxCodes; i += 2) {
            state.dtc.codes[out++] = {static_cast<uint16_t>((src[i] << 8) | src[i + 1]), status};
          }
        };
        add_bucket(dtc_stored_, dtc_stored_count_, DtcStatus::Stored);
        add_bucket(dtc_pending_, dtc_pending_count_, DtcStatus::Pending);
        add_bucket(dtc_permanent_, dtc_permanent_count_, DtcStatus::Permanent);
        state.dtc.count = out;
        state.dtc.operation = DtcOperation::Idle;
        state.dtc.last_result = dtc_had_error_ ? DtcResult::Failed : DtcResult::ReadOk;
        state.dtc.scanned = true;
        state.dtc.last_scan_ms = now_ms;
        state.dtc.revision++;
        LOG_I("ELM", "DTC read done: %u code(s)%s", out, dtc_had_error_ ? " (with errors)" : "");
        phase_ = Phase::Ready;
        errors_ = 0;
      } else {
        dtc_step_++;
        send(dtc_step_ == 1 ? "03" : dtc_step_ == 2 ? "07" : "0A", now_ms);
      }
      break;
    }

    case Phase::DtcClear: {
      uint8_t data[4];
      const int n = parse_hex(rx_, len, data, static_cast<int>(sizeof(data)));
      const bool ok = n >= 1 && data[0] == 0x44;
      state.dtc.operation = DtcOperation::Idle;
      state.dtc.last_result = ok ? DtcResult::ClearOk : DtcResult::Failed;
      state.dtc.revision++;
      LOG_I("ELM", "DTC clear %s", ok ? "ok" : "failed");
      phase_ = Phase::Ready;
      errors_ = 0;
      break;
    }

    default:
      break;
  }
}

void ElmLink::on_timeout(VehicleState& state, uint32_t now_ms) {
  switch (phase_) {
    case Phase::AtInit:
      // Retry the same AT command once, then give up on the adapter.
      if (++errors_ >= kMaxConsecutiveErrors) {
        fail_link(state);
      } else {
        send(kAtSequence[at_step_], now_ms);
      }
      break;

    case Phase::Ready:
      if (++errors_ >= kMaxConsecutiveErrors) fail_link(state);
      break;

    case Phase::DtcRead: {
      dtc_had_error_ = true;
      if (dtc_step_ == 3) {
        on_response(state, now_ms);  // finalise with whatever we have
      } else {
        dtc_step_++;
        send(dtc_step_ == 1 ? "03" : dtc_step_ == 2 ? "07" : "0A", now_ms);
      }
      break;
    }

    case Phase::DtcClear:
      state.dtc.operation = DtcOperation::Idle;
      state.dtc.last_result = DtcResult::Failed;
      state.dtc.revision++;
      phase_ = Phase::Ready;
      errors_ = 0;
      break;

    default:
      break;
  }
}

// ---- Signal helpers -------------------------------------------------------------------

void ElmLink::update_gear(VehicleState& state, uint32_t now_ms) {
  const float speed = state.value_or(SignalId::Speed, now_ms, -1.0f);
  const float rpm = state.value_or(SignalId::Rpm, now_ms, 0.0f);
  if (speed < 2.5f || rpm < kEngineRunningRpm) {
    if (speed < 2.5f) state.set(SignalId::Gear, 0.0f, now_ms);
    return;
  }
  const float ratio = rpm / speed;
  int best = 0;
  float best_err = 1e9f;
  for (int g = 1; g <= kTopGear; ++g) {
    const float err = fabsf(ratio - kRpmPerKmh[g]);
    if (err < best_err) {
      best_err = err;
      best = g;
    }
  }
  // Reject nonsense ratios (clutch slip, coasting, wrong table).
  if (best_err <= 0.35f * ratio) state.set(SignalId::Gear, static_cast<float>(best), now_ms);
}

// ---- Diagnostics ----------------------------------------------------------------------

bool ElmLink::request_dtc_read(VehicleState& state, uint32_t now_ms) {
  if (phase_ != Phase::Ready) {
    state.dtc.last_result = DtcResult::NotConnected;
    state.dtc.revision++;
    return false;
  }
  phase_ = Phase::DtcRead;
  dtc_step_ = 0;
  dtc_had_error_ = false;
  dtc_stored_count_ = dtc_pending_count_ = dtc_permanent_count_ = 0;
  state.dtc.operation = DtcOperation::Reading;
  state.dtc.revision++;
  send("0101", now_ms);
  return true;
}

bool ElmLink::request_dtc_clear(VehicleState& state, uint32_t now_ms) {
  if (phase_ != Phase::Ready) {
    state.dtc.last_result = DtcResult::NotConnected;
    state.dtc.revision++;
    return false;
  }
  phase_ = Phase::DtcClear;
  state.dtc.operation = DtcOperation::Clearing;
  state.dtc.revision++;
  send("04", now_ms);
  return true;
}

}  // namespace core
