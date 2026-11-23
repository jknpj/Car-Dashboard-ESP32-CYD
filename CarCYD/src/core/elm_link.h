/**
 * @file elm_link.h
 * ELM327 OBD-II vehicle link.
 *
 * ElmLink implements core::DataLink for a car connected through an ELM327
 * adapter (mode 01 live data, modes 03/07/0A fault codes, mode 04 clear).
 * It speaks the ELM327 AT protocol over a byte-oriented transport
 * (core::ElmTransport), so the protocol logic stays portable and the PC
 * simulator can compile it. The Bluetooth SPP transport lives in
 * hal/bt_spp.h.
 *
 * Polling runs one PID at a time, round-robin with RPM / speed / coolant
 * weighted more often, which matches what a cheap ELM327 can sustain.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "data_link.h"

namespace core {

/** Two bytes per DTC, up to DtcState::kMaxCodes codes per bucket. */
constexpr uint8_t kMaxDtcBytes = 48;

/** Byte-oriented transport to an ELM327 adapter (BT SPP, UART, ...). */
class ElmTransport {
 public:
  virtual ~ElmTransport() = default;

  /** Initialise the transport (radio on, etc.). Called once from begin(). */
  virtual void begin() = 0;

  /** True while the link to the adapter is established. */
  virtual bool connected() = 0;

  /** Drive connection attempts / device discovery. Call from poll(). */
  virtual void service() = 0;

  /** Bytes waiting to be read. */
  virtual int available() = 0;

  /** Read one byte; -1 if none. */
  virtual int read() = 0;

  /** Send raw bytes. */
  virtual void write(const uint8_t* data, size_t len) = 0;

  /** Drop the connection and go back to (re)connecting. */
  virtual void reset() = 0;

  virtual const char* name() const = 0;
};

class ElmLink final : public DataLink {
 public:
  explicit ElmLink(ElmTransport& transport) : transport_(transport) {}

  const char* name() const override { return "elm327"; }
  void begin(VehicleState& state, uint32_t now_ms) override;
  void end(VehicleState& state) override;
  void poll(VehicleState& state, uint32_t now_ms) override;
  bool request_dtc_read(VehicleState& state, uint32_t now_ms) override;
  bool request_dtc_clear(VehicleState& state, uint32_t now_ms) override;

 private:
  enum class Phase : uint8_t { Idle, Connecting, AtInit, Ready, DtcRead, DtcClear };

  void send(const char* cmd, uint32_t now_ms);
  void on_response(VehicleState& state, uint32_t now_ms);
  void on_timeout(VehicleState& state, uint32_t now_ms);
  void apply_pid(VehicleState& state, uint32_t now_ms);
  void update_gear(VehicleState& state, uint32_t now_ms);
  void fail_link(VehicleState& state);
  void send_next_pid(uint32_t now_ms);

  ElmTransport& transport_;

  Phase phase_ = Phase::Idle;
  bool waiting_ = false;
  char rx_[256];          ///< response bytes, non-hex characters stripped
  uint16_t rx_len_ = 0;   ///< hex characters only
  uint32_t sent_ms_ = 0;  ///< when the current request went out
  uint32_t next_tx_ms_ = 0;

  uint8_t at_step_ = 0;   ///< index into the AT init sequence
  uint8_t pid_index_ = 0; ///< index into the weighted PID poll table
  uint8_t errors_ = 0;    ///< consecutive transport/protocol errors

  // DTC read (mode 03/07/0A) accumulation.
  uint8_t dtc_step_ = 0;
  bool dtc_had_error_ = false;
  uint8_t dtc_stored_[kMaxDtcBytes];
  uint8_t dtc_stored_count_ = 0;
  uint8_t dtc_pending_[kMaxDtcBytes];
  uint8_t dtc_pending_count_ = 0;
  uint8_t dtc_permanent_[kMaxDtcBytes];
  uint8_t dtc_permanent_count_ = 0;
};

}  // namespace core
