/**
 * @file bt_spp.h
 * Bluetooth Classic (SPP) transport for the ELM327 adapter.
 *
 * Connects to a known adapter MAC (saved in the settings) or, when none is
 * stored, scans for a device whose name mentions OBD / ELM / V-LINK / CBT and
 * remembers the MAC through the callback. Cheap ELM327 clones are legacy
 * pairing devices: the PIN is fixed (see ELM_BT_PIN in app_config.h).
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include <BluetoothSerial.h>

#include "../core/elm_link.h"

namespace hal {

class BtSppTransport final : public core::ElmTransport {
 public:
  using MacFoundFn = void (*)(const char* mac);

  /** Target MAC "AA:BB:CC:DD:EE:FF"; empty string = scan for the adapter. */
  void set_mac(const char* mac);
  void set_mac_found_callback(MacFoundFn fn);

  // core::ElmTransport
  void begin() override;
  bool connected() override;
  void service() override;
  int available() override;
  int read() override;
  void write(const uint8_t* data, size_t len) override;
  void reset() override;
  const char* name() const override { return "bt-spp"; }

 private:
  enum class Mode : uint8_t { Idle, Scanning, Connecting, Online };

  bool start_bt();
  void start_scan();
  void stop_scan();
  void try_connect();
  void on_device(BTAdvertisedDevice* dev);

  static void scan_cb_static(BTAdvertisedDevice* dev) {
    if (g_this != nullptr) g_this->on_device(dev);
  }
  static BtSppTransport* g_this;

  BluetoothSerial bt_;
  Mode mode_ = Mode::Idle;
  char mac_[18] = {};
  char last_name_[20] = {};  ///< adapter name from the last discovery, for the event log
  MacFoundFn mac_found_ = nullptr;
  bool bt_started_ = false;
  bool scan_running_ = false;
  bool mac_pending_ = false;    ///< MAC discovered on the BT task, awaiting loop pickup
  bool connect_now_ = false;    ///< connect on the next service() tick
  uint32_t attempt_at_ms_ = 0;  ///< last scan/connect attempt, for the retry timers
  uint32_t scan_started_ms_ = 0;
  uint32_t bt_try_at_ms_ = 0;
  uint8_t connect_failures_ = 0;

  // Fallback when no adapter name matches: strongest uncategorized device.
  char candidate_mac_[18] = {};
  int8_t candidate_rssi_ = -128;
  bool candidate_seen_ = false;
  uint8_t empty_windows_ = 0;
};

}  // namespace hal
