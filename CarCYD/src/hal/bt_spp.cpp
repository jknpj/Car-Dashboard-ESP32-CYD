/**
 * @file bt_spp.cpp
 * Bluetooth Classic SPP transport, see bt_spp.h.
 */
#include "bt_spp.h"

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "../config/app_config.h"
#include "../core/log.h"
#include "../core/util.h"

namespace hal {
namespace {

constexpr char kLocalName[] = "CarCYD";
constexpr uint32_t kScanIntervalMs = 5000;    // rescan pause when nothing found
constexpr uint32_t kConnectTimeoutMs = 12000; // retry connect after this
constexpr int kScanWindowMs = 7000;           // one discovery window

bool name_matches(const char* name) {
  // Case-insensitive substring match against known ELM327 clone names.
  char buf[48];
  size_t i = 0;
  for (const char* p = name; *p != '\0' && i + 1 < sizeof(buf); ++p) {
    if (*p >= 'A' && *p <= 'Z') buf[i++] = static_cast<char>(*p - 'A' + 'a');
    else buf[i++] = *p;
  }
  buf[i] = '\0';
  return strstr(buf, "obd") != nullptr || strstr(buf, "elm") != nullptr ||
         strstr(buf, "v-link") != nullptr || strstr(buf, "cbt") != nullptr;
}

bool parse_mac(const char* text, uint8_t mac[6]) {
  unsigned int b[6];
  if (sscanf(text, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) return false;
  for (int i = 0; i < 6; ++i) mac[i] = static_cast<uint8_t>(b[i]);
  return true;
}

}  // namespace

BtSppTransport* BtSppTransport::g_this = nullptr;

void BtSppTransport::set_mac(const char* mac) {
  if (mac == nullptr) mac_[0] = '\0';
  else {
    strncpy(mac_, mac, sizeof(mac_) - 1);
    mac_[sizeof(mac_) - 1] = '\0';
  }
  if (mode_ == Mode::Idle) return;  // nothing started yet
  // A new target invalidates the current connection attempt.
  if (bt_started_) bt_.disconnect();
  mode_ = mac_[0] != '\0' ? Mode::Connecting : Mode::Scanning;
  attempt_at_ms_ = 0;
}

void BtSppTransport::set_mac_found_callback(MacFoundFn fn) { mac_found_ = fn; }

void BtSppTransport::start_bt() {
  if (bt_started_) return;
  g_this = this;
  bt_.setPin(ELM_BT_PIN, strlen(ELM_BT_PIN));
  bt_.begin(kLocalName, /*master=*/true, /*disableBLE=*/true);
  bt_started_ = true;
  LOG_I("BT", "SPP started as '%s'", kLocalName);
}

void BtSppTransport::start_scan() {
  if (!bt_started_) start_bt();
  if (scan_running_) return;
  LOG_I("BT", "scanning for the adapter");
  scan_running_ = bt_.discoverAsync(BtSppTransport::scan_cb_static, kScanWindowMs / 1280);
  if (!scan_running_) mode_ = Mode::Idle;  // retry on next service()
}
void BtSppTransport::try_connect() {
  uint8_t addr[6];
  if (!parse_mac(mac_, addr)) {
    LOG_W("BT", "invalid MAC '%s', scanning instead", mac_);
    mode_ = Mode::Scanning;
    return;
  }
  bt_.disconnect();
  if (bt_.connect(addr)) {
    LOG_I("BT", "connecting to %s", mac_);
    attempt_at_ms_ = millis();
  } else {
    LOG_W("BT", "connect(%s) refused, retrying later", mac_);
    attempt_at_ms_ = millis() - kConnectTimeoutMs + kScanIntervalMs;
  }
}

void BtSppTransport::on_device(BTAdvertisedDevice* dev) {
  if (dev == nullptr) return;
  const std::string name = dev->getName();
  if (!name_matches(name.c_str())) return;
  const String addr = dev->getAddress().toString();
  snprintf(mac_, sizeof(mac_), "%s", addr.c_str());
  LOG_I("BT", "adapter found: %s (%s)", mac_, name.c_str());
  bt_.discoverAsyncStop();
  scan_running_ = false;
  mac_pending_ = true;  // deliver the callback from the loop context
  mode_ = Mode::Connecting;
  attempt_at_ms_ = 0;
}

// ---- core::ElmTransport ----------------------------------------------------------------

void BtSppTransport::begin() {
  mode_ = mac_[0] != '\0' ? Mode::Connecting : Mode::Scanning;
  attempt_at_ms_ = 0;
  LOG_I("BT", "transport begin, target %s", mac_[0] != '\0' ? mac_ : "(scan)");
}

bool BtSppTransport::connected() {
  return mode_ == Mode::Online && bt_.connected();
}

void BtSppTransport::service() {
  if (!bt_started_) start_bt();
  const uint32_t now = millis();

  if (mac_pending_) {  // deliver the discovery result from the loop context
    mac_pending_ = false;
    if (mac_found_ != nullptr) mac_found_(mac_);
  }

  switch (mode_) {
    case Mode::Idle:
      mode_ = mac_[0] != '\0' ? Mode::Connecting : Mode::Scanning;
      attempt_at_ms_ = 0;
      break;

    case Mode::Scanning:
      if (scan_running_) {
        if (core::elapsed(now, attempt_at_ms_) >= kScanWindowMs + 2000) scan_running_ = false;
      } else if (core::elapsed(now, attempt_at_ms_) >= kScanIntervalMs) {
        attempt_at_ms_ = now;
        start_scan();
      }
      break;

    case Mode::Connecting:
      if (bt_.connected()) {
        mode_ = Mode::Online;
        LOG_I("BT", "connected to %s", mac_);
      } else if (core::elapsed(now, attempt_at_ms_) >= kConnectTimeoutMs) {
        attempt_at_ms_ = now;
        try_connect();
      }
      break;

    case Mode::Online:
      if (!bt_.connected()) {
        LOG_W("BT", "link dropped");
        mode_ = mac_[0] != '\0' ? Mode::Connecting : Mode::Scanning;
        attempt_at_ms_ = 0;
      }
      break;
  }
}

int BtSppTransport::available() { return bt_.available(); }

int BtSppTransport::read() { return bt_.read(); }

void BtSppTransport::write(const uint8_t* data, size_t len) {
  bt_.write(data, len);
}

void BtSppTransport::reset() {
  if (bt_started_) bt_.disconnect();
  mode_ = mac_[0] != '\0' ? Mode::Connecting : Mode::Scanning;
  attempt_at_ms_ = 0;
}

}  // namespace hal
