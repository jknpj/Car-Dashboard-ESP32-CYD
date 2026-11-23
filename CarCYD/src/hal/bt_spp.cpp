/**
 * @file bt_spp.cpp
 * Bluetooth Classic SPP transport, see bt_spp.h.
 */
#include "bt_spp.h"

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include <string>

#include "../config/app_config.h"
#include "../core/link_log.h"
#include "../core/log.h"
#include "../core/util.h"

namespace hal {
namespace {

constexpr char kLocalName[] = "CarCYD";
constexpr uint32_t kBtRetryMs = 5000;         // retry a failed BT stack init
constexpr uint32_t kScanIntervalMs = 3000;    // pause between discovery windows
constexpr uint32_t kScanWindowMs = 8000;      // one inquiry, in ms (BTAdvertisedDeviceCb takes ms)
constexpr uint32_t kScanGraceMs = 1500;       // let the last inquiry results arrive
constexpr uint32_t kConnectTimeoutMs = 15000; // retry a failed connect after this
constexpr uint8_t kFallbackWindows = 2;       // unscanned windows before trying a generic candidate
constexpr uint8_t kMaxConnectFailures = 2;    // forget the MAC and rescan after this many

bool name_matches(const char* name) {
  if (name == nullptr || name[0] == '\0') return false;
  // Case-insensitive substring match against the usual ELM327 clone names.
  char buf[48];
  size_t i = 0;
  for (const char* p = name; *p != '\0' && i + 1 < sizeof(buf); ++p) {
    buf[i++] = (*p >= 'A' && *p <= 'Z') ? static_cast<char>(*p - 'A' + 'a') : *p;
  }
  buf[i] = '\0';
  static const char* const kHints[] = {"obd",  "elm",   "v-link", "vlink", "veepeak", "327",
                                       "icar", "konnwei", "kkt",  "cbt",   "autool", "scantool"};
  for (const char* hint : kHints) {
    if (strstr(buf, hint) != nullptr) return true;
  }
  return false;
}

bool parse_mac(const char* text, uint8_t mac[6]) {
  unsigned int b[6];
  if (sscanf(text, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) return false;
  for (int i = 0; i < 6; ++i) {
    if (b[i] > 0xFFu) return false;
    mac[i] = static_cast<uint8_t>(b[i]);
  }
  return true;
}

/** Cheap OBD dongles usually report a miscellaneous/uncategorized Class of Device. */
bool is_generic_class(uint32_t cod) {
  const uint32_t major = (cod >> 8) & 0x1Fu;
  return major == 0x00 || major == 0x1F;
}

/** Context for the event log: the adapter name, or the tail of the MAC. */
const char* short_ctx(const char* name, const char* mac) {
  if (name != nullptr && name[0] != '\0') return name;
  if (mac == nullptr) return "-";
  const size_t n = strlen(mac);
  return n >= 8 ? mac + n - 8 : mac;
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
  stop_scan();
  if (bt_started_) bt_.disconnect();
  mode_ = mac_[0] != '\0' ? Mode::Connecting : Mode::Scanning;
  attempt_at_ms_ = millis();
  connect_now_ = mac_[0] != '\0';
  connect_failures_ = 0;
  empty_windows_ = 0;
}

void BtSppTransport::set_mac_found_callback(MacFoundFn fn) { mac_found_ = fn; }

bool BtSppTransport::start_bt() {
  if (bt_started_) return true;
  const uint32_t now = millis();
  if (bt_try_at_ms_ != 0 && core::elapsed(now, bt_try_at_ms_) < kBtRetryMs) return false;
  bt_try_at_ms_ = now;
  g_this = this;
  bt_started_ = bt_.begin(kLocalName, /*master=*/true, /*disableBLE=*/true);
  if (!bt_started_) {
    core::link_log::add(core::link_log::Code::BtFail, now);
    LOG_E("BT", "Bluetooth Classic init failed, retrying in %lu ms",
          static_cast<unsigned long>(kBtRetryMs));
    return false;
  }
  // setPin() talks to the GAP layer, so it only works once the stack is up.
  bt_.setPin(ELM_BT_PIN, static_cast<uint8_t>(strlen(ELM_BT_PIN)));
  core::link_log::add(core::link_log::Code::BtStart, now);
  LOG_I("BT", "SPP started as '%s'", kLocalName);
  return true;
}

void BtSppTransport::start_scan() {
  if (scan_running_) return;
  bt_.discoverAsyncStop();  // drop any stale inquiry/callback
  scan_started_ms_ = millis();
  scan_running_ = bt_.discoverAsync(BtSppTransport::scan_cb_static, kScanWindowMs);
  if (scan_running_) {
    LOG_I("BT", "scanning for the adapter (%lu ms)", static_cast<unsigned long>(kScanWindowMs));
  } else {
    LOG_W("BT", "discovery refused by the BT stack, retrying");
  }
}

void BtSppTransport::stop_scan() {
  if (!bt_started_) {
    scan_running_ = false;
    return;
  }
  if (!scan_running_) {
    bt_.discoverAsyncStop();  // harmless, clears a callback left behind
    return;
  }
  bt_.discoverAsyncStop();
  scan_running_ = false;
  LOG_I("BT", "scan window closed");
}

void BtSppTransport::try_connect() {
  uint8_t addr[6];
  if (!parse_mac(mac_, addr)) {
    LOG_W("BT", "invalid MAC '%s', scanning instead", mac_);
    mac_[0] = '\0';
    mode_ = Mode::Scanning;
    attempt_at_ms_ = millis();
    return;
  }
  stop_scan();
  bt_.disconnect();
  LOG_I("BT", "connecting to %s", mac_);
  core::link_log::add(core::link_log::Code::ConnectTry, millis(), short_ctx(last_name_, mac_));
  if (!bt_.connect(addr)) {
    // connect() blocks for up to 10 s, so count the failures and give up on a
    // stale/unreachable address instead of retrying it forever.
    LOG_W("BT", "connect(%s) failed", mac_);
    core::link_log::add(core::link_log::Code::ConnectFail, millis(), short_ctx(last_name_, mac_));
    if (++connect_failures_ >= kMaxConnectFailures) {
      LOG_W("BT", "adapter unreachable, forgetting %s and scanning again", mac_);
      core::link_log::add(core::link_log::Code::Forget, millis(), short_ctx(last_name_, mac_));
      mac_[0] = '\0';
      last_name_[0] = '\0';
      connect_failures_ = 0;
      mode_ = Mode::Scanning;
      attempt_at_ms_ = millis();
    }
  }
}

void BtSppTransport::on_device(BTAdvertisedDevice* dev) {
  if (dev == nullptr) return;
  const std::string name = dev->getName();
  const String addr = dev->getAddress().toString();
  const bool named = dev->haveName() && !name.empty();
  const uint32_t cod = dev->haveCOD() ? dev->getCOD() : 0xFFFFFFFFu;

  LOG_I("BT", "found %s [%s] rssi %d cod 0x%06lX", named ? name.c_str() : "(no name)", addr.c_str(),
        static_cast<int>(dev->getRSSI()), static_cast<unsigned long>(cod));

  // Remember a plausible (uncategorized) device in case no known name shows up:
  // some ELM327 clones advertise a name the list above does not know.
  if (cod == 0xFFFFFFFFu || is_generic_class(cod)) {
    if (!candidate_seen_ || dev->getRSSI() > candidate_rssi_) {
      candidate_seen_ = true;
      candidate_rssi_ = dev->getRSSI();
      snprintf(candidate_mac_, sizeof(candidate_mac_), "%s", addr.c_str());
    }
  }

  if (!name_matches(named ? name.c_str() : "")) return;

  snprintf(mac_, sizeof(mac_), "%s", addr.c_str());
  snprintf(last_name_, sizeof(last_name_), "%s", named ? name.c_str() : "");
  candidate_seen_ = false;
  empty_windows_ = 0;
  connect_failures_ = 0;
  mac_pending_ = true;  // stop the scan and persist the MAC from the loop context
  mode_ = Mode::Connecting;
  connect_now_ = true;
}

// ---- core::ElmTransport ----------------------------------------------------------------

void BtSppTransport::begin() {
  mode_ = mac_[0] != '\0' ? Mode::Connecting : Mode::Scanning;
  attempt_at_ms_ = millis();
  connect_now_ = mac_[0] != '\0';
  LOG_I("BT", "transport begin, target %s", mac_[0] != '\0' ? mac_ : "(scan)");
}

bool BtSppTransport::connected() { return mode_ == Mode::Online && bt_.connected(); }

void BtSppTransport::service() {
  if (!start_bt()) return;
  const uint32_t now = millis();

  if (mac_pending_) {  // deliver the discovery result from the loop context
    mac_pending_ = false;
    stop_scan();
    core::link_log::add(core::link_log::Code::Found, now, last_name_);
    if (mac_found_ != nullptr) mac_found_(mac_);
  }

  switch (mode_) {
    case Mode::Idle:
      mode_ = mac_[0] != '\0' ? Mode::Connecting : Mode::Scanning;
      attempt_at_ms_ = now;
      connect_now_ = mac_[0] != '\0';
      break;

    case Mode::Scanning:
      if (scan_running_) {
        if (core::elapsed(now, scan_started_ms_) >= kScanWindowMs + kScanGraceMs) {
          stop_scan();
          attempt_at_ms_ = now;
          if (empty_windows_ < 0xFF) ++empty_windows_;
          if (empty_windows_ >= kFallbackWindows && candidate_seen_) {
            snprintf(mac_, sizeof(mac_), "%s", candidate_mac_);
            last_name_[0] = '\0';
            LOG_W("BT", "no adapter name matched, trying strongest candidate %s", mac_);
            core::link_log::add(core::link_log::Code::Fallback, now, short_ctx(nullptr, mac_));
            connect_now_ = true;
            mode_ = Mode::Connecting;
          }
        }
      } else if (core::elapsed(now, attempt_at_ms_) >= kScanIntervalMs) {
        attempt_at_ms_ = now;
        start_scan();
      }
      break;

    case Mode::Connecting:
      if (bt_.connected()) {
        mode_ = Mode::Online;
        connect_failures_ = 0;
        empty_windows_ = 0;
        candidate_seen_ = false;
        core::link_log::add(core::link_log::Code::ConnectOk, now, short_ctx(last_name_, mac_));
        LOG_I("BT", "connected to %s", mac_);
      } else if (connect_now_ || core::elapsed(now, attempt_at_ms_) >= kConnectTimeoutMs) {
        connect_now_ = false;
        attempt_at_ms_ = now;
        try_connect();
      }
      break;

    case Mode::Online:
      if (!bt_.connected()) {
        LOG_W("BT", "link dropped");
        core::link_log::add(core::link_log::Code::Drop, now, short_ctx(last_name_, mac_));
        mode_ = mac_[0] != '\0' ? Mode::Connecting : Mode::Scanning;
        attempt_at_ms_ = now;
        connect_now_ = mac_[0] != '\0';
      }
      break;
  }
}

int BtSppTransport::available() { return bt_.available(); }

int BtSppTransport::read() { return bt_.read(); }

void BtSppTransport::write(const uint8_t* data, size_t len) { bt_.write(data, len); }

void BtSppTransport::reset() {
  stop_scan();
  if (bt_started_) bt_.disconnect();
  mode_ = mac_[0] != '\0' ? Mode::Connecting : Mode::Scanning;
  attempt_at_ms_ = millis();
  connect_now_ = mac_[0] != '\0';
  connect_failures_ = 0;
}

}  // namespace hal
