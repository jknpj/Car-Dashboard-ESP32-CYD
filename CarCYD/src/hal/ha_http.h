/**
 * @file ha_http.h
 * Home Assistant transport over WiFi + HTTPS REST (see core/ha_export.h).
 *
 * Connects to the configured WiFi (home network or phone hotspot) and POSTs
 * entity states to https://<ha-host>/api/states/<entity> authenticated with a
 * long-lived access token. Works through Nabu Casa remote URLs
 * (https://home-assistant.<domain>) or a local HA instance.
 *
 * Credentials are only ever held in RAM/NVS on the device; they are never
 * logged or sent anywhere except the configured HA host.
 */
#pragma once

#include <stdint.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "../core/ha_export.h"

namespace hal {

class HaHttp final : public core::HaTransport {
 public:
  // core::HaTransport
  void configure(const char* url, const char* token, const char* ssid, const char* pass, bool enabled) override;
  void service() override;
  bool online() const override;
  bool post_state(const char* entity_id, const char* json_body) override;

  /** True when settings are present and the feature is on (may still be connecting). */
  bool configured() const { return configured_; }

 private:
  bool start_connect();
  void stop_wifi();

  char ssid_[33] = {};
  char pass_[65] = {};
  char token_[193] = {};
  char host_[97] = {};   ///< hostname only (no scheme/path)
  char path_[97] = "/";  ///< leading slash; appended to /api/states/...
  bool configured_ = false;  ///< enabled AND settings complete
  bool started_ = false;     ///< the WiFi driver was touched at least once
  uint32_t attempt_at_ms_ = 0;
};

}  // namespace hal
