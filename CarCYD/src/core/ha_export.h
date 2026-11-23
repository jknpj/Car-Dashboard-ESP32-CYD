/**
 * @file ha_export.h
 * Home Assistant export (health snapshot upload).
 *
 * HaExport reads VehicleState / AlertManager / TripComputer — the same single
 * source of truth the UI and alerts use — and ships a small set of *health*
 * entities to Home Assistant through a core::HaTransport (WiFi + HTTPS REST
 * in hal/ha_http.h). RPM/speed are deliberately not exported: the value is in
 * fuel, tyre pressure, battery, temperatures, fault codes and alerts.
 *
 * Publishing model:
 *   - every kPublishIntervalMs: averaged snapshots of the numeric entities
 *     (fuel, voltage, coolant, intake, DTC count, trip distance/fuel);
 *   - immediately: binary alert entities when an alert (or the MIL) changes
 *     state, so Home Assistant can push a phone notification right away.
 *
 * The entity ids must match the entities created by the Home Assistant
 * package in homeassistant/carcyd.yaml.
 */
#pragma once

#include <stdint.h>

#include "alerts.h"
#include "trip.h"
#include "vehicle_state.h"

namespace core {

/** Transport that ships entity updates to Home Assistant (see hal/ha_http.h). */
class HaTransport {
 public:
  virtual ~HaTransport() = default;

  /** (Re)apply the connection settings. Nothing touches the radio unless enabled and complete. */
  virtual void configure(const char* url, const char* token, const char* ssid, const char* pass, bool enabled) = 0;

  /** Drive the WiFi connection state machine. Call from the main loop. */
  virtual void service() = 0;

  /** True while WiFi is connected (not necessarily that the last POST worked). */
  virtual bool online() const = 0;

  /** POST one entity state as JSON. Returns true on HTTP 2xx. May block a few seconds. */
  virtual bool post_state(const char* entity_id, const char* json_body) = 0;
};

class HaExport {
 public:
  explicit HaExport(HaTransport& transport) : transport_(transport) {}

  /** Enable/disable upload (settings toggle). */
  void set_enabled(bool enabled) { enabled_ = enabled; }

  /** True while an upload burst is in flight (UI may show "uploading"). */
  bool publishing() const { return publishing_; }

  /** True while the WiFi link is up (not necessarily that the last POST worked). */
  bool online() const { return transport_.online(); }

  /** Call from the main loop as often as possible; non-blocking except upload bursts. */
  void poll(const VehicleState& state, const AlertManager& alerts, const TripData& trip, uint32_t now_ms);

 private:
  void publish_numerics(const VehicleState& state, const TripData& trip, uint32_t now_ms);
  bool publish_binary(const char* entity_id, const char* name, bool on);
  void sample(const VehicleState& state, uint32_t now_ms);

  HaTransport& transport_;
  bool enabled_ = false;
  bool publishing_ = false;

  // Numeric aggregation between publishes.
  struct Agg {
    float sum = 0.0f;
    float min = 0.0f;
    float max = 0.0f;
    uint16_t count = 0;
  };
  Agg agg_[kSignalCount];

  // Alert transition tracking (immediate publish on change).
  bool last_alert_active_[kAlertCount] = {};
  bool last_mil_ = false;
  bool mil_initialised_ = false;

  uint32_t last_publish_ms_ = 0;
};

/** The active exporter; app registers it once. Mirrors the core::link pattern. */
namespace ha {
void set_active(HaExport* exporter);
HaExport* active();
}  // namespace ha

}  // namespace core
