/**
 * @file ha_export.cpp
 * Home Assistant health export, see ha_export.h.
 */
#include "ha_export.h"

#include <stdio.h>

#include "log.h"
#include "util.h"

namespace core {
namespace {

constexpr uint32_t kPublishIntervalMs = 60000;  // numeric snapshot cadence
constexpr size_t kJsonMax = 256;  // worst-case entity body

/** Numeric entities: value is the average since the last publish, min/max ride along. */
struct NumericEntity {
  const char* id;             ///< full HA entity id, e.g. "input_number.carcyd_fuel_level"
  SignalId signal;
  const char* unit;           ///< HA unit_of_measurement
  const char* name;           ///< HA friendly_name
  const char* device_class;   ///< HA device_class ("" when none)
};

constexpr NumericEntity kNumericEntities[] = {
    {"input_number.carcyd_fuel_level", SignalId::FuelLevel, "%", "Fuel level", "fuel"},
    {"input_number.carcyd_voltage", SignalId::BatteryVoltage, "V", "Battery voltage", "voltage"},
    {"input_number.carcyd_coolant_temp", SignalId::CoolantTemp, "\xC2\xB0"
                                                                 "C",
     "Coolant temperature", "temperature"},
    {"input_number.carcyd_intake_temp", SignalId::IntakeTemp, "\xC2\xB0"
                                                               "C",
     "Intake temperature", "temperature"},
};
constexpr uint8_t kNumericCount = sizeof(kNumericEntities) / sizeof(kNumericEntities[0]);

/** Binary entities mapped to an alert; also the MIL/DTC pair below. */
struct BinaryEntity {
  const char* id;  ///< full HA entity id, e.g. "input_boolean.carcyd_low_fuel"
  AlertId alert;
  const char* name;
};

constexpr BinaryEntity kBinaryEntities[] = {
    {"input_boolean.carcyd_low_fuel", AlertId::FuelLow, "Low fuel"},
    {"input_boolean.carcyd_low_voltage", AlertId::BatteryLow, "Low battery voltage"},
    {"input_boolean.carcyd_coolant_high", AlertId::CoolantHigh, "Coolant temperature high"},
    {"input_boolean.carcyd_link_lost", AlertId::LinkLost, "OBD link lost"},
};
constexpr uint8_t kBinaryCount = sizeof(kBinaryEntities) / sizeof(kBinaryEntities[0]);

constexpr char kEntityMIL[] = "input_boolean.carcyd_mil";
constexpr char kEntityDtcCount[] = "input_number.carcyd_dtc_count";
constexpr char kEntityTripKm[] = "input_number.carcyd_trip_km";
constexpr char kEntityFuelLitres[] = "input_number.carcyd_fuel_litres";

}  // namespace

// ---- Lifecycle ------------------------------------------------------------------------

namespace ha {
namespace {
HaExport* g_active = nullptr;
}  // namespace

void set_active(HaExport* exporter) { g_active = exporter; }
HaExport* active() { return g_active; }
}  // namespace ha

// ---- Sampling -------------------------------------------------------------------------

void HaExport::sample(const VehicleState& state, uint32_t now_ms) {
  for (uint8_t i = 0; i < kNumericCount; ++i) {
    const NumericEntity& e = kNumericEntities[i];
    if (!state.fresh(e.signal, now_ms)) continue;
    const float v = state.get(e.signal).value;
    Agg& a = agg_[static_cast<uint8_t>(e.signal)];
    a.sum += v;
    if (a.count == 0 || v < a.min) a.min = v;
    if (a.count == 0 || v > a.max) a.max = v;
    a.count++;
  }
}

void HaExport::poll(const VehicleState& state, const AlertManager& alerts, const TripData& trip, uint32_t now_ms) {
  if (!enabled_) return;
  if (!transport_.online()) return;  // sample nothing while offline: HA keeps the last values

  sample(state, now_ms);

  // Alert transitions publish immediately.
  for (uint8_t i = 0; i < kBinaryCount; ++i) {
    const bool active_now = alerts.get(kBinaryEntities[i].alert).active();
    if (active_now != last_alert_active_[i] && publish_binary(kBinaryEntities[i].id, kBinaryEntities[i].name, active_now)) {
      last_alert_active_[i] = active_now;  // only advance on success, so failures retry
    }
  }
  const bool mil = state.dtc.mil_known && state.dtc.mil_on;
  if ((!mil_initialised_ || mil != last_mil_) && publish_binary(kEntityMIL, "Check engine (MIL)", mil)) {
    mil_initialised_ = true;
    last_mil_ = mil;
  }

  if (elapsed(now_ms, last_publish_ms_) >= kPublishIntervalMs) publish_numerics(state, trip, now_ms);
}

// ---- Publishing -----------------------------------------------------------------------

bool HaExport::publish_binary(const char* entity_id, const char* name, bool on) {
  char body[kJsonMax];
  const int n = snprintf(body, sizeof(body), "{\"state\":\"%s\",\"attributes\":{\"friendly_name\":\"%s\"}}",
                         on ? "on" : "off", name);
  if (n < 0 || static_cast<size_t>(n) >= sizeof(body)) return false;
  if (!transport_.post_state(entity_id, body)) {
    LOG_W("HA", "post %s failed", entity_id);
    return false;  // caller keeps the previous state so the next poll retries
  }
  return true;
}

void HaExport::publish_numerics(const VehicleState& state, const TripData& trip, uint32_t now_ms) {
  last_publish_ms_ = now_ms;
  publishing_ = true;

  char body[kJsonMax];
  for (uint8_t i = 0; i < kNumericCount; ++i) {
    const NumericEntity& e = kNumericEntities[i];
    Agg& a = agg_[static_cast<uint8_t>(e.signal)];
    if (a.count == 0) continue;
    const float avg = a.sum / static_cast<float>(a.count);
    a = Agg{};
    const int n = snprintf(body, sizeof(body),
                           "{\"state\":\"%.1f\",\"attributes\":{\"unit_of_measurement\":\"%s\","
                           "\"friendly_name\":\"%s\",\"device_class\":\"%s\"}}",
                           avg, e.unit, e.name, e.device_class);
    if (n < 0 || static_cast<size_t>(n) >= sizeof(body)) continue;
    transport_.post_state(e.id, body);
  }

  // Diagnostic/trip counters (fresh, no averaging).
  if (state.dtc.scanned) {
    snprintf(body, sizeof(body),
             "{\"state\":\"%u\",\"attributes\":{\"friendly_name\":\"DTC count\",\"device_class\":\"\"}}",
             state.dtc.count);
    transport_.post_state(kEntityDtcCount, body);
  }
  snprintf(body, sizeof(body),
           "{\"state\":\"%.2f\",\"attributes\":{\"unit_of_measurement\":\"km\",\"friendly_name\":\"Trip distance\","
           "\"device_class\":\"distance\",\"state_class\":\"total_increasing\"}}",
           trip.distance_km);
  transport_.post_state(kEntityTripKm, body);
  snprintf(body, sizeof(body),
           "{\"state\":\"%.2f\",\"attributes\":{\"unit_of_measurement\":\"L\",\"friendly_name\":\"Trip fuel used\","
           "\"device_class\":\"volume\",\"state_class\":\"total_increasing\"}}",
           trip.fuel_litres);
  transport_.post_state(kEntityFuelLitres, body);

  publishing_ = false;
}

}  // namespace core
