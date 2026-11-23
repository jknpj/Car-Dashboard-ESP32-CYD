/**
 * @file link_log.h
 * Short, power-safe ring of OBD/BT link events.
 *
 * The car has no serial console and no mindful passenger, so the last events
 * of the Bluetooth link are kept in NVS and shown on Settings > Connection.
 * That turns "it did not connect on the way back" into something readable
 * without a laptop.
 */
#pragma once

#include <stdint.h>

#include "storage.h"

namespace core::link_log {

enum class Code : uint8_t {
  Boot = 0,   ///< firmware start
  BtStart,    ///< Bluetooth Classic stack up
  BtFail,     ///< stack init failed
  Found,      ///< adapter matched by name (text = name)
  ConnectTry, ///< connect() issued (text = adapter name)
  ConnectOk,  ///< SPP link up
  ConnectFail,///< connect() returned false
  Forget,     ///< the stored MAC was dropped after repeated failures
  Drop,       ///< an established link went away
  ElmReady,   ///< ELM327 answered the whole AT sequence
  ElmFail,    ///< the link was reset after protocol errors
  Fallback,   ///< no known name, trying the generic candidate
};

struct Entry {
  uint8_t code;      ///< Code
  uint8_t reserved;  ///< padding, keeps the blob layout stable
  uint32_t at_ms;    ///< millis() at the event
  char text[16];     ///< short context: adapter name or MAC tail
};

constexpr uint8_t kMaxEntries = 10;

/** Loads the ring from NVS; missing or corrupt data starts empty. */
void begin(Storage& storage);

/** Appends an event. Call from the main loop only. */
void add(Code code, uint32_t now_ms, const char* text = nullptr);

/** Persists pending events (debounced) ; call from the main loop. */
void service(uint32_t now_ms);

/** Entries in chronological order, `count()` newest entries, 0 = oldest. */
uint8_t count();
const Entry& at(uint8_t index);

/** Short English label for an event code (logs are technical, untranslated). */
const char* label(Code code);

}  // namespace core::link_log
