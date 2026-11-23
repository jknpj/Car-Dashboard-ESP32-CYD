/**
 * @file link_log.cpp
 * See link_log.h.
 */
#include "link_log.h"

#include <string.h>

#include "log.h"
#include "persist.h"
#include "util.h"

namespace core::link_log {
namespace {

constexpr const char* kKey = "link_log";
constexpr uint16_t kVersion = 1;
constexpr uint32_t kSaveDelayMs = 1500;  // batch a burst of events into one NVS write
constexpr const char* kLabels[] = {
    "boot",  "BT up", "BT failed", "found",   "try",       "connected",
    "connect failed", "forgot MAC", "dropped", "ELM ready", "ELM reinit", "fallback",
};

struct Blob {
  uint8_t count;
  uint8_t next;
  Entry entries[kMaxEntries];
};

Blob g_blob = {};
Storage* g_storage = nullptr;
bool g_dirty = false;
uint32_t g_last_save_ms = 0;

}  // namespace

void begin(Storage& storage) {
  g_storage = &storage;
  g_blob = {};
  const persist::LoadResult result = persist::load(storage, kKey, &g_blob, sizeof(g_blob));
  if (result != persist::LoadResult::Ok && result != persist::LoadResult::Upgraded) {
    g_blob = {};
    if (result == persist::LoadResult::Corrupt) LOG_W("LOG", "link log corrupt, starting empty");
  }
  if (g_blob.count > kMaxEntries) g_blob.count = kMaxEntries;
  if (g_blob.next >= kMaxEntries) g_blob.next = 0;
}

void add(Code code, uint32_t now_ms, const char* text) {
  Entry& e = g_blob.entries[g_blob.next];
  e = {};
  e.code = static_cast<uint8_t>(code);
  e.at_ms = now_ms;
  if (text != nullptr) {
    strncpy(e.text, text, sizeof(e.text) - 1);
  }
  g_blob.next = static_cast<uint8_t>((g_blob.next + 1) % kMaxEntries);
  if (g_blob.count < kMaxEntries) g_blob.count++;
  g_dirty = true;
}

void service(uint32_t now_ms) {
  if (!g_dirty || g_storage == nullptr) return;
  if (elapsed(now_ms, g_last_save_ms) < kSaveDelayMs) return;
  g_last_save_ms = now_ms;
  if (persist::save(*g_storage, kKey, kVersion, &g_blob, sizeof(g_blob))) {
    g_dirty = false;
  } else {
    LOG_W("LOG", "link log save failed");
  }
}

uint8_t count() { return g_blob.count; }

const Entry& at(uint8_t index) {
  if (index >= g_blob.count) index = g_blob.count == 0 ? 0 : g_blob.count - 1;
  const uint8_t first = static_cast<uint8_t>((g_blob.next + kMaxEntries - g_blob.count) % kMaxEntries);
  return g_blob.entries[(first + index) % kMaxEntries];
}

const char* label(Code code) {
  const uint8_t i = static_cast<uint8_t>(code);
  return i < (sizeof(kLabels) / sizeof(kLabels[0])) ? kLabels[i] : "?";
}

}  // namespace core::link_log
