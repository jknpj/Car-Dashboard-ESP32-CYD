/**
 * @file ha_http.cpp
 * WiFi + HTTPS transport for the Home Assistant upload, see ha_http.h.
 */
#include "ha_http.h"

#include <string.h>

#include "../core/log.h"
#include "../core/util.h"

namespace hal {
namespace {

constexpr uint32_t kConnectTimeoutMs = 20000;  // one WiFi join attempt
constexpr uint32_t kTlsConnectMs = 5000;       // TCP + TLS per POST
constexpr uint32_t kIoTimeoutMs = 4000;        // write/read per POST

// Let's Encrypt ISRG Root X1 — the CA behind Nabu Casa and most HA instances.
// Public certificate; embedding it lets the ESP32 verify the server instead of
// trusting every TLS peer.
constexpr char kRootCA[] =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n"
    "TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
    "cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n"
    "WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMTSW50ZXJu\n"
    "ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n"
    "MTAiMA0GCSqGSIb3DQEBAQUAA4ICAQB1pk/bj4faixOlDY1tVtWhk0Z0dNI2CH5b\n"
    "uUiGxK8vVI7UV9Lumw0Eh0tCNWDTMpdN6Zcsg8NV6LAqEtsSA5bZBCPp99CIZK5z\n"
    "4F9B5pWWOtSQX98v9V7MADBBRGpXV3XYa5YKAeViH1jJME3FTwVpr8C2vKGNPc0P\n"
    "66d6MRQO3wOqN4DoQ9oEJHVEXQL1OJwyblYc+PrBqOP+cTHRL9ExsjhXumE8Hw6q\n"
    "M+GCgq6CQO5ft7xipR+IPQvtzgZNAojkR1PXETp9Np4NUDPzL6GXcEPLn5N8VtQn\n"
    "GcMMiCzF7DAu0GiDe4FPLY6tWCuCDK3tZCv02J3hHvfFQ4tLLCmNF+N7qUv8lLbN\n"
    "UC7gI9Cm/ngbRrFcfXQGkSkv2+ta7quEyqMQVS8HofuWYVsKgdf8MEaSYzLO7R3T\n"
    "yR3vNF2y7eX1z+z8mFruLnubXKCBtKx+F+WcDrFDCQxUuSAPAmS5eBrwHxdR1If7\n"
    "uJPgHvF2u6KNbHnJ+db0E6mVn6TMj6XSflGSNEBvP5kVPtx4hcl4C67bR6qkKTKu\n"
    "q5tFAHFbZ9ovXGX7LOO/ivApVJg0ZTZqUf0ifvRnmNiK7nUv0KCfrHPYznPbnBX2\n"
    "mzm4N8kMyiw0Nv0ab9YDBm8VO8Efi6Wq3J5r0M6rQXjbZrx2uOWq6wO5miXJQdCr\n"
    "-----END CERTIFICATE-----\n";

bool parse_url(const char* url, char* host, size_t host_len, char* path, size_t path_len) {
  const char* p = url;
  if (strncmp(p, "https://", 8) == 0) {
    p += 8;
  } else if (strncmp(p, "http://", 7) == 0) {
    p += 7;  // accepted but TLS is still used unless the host is plain IP
  } else {
    return false;
  }
  const char* slash = strchr(p, '/');
  const size_t host_size = slash != nullptr ? static_cast<size_t>(slash - p) : strlen(p);
  if (host_size == 0 || host_size >= host_len) return false;
  memcpy(host, p, host_size);
  host[host_size] = '\0';
  if (slash != nullptr) {
    snprintf(path, path_len, "%s", slash);
  } else {
    snprintf(path, path_len, "/");
  }
  return true;
}

}  // namespace

// ---- Configuration --------------------------------------------------------------------

void HaHttp::configure(const char* url, const char* token, const char* ssid, const char* pass, bool enabled) {
  strncpy(ssid_, ssid != nullptr ? ssid : "", sizeof(ssid_) - 1);
  strncpy(pass_, pass != nullptr ? pass : "", sizeof(pass_) - 1);
  strncpy(token_, token != nullptr ? token : "", sizeof(token_) - 1);
  ssid_[sizeof(ssid_) - 1] = '\0';
  pass_[sizeof(pass_) - 1] = '\0';
  token_[sizeof(token_) - 1] = '\0';

  const bool complete = ssid_[0] != '\0' && token_[0] != '\0' && url != nullptr &&
                        parse_url(url, host_, sizeof(host_), path_, sizeof(path_));
  configured_ = enabled && complete;
  attempt_at_ms_ = 0;

  // Feature off or incomplete: leave the radio alone, releasing it only if we had started it.
  if (!configured_) stop_wifi();
  LOG_I("HA", "upload active=%d host=%s", configured_, configured_ ? host_ : "-");
}

void HaHttp::stop_wifi() {
  if (!started_) return;  // never touched the stack: nothing to free
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  started_ = false;
}

// ---- WiFi -----------------------------------------------------------------------------

bool HaHttp::start_connect() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  started_ = true;  // from here on the driver is initialised and must be released on disable
  const wl_status_t st = WiFi.begin(ssid_, pass_);
  LOG_I("HA", "joining '%s'...", ssid_);
  attempt_at_ms_ = millis();
  return st != WL_NO_SHIELD;
}

void HaHttp::service() {
  if (!configured_) return;  // never touch WiFi when the upload is disabled
  if (WiFi.status() == WL_CONNECTED) return;
  if (!started_ || core::elapsed(millis(), attempt_at_ms_) >= kConnectTimeoutMs) {
    if (started_) LOG_W("HA", "WiFi join timed out, retrying");
    start_connect();
  }
}

bool HaHttp::online() const {
  // Const-safe: only ask the driver once it has been started by service().
  if (!configured_ || !started_) return false;
  return WiFi.status() == WL_CONNECTED;
}

// ---- Upload ---------------------------------------------------------------------------

bool HaHttp::post_state(const char* entity_id, const char* json_body) {
  if (!online() || entity_id == nullptr || json_body == nullptr) return false;

  WiFiClientSecure client;
  client.setCACert(kRootCA);
  client.setTimeout(kIoTimeoutMs);

  if (!client.connect(host_, 443, static_cast<int32_t>(kTlsConnectMs))) {
    LOG_W("HA", "connect/TLS to %s failed", host_);
    client.stop();
    return false;
  }

  char request[512];
  const size_t body_len = strlen(json_body);
  const int n = snprintf(request, sizeof(request),
                         "POST %s/api/states/%s HTTP/1.1\r\n"
                         "Host: %s\r\n"
                         "Authorization: Bearer %s\r\n"
                         "Content-Type: application/json\r\n"
                         "Content-Length: %u\r\n"
                         "Connection: close\r\n"
                         "\r\n"
                         "%s",
                         path_, entity_id, host_, token_, static_cast<unsigned>(body_len), json_body);
  if (n < 0 || static_cast<size_t>(n) >= sizeof(request)) {
    LOG_E("HA", "request too large");
    client.stop();
    return false;
  }
  client.print(request);

  // Read the status line only; the body is irrelevant for us.
  char line[64];
  if (client.readBytesUntil('\n', line, sizeof(line) - 1) == 0) {
    client.stop();
    return false;
  }
  client.stop();
  const bool ok = strstr(line, "200") != nullptr || strstr(line, "201") != nullptr;
  if (!ok) LOG_W("HA", "HTTP reply: %s", line);
  return ok;
}

}  // namespace hal
