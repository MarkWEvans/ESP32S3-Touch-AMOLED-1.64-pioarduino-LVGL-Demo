#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  WIFI_MGR_STATE_OFF = 0,
  WIFI_MGR_STATE_CONNECTING = 1,
  WIFI_MGR_STATE_CONNECTED = 2,
  WIFI_MGR_STATE_DISCONNECTED = 3,
} wifi_mgr_state_t;

typedef struct {
  wifi_mgr_state_t state;
  int32_t rssi_dbm;      // 0 when unknown
  uint8_t has_ip;        // 0/1
  uint8_t ip[4];         // valid when has_ip=1
  char ssid[33];         // NUL-terminated when available
  uint32_t ws_clients;   // reserved for future use (currently 0)
} wifi_mgr_snapshot_t;

// Initializes WiFi using WIFI_SSID/WIFI_PASSWORD from pin_config.h (STA mode).
void wifi_manager_init(void);

// Connect/disconnect. Safe to call multiple times.
void wifi_manager_connect(const char *ssid, const char *password);
void wifi_manager_disconnect(void);

// Returns a point-in-time snapshot of WiFi status for UI updates.
bool wifi_manager_snapshot_get(wifi_mgr_snapshot_t *out);

#ifdef __cplusplus
} // extern "C"
#endif


