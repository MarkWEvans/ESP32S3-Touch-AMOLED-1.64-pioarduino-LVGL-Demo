#include "WifiManager.h"

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>

#include "pin_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static portMUX_TYPE g_wifi_spin = portMUX_INITIALIZER_UNLOCKED;

static wifi_mgr_state_t g_state = WIFI_MGR_STATE_OFF;
static int32_t g_rssi_dbm = 0;
static bool g_has_ip = false;
static uint8_t g_ip[4] = {0, 0, 0, 0};
static char g_ssid[33] = {0};
static uint32_t g_ws_clients = 0;

static char g_cfg_ssid[33] = {0};
static char g_cfg_pass[65] = {0};
static uint32_t g_last_begin_ms = 0;
static bool g_task_started = false;
static bool g_events_registered = false;
static bool g_mdns_started = false;
static volatile bool g_mdns_start_req = false;

typedef struct {
  bool connected;
  int32_t rssi_dbm;
  bool has_ip;
  uint8_t ip[4];
  char ssid[33];
} wifi_stack_snapshot_t;

static wifi_stack_snapshot_t wifi_collect_from_stack(void) {
  wifi_stack_snapshot_t s;
  memset(&s, 0, sizeof(s));

  wl_status_t st = WiFi.status();
  if (st != WL_CONNECTED) {
    s.connected = false;
    return s;
  }

  s.connected = true;
  s.rssi_dbm = (int32_t)WiFi.RSSI();

  String ss = WiFi.SSID();
  strncpy(s.ssid, ss.c_str(), sizeof(s.ssid) - 1);
  s.ssid[sizeof(s.ssid) - 1] = '\0';

  IPAddress ip = WiFi.localIP();
  s.ip[0] = ip[0];
  s.ip[1] = ip[1];
  s.ip[2] = ip[2];
  s.ip[3] = ip[3];
  s.has_ip = true;

  return s;
}

static void wifi_register_events_once(void) {
  if (g_events_registered) return;
  g_events_registered = true;

  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    (void)event;
    (void)info;
    portENTER_CRITICAL(&g_wifi_spin);
    if (g_state != WIFI_MGR_STATE_OFF) {
      g_state = WIFI_MGR_STATE_DISCONNECTED;
    }
    g_has_ip = false;
    g_ip[0] = g_ip[1] = g_ip[2] = g_ip[3] = 0;
    g_rssi_dbm = 0;
    g_ssid[0] = '\0';
    portEXIT_CRITICAL(&g_wifi_spin);
    // Let WiFi auto-reconnect do its thing; our poll task also retries.
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    (void)event;
    // Start mDNS after IP is acquired (do it from task context to avoid long work in callback).
    g_mdns_start_req = true;

    // Update IP from the event payload (cheap; no WiFi calls needed).
    uint32_t ip = info.got_ip.ip_info.ip.addr; // little-endian in ESP-IDF
    portENTER_CRITICAL(&g_wifi_spin);
    if (g_state != WIFI_MGR_STATE_OFF) {
      g_state = WIFI_MGR_STATE_CONNECTED;
    }
    g_has_ip = true;
    g_ip[0] = (uint8_t)(ip & 0xFF);
    g_ip[1] = (uint8_t)((ip >> 8) & 0xFF);
    g_ip[2] = (uint8_t)((ip >> 16) & 0xFF);
    g_ip[3] = (uint8_t)((ip >> 24) & 0xFF);
    portEXIT_CRITICAL(&g_wifi_spin);
  }, ARDUINO_EVENT_WIFI_STA_GOT_IP);
}

static void wifi_poll_task(void *arg) {
  (void)arg;
  for (;;) {
    bool have_creds = false;
    char ssid[33];
    char pass[65];
    wifi_mgr_state_t state_before = WIFI_MGR_STATE_OFF;

    portENTER_CRITICAL(&g_wifi_spin);
    have_creds = (g_cfg_ssid[0] != '\0');
    strncpy(ssid, g_cfg_ssid, sizeof(ssid) - 1);
    ssid[sizeof(ssid) - 1] = '\0';
    strncpy(pass, g_cfg_pass, sizeof(pass) - 1);
    pass[sizeof(pass) - 1] = '\0';
    state_before = g_state;
    portEXIT_CRITICAL(&g_wifi_spin);

    // Start mDNS (task context) when requested and WiFi is connected.
    if (g_mdns_start_req && !g_mdns_started && WiFi.status() == WL_CONNECTED) {
      g_mdns_start_req = false;
      g_mdns_started = MDNS.begin(MDNS_NAME);
    }

    wl_status_t st = WiFi.status();
    if (have_creds && st != WL_CONNECTED) {
      uint32_t now = millis();
      // Retry at most every 10 seconds.
      if ((uint32_t)(now - g_last_begin_ms) > 10000UL) {
        g_last_begin_ms = now;
        WiFi.mode(WIFI_STA);
        WiFi.setAutoReconnect(true);
        WiFi.begin(ssid, pass);
        portENTER_CRITICAL(&g_wifi_spin);
        if (g_state != WIFI_MGR_STATE_OFF) g_state = WIFI_MGR_STATE_CONNECTING;
        portEXIT_CRITICAL(&g_wifi_spin);
      }
    }

    // Collect from WiFi stack OUTSIDE critical section (WiFi calls can be slow).
    wifi_stack_snapshot_t snap = wifi_collect_from_stack();

    portENTER_CRITICAL(&g_wifi_spin);
    // Preserve OFF state if user disconnected
    if (g_state == WIFI_MGR_STATE_OFF) {
      // keep everything cleared
      g_has_ip = false;
      g_ip[0] = g_ip[1] = g_ip[2] = g_ip[3] = 0;
      g_rssi_dbm = 0;
      g_ssid[0] = '\0';
    } else if (snap.connected && snap.has_ip) {
      g_state = WIFI_MGR_STATE_CONNECTED;
      g_has_ip = true;
      g_ip[0] = snap.ip[0];
      g_ip[1] = snap.ip[1];
      g_ip[2] = snap.ip[2];
      g_ip[3] = snap.ip[3];
      g_rssi_dbm = snap.rssi_dbm;
      strncpy(g_ssid, snap.ssid, sizeof(g_ssid) - 1);
      g_ssid[sizeof(g_ssid) - 1] = '\0';
    } else {
      // Not connected
      if (have_creds) {
        // If we were explicitly disconnected, keep that; otherwise indicate connecting.
        if (state_before != WIFI_MGR_STATE_DISCONNECTED) {
          g_state = WIFI_MGR_STATE_CONNECTING;
        }
      } else {
        g_state = WIFI_MGR_STATE_OFF;
      }
      g_has_ip = false;
      g_ip[0] = g_ip[1] = g_ip[2] = g_ip[3] = 0;
      g_rssi_dbm = 0;
      g_ssid[0] = '\0';
    }
    portEXIT_CRITICAL(&g_wifi_spin);

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void wifi_manager_connect(const char *ssid, const char *password) {
  if (!ssid) ssid = "";
  if (!password) password = "";

  portENTER_CRITICAL(&g_wifi_spin);
  strncpy(g_cfg_ssid, ssid, sizeof(g_cfg_ssid) - 1);
  g_cfg_ssid[sizeof(g_cfg_ssid) - 1] = '\0';
  strncpy(g_cfg_pass, password, sizeof(g_cfg_pass) - 1);
  g_cfg_pass[sizeof(g_cfg_pass) - 1] = '\0';
  g_state = (g_cfg_ssid[0] != '\0') ? WIFI_MGR_STATE_CONNECTING : WIFI_MGR_STATE_OFF;
  portEXIT_CRITICAL(&g_wifi_spin);

  wifi_register_events_once();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(g_cfg_ssid, g_cfg_pass);
  g_last_begin_ms = millis();
}

void wifi_manager_disconnect(void) {
  portENTER_CRITICAL(&g_wifi_spin);
  g_cfg_ssid[0] = '\0';
  g_cfg_pass[0] = '\0';
  g_state = WIFI_MGR_STATE_OFF;
  g_has_ip = false;
  g_ip[0] = g_ip[1] = g_ip[2] = g_ip[3] = 0;
  g_rssi_dbm = 0;
  g_ssid[0] = '\0';
  portEXIT_CRITICAL(&g_wifi_spin);

  WiFi.disconnect(true, true);
  MDNS.end();
  g_mdns_started = false;
  g_mdns_start_req = false;
}

void wifi_manager_init(void) {
  wifi_register_events_once();

  if (!g_task_started) {
    g_task_started = true;
    xTaskCreate(wifi_poll_task, "wifi_mgr", 4096, NULL, 1, NULL);
  }

  // Default credentials from pin_config.h
  wifi_manager_connect(WIFI_SSID, WIFI_PASSWORD);
}

bool wifi_manager_snapshot_get(wifi_mgr_snapshot_t *out) {
  if (!out) return false;
  portENTER_CRITICAL(&g_wifi_spin);
  out->state = g_state;
  out->rssi_dbm = g_rssi_dbm;
  out->has_ip = g_has_ip ? 1 : 0;
  out->ip[0] = g_ip[0];
  out->ip[1] = g_ip[1];
  out->ip[2] = g_ip[2];
  out->ip[3] = g_ip[3];
  strncpy(out->ssid, g_ssid, sizeof(out->ssid) - 1);
  out->ssid[sizeof(out->ssid) - 1] = '\0';
  out->ws_clients = g_ws_clients;
  portEXIT_CRITICAL(&g_wifi_spin);
  return true;
}
