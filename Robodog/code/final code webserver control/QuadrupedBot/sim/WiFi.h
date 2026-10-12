#pragma once
#include <IPAddress.h>

enum wl_mode_t { WIFI_OFF = 0, WIFI_STA = 1, WIFI_AP = 2, WIFI_AP_STA = 3 };

class WiFiClass {
public:
  void mode(wl_mode_t m) {}
  void softAPConfig(IPAddress localIP, IPAddress gateway, IPAddress subnet) {}
  bool softAP(const char* ssid, const char* passphrase) { return true; }
  void setSleep(bool on) {}
  IPAddress softAPIP() { return IPAddress(); }
};

extern WiFiClass WiFi;
