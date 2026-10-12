#pragma once
#include <functional>
#include <Arduino.h>

class WebServer {
public:
  typedef std::function<void()> THandlerFunction;
  explicit WebServer(int port) {}
  void on(const char* uri, THandlerFunction fn) {}
  void begin() {}
  void handleClient() {}
  bool hasArg(const char* name) { return false; }
  String arg(const char* name) { return String(); }
  void send(int code, const char* contentType, const String& body) {}
  void send(int code, const char* contentType, const char* body) {}
};
