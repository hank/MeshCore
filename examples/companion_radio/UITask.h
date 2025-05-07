#pragma once

#include <MeshCore.h>
#include <helpers/ui/DisplayDriver.h>
#include <stddef.h>

class UITask {
  DisplayDriver* _display;
  mesh::MainBoard* _board;
  unsigned long _next_refresh, _auto_off;
  bool _connected;
  uint32_t _pin_code;
  const char* _node_name;
  char _version_info[32];
  char _origin[62];
  char _msg[80];
  int _msgcount;
  bool _need_refresh = true;
  uint16_t _battery_mv = 0;
  uint16_t _battery_pct = 0;

  void renderCurrScreen();
  void buttonHandler();
  void userLedHandler();

public:

  UITask(mesh::MainBoard* board) : _board(board), _display(NULL) {
      _next_refresh = 0; 
      _connected = false;
  }
  void begin(DisplayDriver* display, const char* node_name, const char* build_date, const char* firmware_version, uint32_t pin_code);

  void setHasConnection(bool connected) { _connected = connected; }
  uint8_t calculate_battery_percentage(uint16_t millivolts);
  void setBatteryLevel(uint16_t millivolts) { 
    _battery_mv = millivolts; 
    calculate_battery_percentage(_battery_mv);
  }
  bool hasDisplay() const { return _display != NULL; }
  void clearMsgPreview();
  void msgRead(int msgcount);
  void newMsg(uint8_t path_len, const char* from_name, const char* text, int msgcount);
  void loop();
};
