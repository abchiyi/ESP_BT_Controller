#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include "functional"

typedef enum
{
  btnNone, // no button
  btnA,
  btnB,
  btnX,
  btnY,
  btnLB,
  btnRB,
  btnSelect,
  btnStart,
  btnXbox,
  btnLS,
  btnRS,
  btnShare,
  btnDirUp,
  btnDirRight,
  btnDirDown,
  btnDirLeft,

  XBOX_BUTTON_MAX,

  joyLHori = (XBOX_BUTTON_MAX + 1),
  joyLVert,
  joyRHori,
  joyRVert,
  trigLT,
  trigRT,

  XBOX_HAT_MAX
} XBOX_INPUT_t;

#define BUTTON_OFFSET 1
#define JOY_OFFSET (XBOX_BUTTON_MAX + 1)

#define XBOX_CALLBACK_MAX 3
typedef enum
{
  XBOX_ON_INPUT,
  XBOX_ON_CONNECTED,
  XBOX_ON_DISCONNECTED,
} XBOX_CALLBACK;

typedef std::function<void(void)> XBOX_CALLBACK_FUNC;

class XBOX
{
private:
public:
  bool button_bits[XBOX_BUTTON_MAX - BUTTON_OFFSET]; // bool
  int16_t analog_hat[XBOX_HAT_MAX - JOY_OFFSET];     // 0 ~ 2047

  void begin();
  bool getButtonPress(XBOX_INPUT_t);
  int16_t getAnalogHat(XBOX_INPUT_t);

  void connect_new();  // 连接新控制器
  void disconnect();   // 断开连接
  bool is_connected(); // 检查蓝牙控制器是否连接

  void setCallBack(XBOX_CALLBACK, XBOX_CALLBACK_FUNC);
};

extern XBOX Controller;

#endif
