#include "XBOX.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "config.h"
#include "tool.h"
#include "atomic"
#include "rw_lock.h"

#include "esp_hidh.h"
#include "esp_hid_gap.h"

#define TAG "Controller"
#define XBOX_CONTROLLER_INDEX_BUTTONS_DIR 12
#define XBOX_CONTROLLER_INDEX_BUTTONS_MAIN 13
#define XBOX_CONTROLLER_INDEX_BUTTONS_CENTER 14
#define XBOX_CONTROLLER_INDEX_BUTTONS_SHARE 15
XBOX Controller; // 控制器对象
rwlock_t rwlock; // 读写锁

// NVS 储存
#define NVS_NAMESPACE "bt_controller"
#define NVS_KEY_LAST_DEV "last_dev"

// BT 连接
#define SCAN_DURATION_SECONDS 5        // 扫描时间
static TaskHandle_t *thc = nullptr;    // 连接手柄任务句柄
std::atomic<bool> SCAN_NEW(false);     // 是否连接新设备
std::atomic<bool> IS_CONNECTED(false); // 是否已连接
#define TRY_CONNECT_TIMES 2            // 尝试连接次数

// 处理回调
static XBOX_CALLBACK_FUNC CB_ARRAY[XBOX_CALLBACK_MAX] = {nullptr};

// 过滤摇杆输出到-2048~2047;
short analogHatFilter(uint16_t rawValue)
{
  const int mid = 32768;      // 摇杆中间原始值
  const int deadzone = 4000;  // 虚位死区阈值（根据实际硬件调整）
  const int maxOutput = 2048; // 目标最大输出值

  // 计算相对于中间值的偏移量（范围：-32768 ~ +32767）
  int offset = (int)rawValue - mid;

  // 死区过滤：中间虚位部分直接返回0
  if (abs(offset) <= deadzone)
    return 0;

  // 确定方向（正：右摇杆，负：左摇杆）
  int direction = (offset > 0) ? 1 : -1;

  // 计算有效偏移量（扣除死区后的实际偏移）
  int effectiveOffset = abs(offset) - deadzone;

  // 计算有效范围的最大偏移量（左/右可能不对称）
  int maxEffectiveOffset;
  if (direction == 1)
  {
    // 右侧有效范围：从 (mid + deadzone) 到 65535
    maxEffectiveOffset = 65535 - (mid + deadzone);
  }
  else
  {
    // 左侧有效范围：从 0 到 (mid - deadzone)
    maxEffectiveOffset = mid - deadzone;
  }

  // 线性缩放至目标范围 [-2048, 2048]
  int scaledValue = (effectiveOffset * maxOutput) / maxEffectiveOffset;

  // 确保不超出目标范围
  scaledValue = (scaledValue > maxOutput) ? maxOutput : scaledValue;

  // 返回带方向的最终值
  return (short)(direction * scaledValue);
}

static struct bt_dev // 蓝牙设备信息
{
  esp_bd_addr_t bda;
  esp_hid_transport_t transport;
  esp_ble_addr_type_t addr_type;
} devInfo;

esp_err_t save_last_dev(bt_dev *bd)
{
  esp_err_t ret;
  nvs_handle_t nvs;
  ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "nvs_open failed, error code = %s", esp_err_to_name(ret));
    return ESP_FAIL;
  }

  ret = nvs_set_blob(nvs, NVS_KEY_LAST_DEV, bd, sizeof(bt_dev));
  if (ret == ESP_OK)
  {
    ret = nvs_commit(nvs);
    ESP_LOGE(TAG, "nvs_commit failed, error code = %s", esp_err_to_name(ret));
  }
  else
    ESP_LOGE(TAG, "nvs_set_blob failed, error code = %s", esp_err_to_name(ret));

  nvs_close(nvs);
  return ESP_OK;
}

esp_err_t read_last_dev(bt_dev *bd)
{
  esp_err_t ret;
  nvs_handle_t nvs;
  ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "nvs_open failed, error code = %s", esp_err_to_name(ret));
    return ESP_FAIL;
  }

  size_t req_size = sizeof(bt_dev);
  ret = nvs_get_blob(nvs, NVS_KEY_LAST_DEV, bd, &req_size);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "nvs_get_blob failed, error code = %s", esp_err_to_name(ret));
    return ESP_FAIL;
  }

  nvs_close(nvs);
  return ESP_OK;
}

void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
  esp_hidh_event_t event = (esp_hidh_event_t)id;
  esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

  switch (event)
  {
  case ESP_HIDH_OPEN_EVENT:
  {
    if (param->open.status == ESP_OK)
    {
      const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
      ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
      esp_hidh_dev_dump(param->open.dev, stdout);
      IS_CONNECTED.store(true);
      auto cbfn = CB_ARRAY[XBOX_ON_CONNECTED];
      if (cbfn)
        cbfn();
    }
    else
    {
      ESP_LOGE(TAG, " OPEN failed!");
    }
    break;
  }
  case ESP_HIDH_BATTERY_EVENT:
  {
    const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
    ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
    break;
  }
  case ESP_HIDH_INPUT_EVENT:
  {

    // const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
    // ESP_LOGI(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
    // ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
    // ESP_LOGI(TAG, "id:%3u", param->input.report_id);

    bool button_bits[16] = {};  // bool
    int16_t analog_hat[6] = {}; // 0 ~ 2047

    uint8_t btnBits;
    /*
    btnA = btnBits & 0b00000001;
    btnB = btnBits & 0b00000010;
    btnX = btnBits & 0b00001000;
    btnY = btnBits & 0b00010000;
    btnLB = btnBits & 0b01000000;
    btnRB = btnBits & 0b10000000;
    */
    btnBits = param->input.data[XBOX_CONTROLLER_INDEX_BUTTONS_MAIN];
    button_bits[btnA] = (btnBits & 0b00000001);
    button_bits[btnB] = (btnBits & 0b00000010);
    button_bits[btnX] = (btnBits & 0b00001000);
    button_bits[btnY] = (btnBits & 0b00010000);
    button_bits[btnLB] = (btnBits & 0b01000000);
    button_bits[btnRB] = (btnBits & 0b10000000);

    /*
    btnSelect = btnBits & 0b00000100;
    btnStart = btnBits & 0b00001000;
    btnXbox = btnBits & 0b00010000;
    btnLS = btnBits & 0b00100000;
    btnRS = btnBits & 0b01000000;
    */

    btnBits = param->input.data[XBOX_CONTROLLER_INDEX_BUTTONS_CENTER];
    button_bits[btnSelect] = (btnBits & 0b00000100);
    button_bits[btnStart] = (btnBits & 0b00001000);
    button_bits[btnXbox] = (btnBits & 0b00010000);
    button_bits[btnLS] = (btnBits & 0b00100000);
    button_bits[btnRS] = (btnBits & 0b01000000);

    /*
    btnShare = btnBits & 0b00000001;
    */
    btnBits = param->input.data[XBOX_CONTROLLER_INDEX_BUTTONS_SHARE];
    button_bits[btnShare] = (btnBits & 0b00000001);

    btnBits = param->input.data[XBOX_CONTROLLER_INDEX_BUTTONS_DIR];
    auto dirUP = btnBits == 1 || btnBits == 2 || btnBits == 8;
    auto dirRight = 2 <= btnBits && btnBits <= 4;
    auto dirDown = 4 <= btnBits && btnBits <= 6;
    auto dirLeft = 6 <= btnBits && btnBits <= 8;

    button_bits[btnDirUp] = dirUP;
    button_bits[btnDirRight] = dirRight;
    button_bits[btnDirDown] = dirDown;
    button_bits[btnDirLeft] = dirLeft;

    /*
     * joyLHori 0
     * joyLVori 2
     * joyRHori 4
     * joyRVori 6
     * trigLT   8
     * trigRT   10
     */
    auto read_analog = [&](uint8_t index)
    {
      return (uint16_t)param->input.data[index] |
             ((uint16_t)param->input.data[index + 1] << 8);
    };

    analog_hat[joyLHori] = analogHatFilter(read_analog(0));
    analog_hat[joyLVert] = analogHatFilter(read_analog(2));
    analog_hat[joyRHori] = analogHatFilter(read_analog(4));
    analog_hat[joyRVert] = analogHatFilter(read_analog(6));
    analog_hat[trigLT] = read_analog(8);
    analog_hat[trigRT] = read_analog(10);

    // 将组装好的数据写入
    rwlock_write_lock(&rwlock);
    memcpy(Controller.analog_hat, analog_hat, sizeof(Controller.analog_hat));
    memcpy(Controller.button_bits, button_bits, sizeof(Controller.button_bits));
    rwlock_write_unlock(&rwlock);

    break;
  }
  case ESP_HIDH_FEATURE_EVENT:
  {
    const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
    ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
             esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
             param->feature.length);
    ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
    break;
  }
  case ESP_HIDH_CLOSE_EVENT:
  {
    const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
    ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
    memset(Controller.analog_hat, 0, sizeof(Controller.analog_hat));
    memset(Controller.button_bits, 0, sizeof(Controller.button_bits));
    IS_CONNECTED.store(false);
    SCAN_NEW.store(false);
    auto cbfn = CB_ARRAY[XBOX_ON_DISCONNECTED];
    if (cbfn)
      cbfn();
    break;
  }
  default:
    ESP_LOGI(TAG, "EVENT: %d", event);
    break;
  }
}

void bt_controller_init()
{
  esp_err_t ret;

  // TODO 分离初始化nvs程序
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG, "BT controller init");

  ESP_ERROR_CHECK(esp_hid_gap_init(HIDH_BLE_MODE));
  ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler));

  esp_hidh_config_t config = {
      .callback = hidh_callback,
      .event_stack_size = 4096,
      .callback_arg = NULL,
  };
  ESP_ERROR_CHECK(esp_hidh_init(&config));
};

// 启动xbox控制器
void XBOX::begin()
{
  ESP_LOGI(TAG, " init controller");

  // 初始化读写锁
  rwlock_init(&rwlock);

  // 初始化蓝牙控制器
  bt_controller_init();

  // 设置连接任务
  static auto task_connect = [](void *pt)
  {
    // 确保连接任务只启动一个
    if (thc != nullptr)
      vTaskDelete(NULL);

    // 读取上次连接设备，如没有信息则扫描
    if (read_last_dev(&devInfo) != ESP_OK)
      SCAN_NEW.store(true);

    auto from_scan = [&]()
    {
      size_t results_len = 0;
      esp_hid_scan_result_t *results = NULL;

      ESP_LOGI(TAG, "SCAN...");
      // start scan for HID devices
      esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);
      ESP_LOGI(TAG, "SCAN: %u results", results_len);
      if (results_len)
      {
        esp_hid_scan_result_t *r = results;
        esp_hid_scan_result_t *cr = NULL;
        while (r)
        {
          printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
          printf("RSSI: %d, ", r->rssi);
          printf("USAGE: %s, ", esp_hid_usage_str(r->usage));

          // TODO 检测 是否等于 0x03C4|0x03C2 判断类型是否为手柄
          if (r->transport == ESP_HID_TRANSPORT_BLE)
          {
            cr = r;
            printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
            printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
          }

          printf("NAME: %s ", r->name ? r->name : "");
          printf("\n");
          r = r->next;
        }
        if (cr)
        {
          // open the last result
          esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);

          // save the last result
          memcpy(&devInfo.bda, cr->bda, sizeof(devInfo.bda));
          devInfo.transport = cr->transport;
          devInfo.addr_type = cr->ble.addr_type;
          save_last_dev(&devInfo);
        }
        // free the results
        esp_hid_scan_results_free(results);
      }
    };

    auto from_last = [&]()
    {
      ESP_LOGI(TAG, "Connected to last device......");
      esp_hidh_dev_open(devInfo.bda, devInfo.transport, devInfo.addr_type);
    };

    uint8_t counter = 0;
    while (true)
    {
      vTaskDelay(100);
      if (IS_CONNECTED.load())
        continue;

      if (SCAN_NEW.load())
      {
        from_scan();
        vTaskDelay(200);
      }
      else
      {
        if (counter < TRY_CONNECT_TIMES)
        {
          counter++;
          ESP_LOGI(TAG, "Try to connect last device,[%d/%d]",
                   counter, TRY_CONNECT_TIMES);
          from_last();
        }
        else
        {
          counter = 0;
          SCAN_NEW.store(true);
        }
      }
    }
  };
  auto ret = xTaskCreate(task_connect, "task_connect", 4096, NULL, 5, thc);
  ESP_ERROR_CHECK(ret == pdPASS ? ESP_OK : ESP_FAIL);
}

/**
 * @brief 获取指定按钮的按压状态
 * @param btn 要检查的按钮枚举值
 * @return bool 如果按钮被按下返回true,否则返回false
 */
bool XBOX::getButtonPress(XBOX_INPUT_t btn)
{
  rwlock_read_lock(&rwlock);
  auto v = this->button_bits[btn];
  rwlock_read_unlock(&rwlock);
  return v;
}

/**
 * @brief 获取 Xbox 控制器模拟摇杆的值
 * @param hat 模拟摇杆的类型(枚举值 XBOX_ANALOG_HAT)
 * @return 返回模拟摇杆的当前值(-2048 到 2047), trig 为 (0 到 2047)
 */
int16_t XBOX::getAnalogHat(XBOX_INPUT_t hat)
{
  rwlock_read_lock(&rwlock);
  auto v = this->analog_hat[hat - (XBOX_BUTTON_MAX + 1)];
  rwlock_read_unlock(&rwlock);
  return v;
}

/**
 * @brief 检查蓝牙控制器的连接状态
 * @return 如果控制器已连接返回 true，否则返回 false
 */
bool XBOX::is_connected()
{
  return IS_CONNECTED.load();
}

void XBOX::disconnect()
{
  ESP_ERROR_CHECK(esp_ble_gap_disconnect(devInfo.bda));
}

void XBOX::connect_new()
{
  this->disconnect();
  SCAN_NEW.store(true);
}

void XBOX::setCallBack(XBOX_CALLBACK cb_type, XBOX_CALLBACK_FUNC cbfn)
{
  if (CB_ARRAY[cb_type])
    ESP_LOGW(TAG, "Callback function overwrite");
  CB_ARRAY[cb_type] = cbfn;
}
