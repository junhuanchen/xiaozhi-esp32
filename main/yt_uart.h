#ifndef _YT_UART_H_
#define _YT_UART_H_

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include <string>
#include <mutex>
#include <deque>
#include <vector>
#include <memory>

#include "protocol.h"
#include "ota.h"
#include "audio_service.h"
#include "device_state_event.h"


class ytUart {
public:
      // 获取全局唯一实例
  static ytUart& instance();

    // 禁止拷贝与赋值
  ytUart(const ytUart&) = delete;
  ytUart& operator=(const ytUart&) = delete;

  // 对外接口：
  esp_err_t protocol_send_data(const char * data, int len);
  // 初始化状态
  bool is_initialized() const { return initialized_; }
  // 停止
  void stop();
  bool get_ble_status();
 

  private:
  // 私有构造：初始化 UART
  ytUart();
  ~ytUart();


  // UART 配置常量
  static constexpr int UART_NUM = 1;
  static constexpr int UART_TX_PIN = 18;
  static constexpr int UART_RX_PIN = 19;
  static constexpr int UART_BAUDRATE = 9600;
  static constexpr int BUF_SIZE = 1024;
  
  // 协议常量
  static constexpr uint8_t FRAME_LEN = 4;
  static constexpr uint8_t START_BYTE = 0x99;
  static constexpr uint8_t END_BYTE = 0x66;

    // 触摸组合检测常量
  static constexpr int64_t TOUCH_COMBO_TIMEOUT_US = 700000; // 700ms in microseconds
  
  // 命令处理相关
  enum class CommandType : uint8_t {
      WAKEUP = 0x00,
      TOUCH1 = 0x01,
      TOUCH2 = 0x02,
      TOUCH3 = 0x03,
      TOUCH4 = 0x04,
      TOUCH3A4 = 0x10,
      STOP = 0x20,
      FORCE_TO_LISTEN = 0x21,
      OPEN_BLE = 0x22,
      CLOSE_BLE = 0x23,
      SPK_WAKEWORD_STUDY = 0x30,
      SPK_WAKEWORD_CLR = 0x31,
      SPK_WAKEWORD_STUDY_STOP = 0x32,
      SPK_WAKEWORD_STUDY_SUCCESS = 0x33,
      SPK_WAKEWORD_STUDY_FAIL = 0x34,
      SPK_WAKEWORD_STUDY_FINISH = 0x35,
      SPK_WAKEWORD_STUDY_TIMEOUT = 0x36,
      SPK_WAKEWORD_STUDY_RESET = 0x37,
  };
  
  struct CommandEntry {
      uint8_t type;
      uint8_t id;
      void (*callback)();
  };
  

  // 私有成员
  bool initialized_;
  TaskHandle_t rx_task_handle_;
  static const CommandEntry command_table_[];
  static bool ble_opened;

 
 
  
  // 私有方法
  esp_err_t init_uart();
  static void uart_rx_task(void* arg);
  static void handle_command(uint8_t type, uint8_t id);
  


  // 命令回调函数
  static void on_wakeup();
  static void on_touch1();
  static void on_touch2();
  static void on_touch3();
  static void on_touch4();
  static void on_touch3A4();
  static void on_stop();
  static void on_listening();

  static void spk_wakeword_study();
  static void spk_wakeword_clr();
  static void spk_study_stop();
  static void spk_study_success();
  static void spk_study_fail();
  static void spk_study_finish();
  static void spk_study_timeout();
  static void spk_study_reset();

  static void open_ble();
  static void close_ble();

};

#endif // _TOUCHUAN_UART_H_
