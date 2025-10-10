#include "yt_uart.h"


#include <freertos/task.h>
#include <esp_log.h>
#include <esp_system.h>
#include "driver/uart.h"

#include <cstring>

#include "mcp_server.h"
#include "application.h"
#include "assets/lang_config.h"


#include "system_reset.h"
#include "wifi_board.h"


static const char *TAG = "yt_uart";


bool ytUart::ble_opened = false;



// 参数表
const ytUart::CommandEntry ytUart::command_table_[] = {
    {static_cast<uint8_t>(CommandType::WAKEUP), 0x01, on_wakeup},
    {static_cast<uint8_t>(CommandType::TOUCH1), 0x02, on_touch1},
    {static_cast<uint8_t>(CommandType::TOUCH2), 0x03, on_touch2},
    {static_cast<uint8_t>(CommandType::TOUCH3), 0x04, on_touch3},
    {static_cast<uint8_t>(CommandType::TOUCH4), 0x05, on_touch4},
    {static_cast<uint8_t>(CommandType::TOUCH3A4), 0x11, on_touch3A4},
    {static_cast<uint8_t>(CommandType::STOP), 0x21, on_stop}, // 停止命令，无回调
    {static_cast<uint8_t>(CommandType::FORCE_TO_LISTEN), 0x22, on_listening}, // 强制进入监听状态，无回调
    {static_cast<uint8_t>(CommandType::OPEN_BLE), 0x23, open_ble},
    {static_cast<uint8_t>(CommandType::CLOSE_BLE), 0x24, close_ble},
    {static_cast<uint8_t>(CommandType::SPK_WAKEWORD_STUDY), 0x31, spk_wakeword_study},
    {static_cast<uint8_t>(CommandType::SPK_WAKEWORD_CLR), 0x32, spk_wakeword_clr},
    {static_cast<uint8_t>(CommandType::SPK_WAKEWORD_STUDY_STOP), 0x33, spk_study_stop},
    {static_cast<uint8_t>(CommandType::SPK_WAKEWORD_STUDY_SUCCESS), 0x34, spk_study_success},
    {static_cast<uint8_t>(CommandType::SPK_WAKEWORD_STUDY_FAIL), 0x35, spk_study_fail},
    {static_cast<uint8_t>(CommandType::SPK_WAKEWORD_STUDY_FINISH), 0x36, spk_study_finish},
    {static_cast<uint8_t>(CommandType::SPK_WAKEWORD_STUDY_TIMEOUT), 0x37, spk_study_timeout},
    {static_cast<uint8_t>(CommandType::SPK_WAKEWORD_STUDY_RESET), 0x38, spk_study_reset},
};

// 返回单例实例（C++11 保证线程安全）
ytUart& ytUart::instance() {
    static ytUart inst;
    return inst;
}

// 私有构造：配置并安装 UART 驱动
ytUart::ytUart() : initialized_(false), rx_task_handle_(nullptr) {
    if (init_uart() == ESP_OK) {
        initialized_ = true;
        ESP_LOGI(TAG, "UART initialized successfully");
    } else {
        ESP_LOGE(TAG, "UART initialization failed");
    }
}

ytUart::~ytUart() {
    stop();
}

esp_err_t ytUart::init_uart() {
    uart_config_t cfg = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    esp_err_t ret = uart_param_config(static_cast<uart_port_t>(UART_NUM), &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(static_cast<uart_port_t>(UART_NUM), UART_TX_PIN, UART_RX_PIN, 
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_driver_install(static_cast<uart_port_t>(UART_NUM), BUF_SIZE, BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 创建接收任务
    BaseType_t task_ret = xTaskCreate(uart_rx_task, "uart_rx_task", 4096, this, 10, &rx_task_handle_);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART RX task");
        uart_driver_delete(static_cast<uart_port_t>(UART_NUM));
        return ESP_FAIL;
    }

    return ESP_OK;
}

void ytUart::stop() {
    if (rx_task_handle_) {
        vTaskDelete(rx_task_handle_);
        rx_task_handle_ = nullptr;
    }
    
    if (initialized_) {
        uart_driver_delete(static_cast<uart_port_t>(UART_NUM));
        initialized_ = false;
    }
}

esp_err_t ytUart::protocol_send_data(const char* data, int len) {
    if (!initialized_) {
        ESP_LOGE(TAG, "UART not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!data || len <= 0) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }

    int written = uart_write_bytes(static_cast<uart_port_t>(UART_NUM), data, len);
    if (written != len) {
        ESP_LOGE(TAG, "uart_write_bytes wrote %d/%d bytes", written, len);
        return ESP_FAIL;
    }
    
    ESP_LOGD(TAG, "Sent %d bytes successfully", len);
    return ESP_OK;
}

void ytUart::uart_rx_task(void* arg) {
    //ytUart* instance = static_cast<ytUart*>(arg);
    uint8_t* buf = static_cast<uint8_t*>(malloc(BUF_SIZE));
    
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate RX buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UART RX task started");
    
    while (true) {
        int len = uart_read_bytes(static_cast<uart_port_t>(UART_NUM), buf, BUF_SIZE - 1, 
                                 pdMS_TO_TICKS(20));
        
        if (len > 0) {
            ESP_LOGI(TAG, "Received %d bytes", len);
            
            // 解析数据帧
            for (int i = 0; i <= len - FRAME_LEN; i++) {
                if (buf[i] == START_BYTE && buf[i + 3] == END_BYTE) {
                    uint8_t type = buf[i + 1];
                    uint8_t id = buf[i + 2];
                    

                    handle_command(type, id);
                    i += 3; // 跳过已解析的帧
                }
            }
        }
    }
    
    free(buf);
    vTaskDelete(NULL);
}

void ytUart::handle_command(uint8_t type, uint8_t id) {
    constexpr size_t table_size = sizeof(command_table_) / sizeof(command_table_[0]);
    
    for (size_t i = 0; i < table_size; i++) {
        if (command_table_[i].type == type && command_table_[i].id == id) {
            if (command_table_[i].callback) {
                ESP_LOGI(TAG, "Executing command: type=0x%02X, id=0x%02X", type, id);
                command_table_[i].callback();
            }
            return;
        }
    }
    
    ESP_LOGW(TAG, "Unknown command: type=0x%02X, id=0x%02X", type, id);
}




// 命令回调函数实现
void ytUart::on_wakeup() {
    ESP_LOGI(TAG, "执行唤醒动作...");
    // TODO: 在这里放你唤醒时的业务逻辑
    if(ble_opened == true)
    {
        Application::GetInstance().ForceIdle();
        ESP_LOGI(TAG, "蓝牙连接中，忽略唤醒操作");
        return;
    }
    std::string wake_word="你好";
    Application::GetInstance().WakeWordInvokeByUart(wake_word); 

}

void ytUart::on_touch1() {
    ESP_LOGI(TAG, "执行Touch1对应的动作...");
  
}

void ytUart::on_touch2() {
    ESP_LOGI(TAG, "执行Touch2对应的动作...");
   
   
}


void ytUart::on_touch3() {
    ESP_LOGI(TAG, "执行Touch3对应的动作...");
 
    
}

void ytUart::on_touch4() {
    ESP_LOGI(TAG, "执行Touch4对应的动作...");
  
}

void ytUart::on_touch3A4() {
    ESP_LOGI(TAG, "执行 Touch3 + Touch4 对应的动作...");
 
}


void ytUart::on_stop() {
    ESP_LOGI(TAG, "执行停止动作...");
    Application::GetInstance().ForceIdle();
}

void ytUart::on_listening() {
    ESP_LOGI(TAG, "执行强制进入监听状态动作... ble_opened: %d", ble_opened);
    if(ble_opened == false)
    {
        Application::GetInstance().EnterListeningState();
    }
    else
    {
        Application::GetInstance().ForceIdle();
    }
    
}

void ytUart::spk_wakeword_study() {
    ESP_LOGI(TAG, "执行唤醒词学习动作...");
    Application::GetInstance().ForceIdle();

}

void ytUart::spk_wakeword_clr() {
   // ESP_LOGI(TAG, "执行唤醒词清除动作...");
    ESP_LOGI(TAG, "按键唤醒...");
      // TODO: 在这里放你唤醒时的业务逻辑
    if(ble_opened == true)
    {
        Application::GetInstance().ForceIdle();
        ESP_LOGI(TAG, "蓝牙连接中，忽略唤醒操作");
        return;
    }
    std::string wake_word="你好";
    Application::GetInstance().WakeWordInvokeByUart(wake_word); 
  
}

void ytUart::spk_study_stop() {
   // ESP_LOGI(TAG, "执行唤醒词学习停止动作...");
     ESP_LOGI(TAG, "执行配网动作...");
     static_cast<WifiBoard*>(&Board::GetInstance())->ResetWifiConfiguration();
}

void ytUart::spk_study_success() {
    ESP_LOGI(TAG, "执行唤醒词学习成功动作...");
}

void ytUart::spk_study_fail() {
    ESP_LOGI(TAG, "执行唤醒词学习失败动作...");

}

void ytUart::spk_study_finish() {
    ESP_LOGI(TAG, "执行唤醒词学习完成动作...");

}

void ytUart::spk_study_timeout() {
    ESP_LOGI(TAG, "执行唤醒词学习超时动作...");

}

void ytUart::spk_study_reset() {
    ESP_LOGI(TAG, "执行唤醒词学习重置动作...");

}


void ytUart::open_ble() {
    ESP_LOGI(TAG, "执行打开蓝牙动作...");
    ble_opened = true;
    Application::GetInstance().ForceIdle();
}

void ytUart::close_ble() {
    ESP_LOGI(TAG, "执行关闭蓝牙动作...");
    ble_opened = false;
}


bool ytUart::get_ble_status() {
    ESP_LOGI(TAG, "获取蓝牙状态...");
    return ble_opened;
}